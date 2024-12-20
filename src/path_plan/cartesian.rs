//! Cartesian stroke

use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions, J_TOOL};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{assert_pose_eq, dump_joints, joints};
use bitflags::bitflags;
use nalgebra::{Isometry3, Translation3, Vector3};
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::collections::HashSet;
use std::fmt;
use std::time::Instant;

/// Reasonable default transition costs. Rotation of smaller joints is more tolerable.
/// The sum of all weights is 6.0
pub const DEFAULT_TRANSITION_COSTS: [f64; 6] = [1.2, 1.1, 1.1, 0.9, 0.9, 0.8];

/// Class doing Cartesian planning
pub struct Cartesian<'a> {
    pub robot: &'a KinematicsWithShape,

    /// Check step size in meters. Objects and features of the robotic cell smaller
    /// than this may not be noticed during collision checks.
    pub check_step_m: f64,

    /// Check step size in radians. Objects and features of the robotic cell smaller
    /// than this may not be noticed during collision checks.
    pub check_step_rad: f64,

    /// Maximum allowed transition cost between Joints
    pub max_transition_cost: f64,

    /// Transition cost coefficients (smaller joints are allowed to rotate more)
    pub transition_coefficients: Joints,

    /// If movement between adjacent poses results transition costs over this threshold,
    /// the Cartesian segment is divided by half, checking boths side separatedly while
    /// collision checking also the middle segment.
    pub linear_recursion_depth: usize,

    /// RRT planner that plans the onboarding part, and may potentially plan other
    /// parts that involve collision free relocation of the robot, but without
    /// Cartesian (linear) movement.
    pub rrt: RRTPlanner,

    /// If set, linear interpolated poses are included in the output.
    /// Otherwise, they are discarded, many robots can do Cartesian stroke
    /// much better on they own
    pub include_linear_interpolation: bool,

    /// If set, tool is only checked for collision on joint (RRT) relocations
    /// but not on cartesian stroke. This is necessary if the tool comes into
    /// contact or very close to the target surface when the robot is working
    /// on it during Cartesian strokes.
    pub cartesian_excludes_tool: bool,

    /// Debug mode for logging
    pub debug: bool,
}

bitflags! {
    /// Flags that can be set on AnnotatedJoints in the output
    #[derive(Clone, Copy)]
    pub struct PathFlags: u32 {
        const NONE = 0b0000_0000;

        /// Position is part of the movement from the initil ("home") pose to the landing
        /// pose. It may include very arbitrary joint movements, so Cartesian stroke
        /// may not work on this part of the trajectory.
        const ONBOARDING =          0b0000_0010;
        /// Position directly matches one of the stroke poses given in the input.
        const TRACE =               0b0000_0100;
        /// Position is linear interpolation between two poses of the trace. These poses
        /// are not needed for the robots that have built-in support for Cartesian stroke,
        /// but may be important for more developed models that only rotate between
        /// the given joint positions without guarantees that TCP movement is linear.
        const LIN_INTERP =          0b0000_1000;
        /// Position corresponds the starting pose ("land") that is normally little above
        /// the start of the required trace
        const LAND =                0b0001_0000;

        /// The tool is movind down from the landing position to the first stroke position.
        /// Activate the tool mechanism if any (start the drill, open sprayer, etc).
        const LANDING =             0b0010_0000;

        /// Position corresponds the ennding pose ("park") that is normally little above
        /// the end of the required trace. This is the last pose to include into the
        /// output.
        const PARK =                0b0100_0000;

        /// The tool is moving up from the last trace point to the parking position
        /// (shut down the effector mechanism if any)
        const PARKING =             0b1000_0000;

        /// Combined flag representing the "original" position, so the one that was
        /// given in the input.
        const ORIGINAL = Self::TRACE.bits() | Self::LAND.bits() | Self::PARK.bits();

        /// Special flag used in debugging
        const DEBUG = 0b1000_0000_0000_0000;

        // The movement INTO this pose is Cartesian stroke
        const CARTESIAN = Self::LIN_INTERP.bits() | Self::LAND.bits() | Self::PARK.bits();
    }
}

#[derive(Clone, Copy)]
struct AnnotatedPose {
    pose: Pose,
    flags: PathFlags,
}

/// Annotated joints specifying if it is joint-joint or Cartesian move (to this joint, not from)
#[derive(Clone, Copy)]
pub struct AnnotatedJoints {
    pub joints: Joints,
    pub flags: PathFlags,
}

impl AnnotatedPose {
    pub(crate) fn interpolate(&self, other: &AnnotatedPose, p: f64) -> AnnotatedPose {
        assert!((0.0..=1.0).contains(&p));

        // Interpolate translation (linearly)
        let self_translation = &self.pose.translation.vector;
        let other_translation = &other.pose.translation.vector;

        let translation = self_translation.lerp(&other_translation, p);
        let rotation = self.pose.rotation.slerp(&other.pose.rotation, p);

        AnnotatedPose {
            pose: Pose::from_parts(Translation3::from(translation), rotation),
            flags: PathFlags::LIN_INTERP,
        }
    }
}

fn flag_representation(flags: &PathFlags) -> String {
    const FLAG_MAP: &[(PathFlags, &str)] = &[
        (PathFlags::LIN_INTERP, "LIN_INTERP"),
        (PathFlags::LAND, "LAND"),
        (PathFlags::LANDING, "LANDING"),
        (PathFlags::PARK, "PARK"),
        (PathFlags::PARKING, "PARKING"),
        (PathFlags::TRACE, "TRACE"),
        (PathFlags::CARTESIAN, "CARTESIAN"),
        (PathFlags::ONBOARDING, "ONBOARDING"),
        (PathFlags::DEBUG, "DEBUG"),
    ];

    FLAG_MAP
        .iter()
        .filter(|(flag, _)| flags.contains(*flag))
        .map(|(_, name)| *name)
        .collect::<Vec<_>>()
        .join(" | ")
}
impl fmt::Debug for PathFlags {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let flag_string = flag_representation(self);
        write!(f, "{}", flag_string)
    }
}

impl fmt::Debug for AnnotatedPose {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let translation = self.pose.translation.vector;
        let rotation = self.pose.rotation;
        write!(
            formatter,
            "{}: [{:.3}, {:.3}, {:.3}], quat {{ w: {:.3}, i: {:.3}, j: {:.3}, k: {:.3} }}",
            flag_representation(&self.flags),
            translation.x,
            translation.y,
            translation.z,
            rotation.w,
            rotation.i,
            rotation.j,
            rotation.k
        )
    }
}

impl fmt::Debug for AnnotatedJoints {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{}: {:.2}, {:.2}, {:.2}, {:.2}, {:.2}, {:.2} ",
            flag_representation(&self.flags),
            self.joints[0].to_degrees(),
            self.joints[1].to_degrees(),
            self.joints[2].to_degrees(),
            self.joints[3].to_degrees(),
            self.joints[4].to_degrees(),
            self.joints[5].to_degrees(),
        )
    }
}

struct Transition {
    from: AnnotatedPose,
    to: AnnotatedPose,
    previous: Joints,
    solutions: Solutions,
}

impl Cartesian<'_> {
    pub fn transitionable(&self, from: &Joints, to: &Joints) -> bool {
        utils::transition_costs(from, to, &self.transition_coefficients) <= self.max_transition_cost
    }

    /// Path plan for the given vector of poses. The returned path must be transitionable
    /// and collision free.
    pub fn plan(
        &self,
        from: &Joints,
        land_first: &Option<Pose>,
        steps: &Vec<Pose>,
        park: &Option<Pose>,
    ) -> Result<Vec<AnnotatedJoints>, String> {       
        if self.robot.collides(from) {
            return Err("Onboarding point collides".into());
        }

        let (land, land_flags) = if let Some(land_pose) = land_first {
            println!("Starting from land");
            (land_pose, PathFlags::LAND)
        } else if let Some(step_pose) = steps.first() {
            println!("Starting from trace");
            (step_pose, PathFlags::TRACE)
        } else if let Some(park_pose) = park {
            println!("Starting from park");
            (park_pose, PathFlags::PARKING)
        } else {
            return Ok(vec![]); // No job to do
        };

        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Err("Unable to start from onboarding point".into());
        }
        println!("land_flags {:?}", land_flags);

        let poses = self.with_intermediate_poses(land_first, &steps, park);
        strategies
            .par_iter()
            .find_map_any(|strategy| {
                match self.probe_strategy(
                    from,
                    &AnnotatedJoints {
                        joints: strategy.clone(),
                        flags: land_flags,
                    },
                    &poses,
                ) {
                    Ok(outcome) => Some(Ok(outcome)), // Return success wrapped in Some
                    Err(msg) => {
                        if self.debug {
                            println!("Strategy failed: {}", msg);
                        }
                        None // Continue searching
                    }
                }
            })
            .unwrap_or_else(|| {
                Err(format!(
                    "No strategy worked out of {} tried",
                    strategies.len()
                ))
            })
    }

    /// Computes pose, moved by the distance dz in the local orientation of the Isometry3.
    /// This is used for computing starting and landing poses if the Cartesian path needs
    /// to exclude the tool from collision check (as it touches the working surface during
    /// operation)
    ///
    /// This function takes Option and returns None if None is passed. This is handy
    /// when start and end poses are taken from the list that might be empty
    ///
    /// positive value of z means moving in negative direction (lifting up)
    pub fn elevated_z(isometry: Option<&Isometry3<f64>>, dz: f64) -> Option<Isometry3<f64>> {
        if let Some(isometry) = isometry {
            // Extract the rotation component as a UnitQuaternion
            let rotation = isometry.rotation;

            // Determine the local Z-axis direction (quaternion's orientation)
            let local_z_axis = rotation.transform_vector(&Vector3::z());

            // Compute the new translation by adding dz along the local Z-axis
            let translation = isometry.translation.vector - dz * local_z_axis;

            // Return a new Isometry3 with the updated translation and the same rotation
            Some(Isometry3::from_parts(translation.into(), rotation))
        } else {
            None
        }
    }

    fn probe_strategy(
        &self,
        start: &Joints,
        work_path_start: &AnnotatedJoints,
        poses: &Vec<AnnotatedPose>,
    ) -> Result<Vec<AnnotatedJoints>, String> {

        // Use Rayon to run both functions in parallel
        let (onboarding, stroke) = rayon::join(
            || self.rrt.plan_rrt(start, &work_path_start.joints, self.robot),
            || self.compute_strategy(work_path_start, &poses),
        );

        // Use `?` to propagate errors upwards
        let onboarding = onboarding?;
        let stroke = stroke?;

        // Combine results into `trace`
        let mut trace = Vec::with_capacity(onboarding.len() + stroke.len() + 10);

        // Add onboarding, omitting the last entry
        trace.extend(onboarding.iter().take(onboarding.len() - 1).map(|joints| {
            AnnotatedJoints {
                joints: *joints,
                flags: PathFlags::ONBOARDING,
            }
        }));

        // Add stroke
        trace.extend(stroke);

        Ok(trace)
    }

    /// Probe the given strategy
    fn compute_strategy(
        &self,
        work_path_start: &AnnotatedJoints,
        poses: &Vec<AnnotatedPose>,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        println!("Cartesian planning started");
        let started = Instant::now();
        let excluded_joints = if self.cartesian_excludes_tool {
            // Exclude tool for cartesian
            HashSet::from([J_TOOL])
        } else {
            HashSet::with_capacity(0)
        };

        let mut trace = Vec::with_capacity(100 + poses.len() + 10);
        // Push the strategy point, from here the move must be already CARTESIAN
        trace.push(work_path_start.clone());

        // "Complete" trace with all intermediate poses. Onboarding is not included
        // as rrt planner checks that path itself.
        let mut check_trace = Vec::with_capacity(poses.len());
        let mut step = 1;

        let mut pairs_iterator = poses.windows(2);

        while let Some([from, to]) = pairs_iterator.next() {
            let prev = trace.last().expect("Should have start and strategy points");
            assert_pose_eq(&from.pose, &self.robot.forward(&prev.joints), 1E-5, 1E-5);

            match self.step_adaptive_linear_transition(&prev.joints, from, to, 0) {
                Ok(next) => {
                    // This trace contains all intermediate poses (collision check)
                    check_trace.push(next);

                    if self.include_linear_interpolation
                        || !to.flags.contains(PathFlags::LIN_INTERP)
                    {
                        trace.push(AnnotatedJoints {
                            joints: next,
                            flags: to.flags,
                        });
                    }
                }
                Err(failed_transition) => {
                    self.log_failed_transition(&failed_transition, step);
                    return Err(format!(
                        "Linear stroke does not transit at step {} and cannot be fixed",
                        step
                    )
                    .into());
                }
            }
            if to.flags.contains(PathFlags::TRACE) {
                step += 1;
            }
        }

        if self.debug {
            println!("Cartesian planning till collision check took {:?}", started.elapsed());
        }
        let collides = check_trace
            .par_iter()
            .any(|joints| self.robot.collides_except(joints, &excluded_joints));
        if collides {
            return Err("Collision detected".into());
        }
        if self.debug {
            println!("Cartesian planning took {:?}", started.elapsed());
        }
        Ok(trace)
    }

    // Transition cartesian way from 'from' into 'to' while assuming 'from'
    // Returns the resolved end pose.
    fn step_adaptive_linear_transition(
        &self,
        starting: &Joints,
        from: &AnnotatedPose,
        to: &AnnotatedPose,
        depth: usize,
    ) -> Result<Joints, Transition> {
        if self.debug {
            assert_pose_eq(
                &self.robot.kinematics.forward(starting),
                &from.pose,
                1E-5,
                1E-5,
            );
        }

        pub const DIV_RATIO: f64 = 0.5;

        let midpoint = from.interpolate(to, DIV_RATIO);
        let poses = vec![from, &midpoint, to];

        let mut current = *starting;
        let mut prev_pose = from;
        for pose in poses {
            let mut success = false;

            // Not checked for collisions yet
            let solutions = self
                .robot
                .kinematics
                .inverse_continuing(&pose.pose, &current);

            // Solutions are already sorted best first
            for next in &solutions {
                // Internal "miniposes" generated through recursion are not checked for collision.
                // Outer calling code is reposible for check if 'current' is collision free
                if self.transitionable(&current, next) {
                    success = true;
                    current = *next;
                    break; // break out of solutions loop
                }
            }
            if !success {
                // Try recursive call. If succeeds, assume as done anyway.
                if depth >= self.linear_recursion_depth {
                    return Err(Transition {
                        from: *prev_pose,
                        to: *pose,
                        previous: current,
                        solutions: solutions,
                    });
                }

                // Try to bridge till them middle first, and then from the middle
                // This will result in shorter distance between from and to.
                let till_middle =
                    self.step_adaptive_linear_transition(starting, from, &midpoint, depth + 1)?;
                current =
                    self.step_adaptive_linear_transition(&till_middle, &midpoint, to, depth + 1)?;
            }
            prev_pose = pose;
        }
        Ok(current)
    }

    fn log_failed_transition(&self, transition: &Transition, step: i32) {
        if !self.debug {
            return;
        }
        println!("Step {} from [1..n] failed", step);
        println!(
            "No transition with cost below {}:",
            self.max_transition_cost.to_degrees()
        );
        println!(
            "   from: {:?} collides {}",
            transition.from,
            self.robot.collides(&transition.previous)
        );
        dump_joints(&transition.previous);
        println!("   to: {:?}", transition.to);
        println!("   Possible transitions:");
        for s in &transition.solutions {
            dump_joints(s);
            println!(
                "   transition {} collides: {}",
                utils::transition_costs(&transition.previous, s, &DEFAULT_TRANSITION_COSTS)
                    .to_degrees(),
                self.robot.collides(s)
            );
        }
    }

    fn with_intermediate_poses(
        &self,
        land: &Option<Pose>,
        steps: &Vec<Pose>,
        park: &Option<Pose>,
    ) -> Vec<AnnotatedPose> {
        let mut poses = Vec::with_capacity(10 * steps.len() + 2);

        // Add the landing pose if one provided
        if let Some(land) = land {
            poses.push(AnnotatedPose {
                pose: *land,
                flags: PathFlags::LAND,
            });
        }

        // Add the steps and intermediate poses between them
        for step in steps {
            // There may be previous step or landing pose if provided
            if let Some(last) = poses.last() {
                let last = last.clone();
                self.add_intermediate_poses(&last.pose, &step, &mut poses, last.flags);
            }
            poses.push(AnnotatedPose {
                pose: step.clone(),
                flags: PathFlags::TRACE,
            });
        }

        // Add the parking pose if one provided
        if let (Some(park), Some(last)) = (park, poses.last()) {
            self.add_intermediate_poses(
                &last.pose.clone(),
                &park.clone(),
                &mut poses,
                PathFlags::PARK,
            );
            poses.push(AnnotatedPose {
                pose: park.clone(),
                flags: PathFlags::PARK,
            });
        }

        poses
    }

    /// Add intermediate poses. start and end poses are not added.
    fn add_intermediate_poses(
        &self,
        start: &Pose,
        end: &Pose,
        poses: &mut Vec<AnnotatedPose>,
        flags: PathFlags,
    ) {
        // Calculate the translation difference and distance
        let translation_diff = end.translation.vector - start.translation.vector;
        let translation_distance = translation_diff.norm();

        // Calculate the rotation difference and angle
        let rotation_diff = end.rotation * start.rotation.inverse();
        let rotation_angle = rotation_diff.angle();

        // Calculate the number of steps required for translation and rotation
        let translation_steps = (translation_distance / self.check_step_m).ceil() as usize;
        let rotation_steps = (rotation_angle / self.check_step_rad).ceil() as usize;

        // Choose the greater step count to achieve finer granularity between poses
        let steps = translation_steps.max(rotation_steps).max(1);

        // Calculate incremental translation and rotation per chosen step count
        let translation_step = translation_diff / steps as f64;

        // Generate each intermediate pose. Start and end poses are excluded.
        for i in 1..steps {
            let fraction = i as f64 / steps as f64;

            // Interpolate translation and rotation
            let intermediate_translation = start.translation.vector + translation_step * i as f64;
            let intermediate_rotation = start.rotation.slerp(&end.rotation, fraction);

            // Construct the intermediate pose
            let intermediate_pose =
                Pose::from_parts(intermediate_translation.into(), intermediate_rotation);

            poses.push(AnnotatedPose {
                pose: intermediate_pose,
                flags: flags | PathFlags::LIN_INTERP,
            });
        }
    }
}

//! Cartesian stroke

use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{dump_joints, transition_costs};
use bitflags::bitflags;
use nalgebra::Translation3;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
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
        const ONBOARDING =          1 << 1;

        /// Position directly matches one of the stroke poses given in the input.
        const TRACE =               1 << 2;

        /// Position is a linear interpolation between two poses of the trace. These poses
        /// are not needed for the robots that have built-in support for Cartesian stroke,
        /// but may be important for more developed models that only rotate between
        /// the given joint positions without guarantees that TCP movement is linear.
        const LIN_INTERP =          1 << 3;

        /// Position corresponds the starting pose ("land") that is normally little above
        /// the start of the required trace
        const LAND =                1 << 4;

        /// The tool is movind down from the landing position to the first stroke position.
        /// Activate the tool mechanism if any (start the drill, open sprayer, etc).
        const LANDING =             1 << 5;

        /// Position corresponds the ennding pose ("park") that is normally little above
        /// the end of the required trace. This is the last pose to include into the
        /// output.
        const PARK =                1 << 6;

        /// The tool is moving up from the last trace point to the parking position
        /// (shut down the effector mechanism if any)
        const PARKING =             1 << 7;

        /// Used with raster projector, indicates the movement considered "forwards"
        const FORWARDS =             1 << 8;

        /// Used with raster projector, indicates the movement considered "backwards"
        const BACKWARDS =            1 << 9;

        /// Mildly altered to make the stroke possible
        const ALTERED =              1 << 10;

        /// Combined flag representing the "original" position, so the one that was
        /// given in the input.
        const ORIGINAL = Self::TRACE.bits() | Self::LAND.bits() | Self::PARK.bits();

        /// Special flag used in debugging to mark out anything of interest. Largest can be stored
        /// in u32
        const DEBUG = 1 << 31;

        // The movement INTO this pose is Cartesian stroke
        const CARTESIAN = Self::LIN_INTERP.bits() | Self::LAND.bits() | Self::PARK.bits();
    }
}

#[derive(Clone, Copy)]
pub(crate) struct AnnotatedPose {
    pub (crate) pose: Pose,
    pub (crate) flags: PathFlags,
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
        (PathFlags::PARK, "PARK"),
        (PathFlags::TRACE, "TRACE"),
        (PathFlags::CARTESIAN, "CARTESIAN"),
        (PathFlags::ONBOARDING, "ONBOARDING"),
    ];

    FLAG_MAP
        .iter()
        .filter(|(flag, _)| flags.contains(*flag))
        .map(|(_, name)| *name)
        .collect::<Vec<_>>()
        .join(" | ")
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

    /// Path plan for the given vector of poses. The returned path must be transitionable
    /// and collision free.
    pub fn plan(
        &self,
        from: &Joints,
        land: &Pose,
        steps: Vec<Pose>,
        park: &Pose,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        if self.robot.collides(from) {
            return Err("Onboarding point collides".into());
        }
        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Err("Unable to start from onboarding point".into());
        }
        let poses = self.with_intermediate_poses(land, &steps, park);
        println!("Probing {} strategies", strategies.len());

        let stop = Arc::new(AtomicBool::new(false));

        strategies
            .par_iter()
            .find_map_any(|strategy| {
                match self.probe_strategy(
                    strategy, &poses, &stop
                ) {
                    Ok(outcome) => {
                        println!("Strategy worked out: {:?}", strategy);
                        stop.store(true, Ordering::Relaxed);
                        Some(Ok(outcome))
                    }
                    Err(msg) => {
                        if self.debug {
                            println!("Strategy failed: {:?}, {}", strategy, msg);
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

    /// Probe the given strategy
    fn probe_strategy(
        &self,
        work_path_start: &Joints,
        poses: &Vec<AnnotatedPose>,
        stop: &AtomicBool,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        println!("Cartesian planning started, computing strategy {work_path_start:?}");

        let started = Instant::now();
        let mut trace = Vec::with_capacity(100 + poses.len() + 10);
        // Push the strategy point, from here the move must be already CARTESIAN
        trace.push(AnnotatedJoints {
            joints: *work_path_start,
            flags: PathFlags::LAND, 
        }.clone());

        let mut step = 1;

        let mut pairs_iterator = poses.windows(2);

        while let Some([from, to]) = pairs_iterator.next() {
            let prev = trace
                .last()
                .expect("Should have start and strategy points")
                .clone();

            let transition = self.step_adaptive_linear_transition(&prev.joints, from, to, 0);
            match transition {
                Ok(extension) => {
                    for (p, step) in extension.iter().enumerate() {
                        let flags = if p < extension.len() - 1 {
                            (to.flags | PathFlags::LIN_INTERP)
                                & !(PathFlags::TRACE | PathFlags::PARK)
                        } else {
                            to.flags
                        };
                        trace.push(AnnotatedJoints {
                            joints: step.clone(),
                            flags: flags,
                        });
                    }
                }

                Err(failed_transition) => {
                    let mut success = false;
                    // Try with altered pose
                    println!(
                        "Closing step {:?} with RRT", step
                    );
                    let solutions = self.robot.inverse_continuing(&to.pose, &prev.joints);
                    for next in solutions {
                        let path = self.rrt.plan_rrt(&prev.joints, &next, self.robot, stop);
                        if let Ok(path) = path {
                            println!("  ... closed with RRT {} steps", path.len());
                            for step in path {
                                trace.push(AnnotatedJoints {
                                    joints: step,
                                    flags: to.flags & !PathFlags::LIN_INTERP,
                                });
                            }
                            success = true;
                            break;
                        }
                    }

                    if !success {
                        self.log_failed_transition(&failed_transition, step);
                        return Err(format!("Failed to transition at step {}", step));
                    }
                }
            }
            if to.flags.contains(PathFlags::TRACE) {
                step += 1;
            }
        }

        if self.debug {
            println!(
                "Cartesian planning till collision check took {:?}",
                started.elapsed()
            );
        }
        if stop.load(Ordering::Relaxed) {
            return Err("Stopped".into());
        }

        Ok(trace)
    }

    /// Transition cartesian way from 'from' into 'to' while assuming 'from'
    /// Returns all path, not including "starting", that should not be empty
    /// as it succeeded. Returns description of the transition on failure.
    fn step_adaptive_linear_transition(
        &self,
        starting: &Joints,
        from: &AnnotatedPose, // FK of "starting"
        to: &AnnotatedPose,
        depth: usize,
    ) -> Result<Vec<Joints>, Transition> {
        pub const DIV_RATIO: f64 = 0.5;

        // Not checked for collisions yet
        let solutions = self
            .robot
            .kinematics
            .inverse_continuing(&to.pose, &starting);

        // Solutions are already sorted best first
        for next in &solutions {
            // Internal "miniposes" generated through recursion are not checked for collision.
            // They only check agains continuity of the robot movement (no unexpected jerks)
            let cost = transition_costs(starting, next, &self.transition_coefficients);
            if cost <= self.max_transition_cost {
                return Ok(vec![next.clone()]); // Track minimal cost observed
            }
        }

        // Transitioning not successful.
        // Recursive call reduces step, the goal is to check if there is a continuous
        // linear path on any step.
        if depth < self.linear_recursion_depth {
            // Try to bridge till them middle first, and then from the middle
            // This will result in a shorter distance between from and to.
            let midpose = from.interpolate(to, DIV_RATIO);
            let first_track =
                self.step_adaptive_linear_transition(starting, from, &midpose, depth + 1)?;
            let mid_step = first_track.last().unwrap().clone();

            // If both bridgings were successful, return the final position that resulted from
            // bridging from middle joints to the final pose on this step
            let second_track =
                self.step_adaptive_linear_transition(&mid_step, &midpose, to, depth + 1)?;

            Ok(first_track
                .into_iter()
                .chain(second_track.into_iter())
                .collect())
        } else {
            Err(Transition {
                from: from.clone(),
                to: to.clone(),
                previous: starting.clone(),
                solutions: solutions,
            })
        }
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
        land: &Pose,
        steps: &Vec<Pose>,
        park: &Pose,
    ) -> Vec<AnnotatedPose> {
        let mut poses = Vec::with_capacity(10 * steps.len() + 2);

        // Add the landing pose
        poses.push(AnnotatedPose {
            pose: *land,
            flags: PathFlags::LAND,
        });

        if !steps.is_empty() {
            // Add intermediate poses between land and the first step
            self.add_intermediate_poses(land, &steps[0], &mut poses);

            // Add the steps and intermediate poses between them
            for i in 0..steps.len() - 1 {
                poses.push(AnnotatedPose {
                    pose: steps[i].clone(),
                    flags: PathFlags::TRACE,
                });

                self.add_intermediate_poses(&steps[i], &steps[i + 1], &mut poses);
            }

            // Add the last step
            let last = *steps.last().unwrap();
            poses.push(AnnotatedPose {
                pose: last.clone(),
                flags: PathFlags::TRACE,
            });

            // Add intermediate poses between the last step and park
            self.add_intermediate_poses(&last, park, &mut poses);
        } else {
            // If no steps, add intermediate poses between land and park directly
            self.add_intermediate_poses(land, park, &mut poses);
        }

        // Add the parking pose
        poses.push(AnnotatedPose {
            pose: park.clone(),
            flags: PathFlags::PARK,
        });

        poses
    }

    /// Add intermediate poses. start and end poses are not added.
    fn add_intermediate_poses(&self, start: &Pose, end: &Pose, poses: &mut Vec<AnnotatedPose>) {
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
                flags: PathFlags::LIN_INTERP,
            });
        }
    }
}

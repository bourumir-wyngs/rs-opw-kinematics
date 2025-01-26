//! Cartesian stroke

use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{assert_pose_eq, dump_joints, transition_costs};
use bitflags::bitflags;
use nalgebra::Translation3;
use rayon::prelude::{IntoParallelRefIterator, ParallelIterator};
use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use crate::altered_pose::alter_poses;

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
                    from, strategy, &poses, &stop
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
        start: &Joints,
        strategy: &Joints,
        poses: &Vec<AnnotatedPose>,
        stop: &Arc<AtomicBool>,
    ) -> Result<Vec<AnnotatedJoints>, String> {
        let onboarding = self.rrt.plan_rrt(start, strategy, self.robot, stop)?;
        if stop.load(Ordering::Relaxed) {
            // Do not attempt Cartesian planning if we already cancelled.
            return Err("Stopped".into());
        }

        let mut trace = Vec::with_capacity(onboarding.len() + poses.len() + 10);

        // Do not take the last value as it is same as 'strategy'
        for joints in onboarding.iter().take(onboarding.len() - 1) {
            trace.push(AnnotatedJoints {
                joints: *joints,
                flags: PathFlags::ONBOARDING,
            });
        }

        // Push the strategy point, from here the move must be already CARTESIAN
        trace.push(AnnotatedJoints {
            joints: *strategy,
            flags: PathFlags::TRACE | PathFlags::CARTESIAN
        });

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
                        let flags = if to.flags.contains(PathFlags::PARK) {
                            // At the end of the stroke, we do not assume CARTESIAN
                            PathFlags::TRACE | to.flags
                        } else {
                            PathFlags::TRACE | PathFlags::CARTESIAN | to.flags
                        };

                        trace.push(AnnotatedJoints {
                            joints: next,
                            flags: flags
                        });
                    }
                }
                Err(failed_transition) => {
                    let mut success = false;
                    // Try with a slightly altered pose.
                    for altered_to in alter_poses(to, self.check_step_rad) {
                        match self.step_adaptive_linear_transition(
                            &prev.joints,
                            from,
                            &altered_to,
                            0,
                        ) {
                            Ok(next) => {
                                println!(
                                    "    Transition with altered pose {:?} successful",
                                    altered_to
                                );
                                check_trace.push(next);
                                trace.push(AnnotatedJoints {
                                    joints: next,
                                    flags: altered_to.flags,
                                });
                                success = true;
                                break;
                            }
                            Err(_) => { // Try next
                            }
                        }
                    }

                    if !success {
                        self.log_failed_transition(&failed_transition, step);
                        return Err(format!(
                            "Failed to transition at step {} with all alterations tried",
                            step
                        ));
                    }
                }
            }
            if to.flags.contains(PathFlags::TRACE) {
                step += 1;
            }
        }

        let collides = check_trace
            .par_iter()
            .any(|joints| self.robot.collides(joints));
        if collides {
            return Err("Collision detected".into());
        }
        // If all good, cancel other tasks still running
        stop.store(true, Ordering::Relaxed);
        Ok(trace)
    }

    // Transition cartesian way from 'from' into 'to' while assuming 'from'
    // Returns the resolved end pose.
    // Transition cartesian way from 'from' into 'to' while assuming 'from'
    // Returns the resolved end pose.
    fn step_adaptive_linear_transition(
        &self,
        starting: &Joints,
        from: &AnnotatedPose, // FK of "starting"
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

        // Not checked for collisions yet
        let solutions = self.robot.kinematics.inverse_continuing(&to.pose, &starting);
        let mut cheapest_cost = std::f64::MAX;        

        // Solutions are already sorted best first
        for next in &solutions {
            // Internal "miniposes" generated through recursion are not checked for collision.
            // They only check agains continuity of the robot movement (no unexpected jerks)
            let cost = transition_costs(starting, next, &self.transition_coefficients);
            if cost <= self.max_transition_cost {
                return Ok(next.clone()); // Track minimal cost observed
            }
            cheapest_cost = cheapest_cost.min(cost);            
        }

        // Transitioning not successful.
        // Recursive call reduces step, the goal is to check if there is a continuous
        // linear path on any step.
        if depth < self.linear_recursion_depth {
            // Try to bridge till them middle first, and then from the middle
            // This will result in a shorter distance between from and to.
            let midpose = from.interpolate(to, DIV_RATIO);
            let middle_joints =
                self.step_adaptive_linear_transition(starting, from, &midpose, depth + 1)?;

            // If both bridgings were successful, return the final position that resulted from
            // bridging from middle joints to the final pose on this step
            let end_step =
                self.step_adaptive_linear_transition(&middle_joints, &midpose, to, depth + 1)?;

            let cost_start_mid =
                transition_costs(starting, &middle_joints, &self.transition_coefficients);
            let cost_mid_end =
                transition_costs(&middle_joints, &end_step, &self.transition_coefficients);
            if cost_start_mid <= self.max_transition_cost
                && cost_mid_end <= self.max_transition_cost
            {
                // Both steps are very small
                return Ok(end_step);
            }

            // Cost of any step separately should not be more than cost of the whole transition
            if cost_start_mid <= cheapest_cost && cost_mid_end <= cheapest_cost {
                // Otherwise only recognize if the cost does not rise in the intermediate steps.
                return Ok(end_step);
            }
            return Err(Transition {
                from: from.clone(),
                to: to.clone(),
                previous: starting.clone(),
                solutions: solutions,
            });
        }

        Err(Transition {
            from: from.clone(),
            to: to.clone(),
            previous: starting.clone(),
            solutions: solutions,
        })
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

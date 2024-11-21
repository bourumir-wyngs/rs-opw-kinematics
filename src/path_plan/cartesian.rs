//! Cartesian stroke
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{assert_pose_eq, dump_joints, dump_solutions};
use bitflags::bitflags;
use nalgebra::Translation3;
use rayon::prelude::*;
use std::fmt;
use std::fmt::Formatter;

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

    pub rrt: RRTPlanner,

    /// If set, linear interpolated poses are included in the output.
    /// Otherwise, they are discarded, many robots can do Cartesian stroke
    /// much better on they own
    pub include_linear_interpolation: bool,

    /// Debug mode for logging
    pub debug: bool,
}

bitflags! {
    #[derive(Clone, Copy)]
    struct PoseFlags: u32 {
        const TRACE = 0b00000001;
        const LINEAR_INTERPOLATED = 0b00000010;
        const LAND = 0b00000100;
        const PARK = 0b00001000;

        const ORIGINAL = Self::TRACE.bits() | Self::LAND.bits() | Self::PARK.bits();
    }
}

#[derive(Clone, Copy)]
struct AnnotatedPose {
    pub pose: Pose,
    pub flags: PoseFlags,
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
            flags: PoseFlags::LINEAR_INTERPOLATED,
        }
    }
}

impl fmt::Debug for AnnotatedPose {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        fn flag_representation(flags: &PoseFlags) -> String {
            const FLAG_MAP: &[(PoseFlags, &str)] = &[
                (PoseFlags::LINEAR_INTERPOLATED, "LINEAR_INTERPOLATED"),
                (PoseFlags::LAND, "LAND"),
                (PoseFlags::PARK, "PARK"),
                (PoseFlags::TRACE, "TRACE"),
            ];

            FLAG_MAP
                .iter()
                .filter(|(flag, _)| flags.contains(*flag))
                .map(|(_, name)| *name)
                .collect::<Vec<_>>()
                .join(" | ")
        }

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
    pub fn plan_sequential(
        &self,
        from: &Joints,
        land: &Pose,
        steps: Vec<Pose>,
        park: &Pose,
    ) -> Vec<Joints> {
        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Vec::new();
        }
        let poses = self.with_intermediate_poses(land, &steps, park);
        for strategy in &strategies {
            match self.probe_strategy(from, strategy, &poses) {
                Ok(outcome) => {
                    return outcome;
                }
                Err(msg) => {
                    // Continue next
                }
            }
        }
        Vec::new()
    }

    /// Probe the given strategy
    fn probe_strategy(
        &self,
        start: &Joints,
        strategy: &Joints,
        poses: &Vec<AnnotatedPose>,
    ) -> Result<Vec<Joints>, String> {
        let mut trace = self.rrt.plan_rrt(start, strategy, self.robot)?;
        trace.reserve(poses.len());

        for step in poses.windows(2) {
            let (from, to) = (&step[0], &step[1]);
            let prev = trace.last().expect("Should have start and strategy points");
            match self.step_adaptive_linear_transition(prev, from, to) {
                Ok(next) => {
                    if self.include_linear_interpolation
                        || !to.flags.contains(PoseFlags::LINEAR_INTERPOLATED)
                    {
                        trace.push(next);
                    }
                }
                Err(failed_transition) => {
                    self.log_failed_transition(&failed_transition);
                    return Err("Failed strategy".into());
                }
            }
        }

        Ok(trace)
    }

    // Transition cartesian way from 'from' into 'to' while assuming 'from'
    // is represented by 'starting'.
    // Returns the resolved end pose.
    fn step_adaptive_linear_transition(
        &self,
        starting: &Joints,
        from: &AnnotatedPose,
        to: &AnnotatedPose,
    ) -> Result<Joints, Transition> {
        assert_pose_eq(
            &self.robot.kinematics.forward(starting),
            &from.pose,
            1E-5,
            1E-5,
        );

        let midpoint = from.interpolate(to, 0.5);
        let poses = vec![from, &midpoint, to];

        let mut current = *starting;
        let mut prev_pose = &from;
        for pose in poses.iter() {
            let mut success = false;
            let solutions = self
                .robot
                .kinematics
                .inverse_continuing(&pose.pose, &current);

            // Solutions are already sorted best first
            for next in &solutions {
                if !self.robot.collides(next) && self.transitionable(&current, next) {
                    success = true;
                    current = *next;
                    break; // break out of solutions loop
                }
            }
            if !success {
                return Err(Transition {
                    from: **prev_pose,
                    to: **pose,
                    previous: current,
                    solutions: solutions,
                });
            }
            prev_pose = pose;
        }
        Ok(current)
    }

    fn log_failed_transition(&self, transition: &Transition) {
        if !self.debug {
            return;
        }
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
            pose: land.clone(),
            flags: PoseFlags::LAND,
        });

        if !steps.is_empty() {
            // Add intermediate poses between land and the first step
            self.add_intermediate_poses(land, &steps[0], &mut poses);

            // Add the steps and intermediate poses between them
            for i in 0..steps.len() - 1 {
                poses.push(AnnotatedPose {
                    pose: steps[i].clone(),
                    flags: PoseFlags::TRACE,
                });

                self.add_intermediate_poses(&steps[i], &steps[i + 1], &mut poses);
            }

            // Add the last step
            let last = *steps.last().unwrap();
            poses.push(AnnotatedPose {
                pose: last.clone(),
                flags: PoseFlags::TRACE,
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
            flags: PoseFlags::PARK,
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
                flags: PoseFlags::LINEAR_INTERPOLATED,
            });
        }
    }
}

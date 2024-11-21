//! Cartesian stroke

use crate::chunked_vector::ChunkedVec;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt::RRTPlanner;
use crate::utils;
use crate::utils::{dump_joints, dump_solutions};
use bitflags::bitflags;
use rayon::prelude::*;
use std::fmt;

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

struct AnnotatedPose {
    pub pose: Pose,
    pub flags: PoseFlags,
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
        from: &Joints,
        strategy: &Joints,
        poses: &ChunkedVec<AnnotatedPose>,
    ) -> Result<Vec<Joints>, String> {
        let mut trace = self.rrt.plan_rrt(from, strategy, self.robot)?;

        // 'strategy' does not need to be pushed, start and goal are included in rrt plan.
        let mut step = 0;
        let mut substep = 0;
        let mut previous_pose = None;
        for pose in poses.iter() {
            if step == 0 && substep == 0 {
                println!("Starting with {:?}", pose);
            }
            let mut success = false;
            let previous = *trace.last().expect("Should have at least start_from");
            // Ik without collision checking but with constraint checking.
            let solutions = self
                .robot
                .kinematics
                .inverse_continuing(&pose.pose, &previous);
            // Solutions are already sorted best first
            for last in &solutions {
                // Transition and collision checks.
                if !self.robot.collides(last) && self.transitionable(&previous, last) {
                    if self.include_linear_interpolation
                        || !pose.flags.contains(PoseFlags::LINEAR_INTERPOLATED)
                    {
                        // Linear interpolation can be discarded after checking.
                        trace.push(*last);
                    }                    
                    success = true;
                    self.log_successful_transition(&mut step, &mut substep, pose);                    
                    previous_pose = Some(pose);
                    break; // break out of solutions loop
                }
            }

            if !success {
                self.log_failed_transition(
                    &mut step,
                    &mut substep,
                    &mut previous_pose,
                    pose,
                    &previous,
                    &solutions,
                );
                return Err("Strategy unsuccessful".into()); // This strategy was not successful.
            }
        }
        Ok(trace)
    }

    fn log_successful_transition(&self, step: &mut i32, substep: &mut i32, pose: &AnnotatedPose) {
        if pose.flags.intersects(PoseFlags::ORIGINAL) {
            if self.debug {
                println!(
                    "{} done with {} substeps. Master transition into step {}",
                    step,
                    substep,
                    *step + 1,
                );
            }
            *substep = 0;
            *step += 1;
        } else {
            println!("    step {} {}", step, substep);
            *substep += 1;
        }
    }

    fn log_failed_transition(
        &self,
        step: &mut i32,
        substep: &mut i32,
        previous_pose: &mut Option<&AnnotatedPose>,
        pose: &AnnotatedPose,
        previous: &Joints,
        solutions: &Solutions,
    ) {
        if !self.debug {
            return;
        }
        println!(
            "No transition at step {} substep {} with cost below {}:",
            step,
            substep,
            self.max_transition_cost.to_degrees()
        );
        println!(
            "   from: {:?} collides {}",
            previous_pose,
            self.robot.collides(&previous)
        );
        dump_joints(&previous);
        println!("   to: {:?}", pose);
        println!("   Possible transitions:");
        for s in solutions {
            dump_joints(s);
            println!(
                "   transition {} collides: {}",
                utils::transition_costs(&previous, s, &DEFAULT_TRANSITION_COSTS).to_degrees(),
                self.robot.collides(s)
            );
        }
    }

    fn with_intermediate_poses(
        &self,
        land: &Pose,
        steps: &Vec<Pose>,
        park: &Pose,
    ) -> ChunkedVec<AnnotatedPose> {
        let mut poses = ChunkedVec::new(10 * steps.len() + 2);

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
    fn add_intermediate_poses(
        &self,
        start: &Pose,
        end: &Pose,
        poses: &mut ChunkedVec<AnnotatedPose>,
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
                flags: PoseFlags::LINEAR_INTERPOLATED,
            });
        }
    }
}

//! Cartesian stroke
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;
use bitflags::bitflags;
use crate::chunked_vector::ChunkedVec;
use crate::utils;

use rayon::prelude::*;
use crate::rrt::RRTPlanner;

/// Reasonable default transition costs. Rotation of smaller joints is more tolerable.
/// The sum of all weights is 6.0
pub const DEFAULT_TRANSITION_COSTS: [f64; 6] = [1.2, 1.1, 1.1, 0.9, 0.9, 0.8];

/// Class doing Cartesian planning
pub(crate) struct Cartesian<'a> {
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
    transition_coefficients: Joints,

    pub rrt: RRTPlanner,

    /// If set, linear interpolated poses are included in the output.
    /// Otherwise, they are discarded, many robots can do Cartesian stroke
    /// much better on they own
    pub include_linear_interpolation: bool,

    /// Debug mode for logging
    pub debug: bool,
}


bitflags! {
    struct PoseFlags: u32 {
        const ORIGINAL = 0b00000001;
        const LINEAR_INTERPOLATED = 0b00000010;   
        const LAND = 0b00000100;
        const PARK = 0b00001000;
    }
}

struct AnnotatedPose {
    pub pose: Pose,
    pub flags: PoseFlags,
}

impl Cartesian<'_> {
    pub fn transitionable(&self, from: &Joints, to: &Joints) -> bool {
        utils::transition_costs(from, to, &self.transition_coefficients)
            <= self.max_transition_cost
    }

    /// Path plan for the given vector of poses. The returned path must be transitionable
    /// and collision free. 
    pub fn plan_sequential(&self, from: &Joints, land: &Pose, steps: Vec<Pose>, park: &Pose) -> Vec<Joints> {
        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Vec::new();
        }
        let poses = self.with_intermediate_poses(land, &steps, park);
        for strategy in &strategies {
            let outcome = self.probe_strategy(from, strategy, &poses);
            if !outcome.is_empty() {
                return outcome;
            }
        }
        Vec::new()
    }

    pub fn plan(&self, from: &Joints, land: &Pose, steps: Vec<Pose>, park: &Pose) -> Vec<Joints> {
        // strategies will be collision free and constraint compliant
        let strategies = self.robot.inverse_continuing(land, from);
        if strategies.is_empty() {
            return Vec::new();
        }
        let poses = self.with_intermediate_poses(land, &steps, park);

        strategies
            .par_iter() // Parallel iterator over strategies
            .map(|strategy| self.probe_strategy(from, strategy, &poses)) // Probe each strategy
            .find_any(|outcome| !outcome.is_empty()) // Find the first non-empty outcome
            .unwrap_or_else(Vec::new) // Return the result or an empty vector if none
    }


    /// Probe the given strategy
    pub fn probe_strategy(&self, from: &Joints, strategy: &Joints,
                          poses: &ChunkedVec<AnnotatedPose>) -> Vec<Joints> {
        let mut trace = Vec::new();

        // Plan collision free 'onboarding' from the home position
        match self.rrt.plan_rrt(from, strategy, self.robot) {
            Ok(joints) => {
                if joints.is_empty() {
                    if self.debug {
                        eprintln!("Failed to onboard, no plan");
                        return Vec::new();
                    }
                }
                trace = joints;
                trace.reserve(poses.len() + 2);
                if self.debug {
                    println!("Onboarding {} steps", trace.len());
                    return Vec::new();
                }
            }
            Err(err) => {
                if self.debug {
                    eprintln!("Failed to onboard, error: {}", err);
                }
                return Vec::new(); // Failed to onboard
            }
        }

        // 'strategy' does not need to be pushed, start and goal are included in rrt plan.
        for pose in poses.iter() {
            let success;
            let previous = trace.last().expect("Should have at least start_from");
            // Ik without collision checking but with constraint checking.
            let solutions =
                self.robot.kinematics.inverse_continuing(&pose.pose, previous);
            // Solutions are already sorted best first
            if let Some(last) = solutions.last() {
                // Transition and collision checks.
                if self.transitionable(previous, last) && !self.robot.collides(last) {
                    if self.include_linear_interpolation ||
                        !pose.flags.contains(PoseFlags::LINEAR_INTERPOLATED) {
                        // Linear interpolation can be discarded after checking.
                        trace.push(*last);
                    }
                    success = true;
                } else {
                    success = false;
                }
            } else {
                success = false;
            }
            if !success {
                return Vec::new(); // This strategy was not successful.
            }
        }
        trace
    }

    pub fn with_intermediate_poses(&self, land: &Pose, steps: &Vec<Pose>, park: &Pose)
                                   -> ChunkedVec<AnnotatedPose> {
        let mut poses = ChunkedVec::new(10 * steps.len() + 2);

        // Add the landing pose
        poses.push(AnnotatedPose {
            pose: land.clone(),
            flags: PoseFlags::LAND | PoseFlags::ORIGINAL,
        });

        if !steps.is_empty() {
            // Add intermediate poses between land and the first step
            self.add_intermediate_poses(land, &steps[0], &mut poses);

            // Add the steps and intermediate poses between them
            for i in 0..steps.len() - 1 {
                poses.push(AnnotatedPose {
                    pose: steps[i].clone(),
                    flags: PoseFlags::ORIGINAL,
                });

                self.add_intermediate_poses(&steps[i], &steps[i + 1], &mut poses);
            }

            // Add the last step
            let last = *steps.last().unwrap();
            poses.push(AnnotatedPose {
                pose: last.clone(),
                flags: PoseFlags::ORIGINAL,
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
            flags: PoseFlags::PARK | PoseFlags::ORIGINAL,
        });

        poses
    }

    pub fn check_collisions(&self, poses: &mut ChunkedVec<AnnotatedPose>) -> bool {
        todo!()
    }

    /// Add intermediate poses. start and end poses are not added.
    pub fn add_intermediate_poses(
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
            let intermediate_pose = Pose::from_parts(
                intermediate_translation.into(),
                intermediate_rotation,
            );

            poses.push(AnnotatedPose {
                pose: intermediate_pose,
                flags: PoseFlags::LINEAR_INTERPOLATED,
            });
        }
    }
}
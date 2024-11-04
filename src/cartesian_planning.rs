extern crate nalgebra as na;
use na::{Isometry3, Vector3, UnitQuaternion};
use std::hash::{Hash, Hasher};
use rayon::prelude::IntoParallelRefIterator;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;

use rayon::prelude::*;

const GRID_XYZ: f64 = 0.001;  // positional grid cell size
const GRID_QUAT: f64 = 0.01;  // rotational grid cell size

#[derive(Debug)]
pub struct ComplexAlternative {
    pub solutions: Solutions,
    pub pose: Pose,
}

impl PartialEq for ComplexAlternative {
    fn eq(&self, other: &Self) -> bool {
        self.pose.translation == other.pose.translation && self.pose.rotation == other.pose.rotation
    }
}

impl Eq for ComplexAlternative {}

impl Hash for ComplexAlternative {
    fn hash<H: Hasher>(&self, state: &mut H) {

        // Quantize the translation components and hash
        let translation = self.pose.translation.vector;
        ((translation.x / GRID_XYZ).round() as i64).hash(state);
        ((translation.y / GRID_XYZ).round() as i64).hash(state);
        ((translation.z / GRID_XYZ).round() as i64).hash(state);

        // Quantize the rotation quaternion components and hash
        let rotation = self.pose.rotation;
        ((rotation.i / GRID_QUAT).round() as i64).hash(state);
        ((rotation.j / GRID_QUAT).round() as i64).hash(state);
        ((rotation.k / GRID_QUAT).round() as i64).hash(state);
        ((rotation.w / GRID_QUAT).round() as i64).hash(state);
    }
}

pub fn transition_costs(joints1: &Joints, joints2: &Joints) -> f64 {
    joints1.iter()
        .zip(joints2.iter())
        .map(|(a, b)| (a - b).abs())
        .sum()
}

impl ComplexAlternative {
    pub fn transition_costs(&self, other: &ComplexAlternative) -> f64 {
        // Initialize the minimum cost to a large value
        let mut min_cost = f64::INFINITY;

        // Iterate over each solution in self and other to find the minimum transition cost
        for joints1 in &self.solutions {
            for joints2 in &other.solutions {
                // Calculate the transition cost between joints1 and joints2
                let cost: f64 = joints1.iter()
                    .zip(joints2.iter())
                    .map(|(a, b)| (a - b).abs())
                    .sum();

                // Update the minimum cost if a lower cost is found
                if cost < min_cost {
                    min_cost = cost;
                }
            }
        }

        min_cost
    }

    pub fn branch_single_threaded(&self, kinematics: &KinematicsWithShape) -> Vec<ComplexAlternative> {
        let mut branches = Vec::with_capacity(12);

        // Generate a grid of positions around the current pose's translation
        for dx in (-1..=1).map(|i| i as f64 * GRID_XYZ) {
            for dy in (-1..=1).map(|i| i as f64 * GRID_XYZ) {
                for dz in (-1..=1).map(|i| i as f64 * GRID_XYZ) {
                    let translation = self.pose.translation.vector + na::Vector3::new(dx, dy, dz);

                    // Generate a grid of small rotations around the current rotation quaternion
                    for dq_x in (-1..=1).map(|i| i as f64 * GRID_QUAT) {
                        for dq_y in (-1..=1).map(|i| i as f64 * GRID_QUAT) {
                            for dq_z in (-1..=1).map(|i| i as f64 * GRID_QUAT) {
                                let rotation = self.pose.rotation
                                    * na::UnitQuaternion::from_euler_angles(dq_x, dq_y, dq_z);

                                // Create the new pose with the modified translation and rotation
                                let new_pose = Isometry3::from_parts(
                                    translation.into(),
                                    rotation,
                                );

                                // Create a new ComplexAlternative with the generated solutions and new pose
                                branches.push(ComplexAlternative {
                                    // Use kinematics to find solutions for the new pose
                                    // KinematicsWithShape will only return non-colliding and
                                    // constraint-compliant solutions
                                    solutions: kinematics.inverse(&new_pose),
                                    pose: new_pose,
                                });
                            }
                        }
                    }
                }
            }
        }

        branches
    }

    pub fn branch(&self, kinematics: &KinematicsWithShape) -> Vec<ComplexAlternative> {
        // Define the grid values for position and orientation without a zero offset
        let position_offsets = [-GRID_XYZ, GRID_XYZ];
        let rotation_offsets = [-GRID_QUAT, GRID_QUAT];

        // Generate a grid of positions and small rotations in parallel
        let branches: Vec<ComplexAlternative> = position_offsets
            .into_par_iter()  // Start the parallel iterator chain
            .flat_map(|dx| {
                position_offsets.into_par_iter().flat_map(move |dy| {
                    position_offsets.into_par_iter().flat_map(move |dz| {
                        // Calculate the translation vector with the specified offsets
                        let translation = self.pose.translation.vector + na::Vector3::new(dx, dy, dz);

                        // Parallelize over the rotation components
                        rotation_offsets.into_par_iter().flat_map(move |dq_x| {
                            rotation_offsets.into_par_iter().flat_map(move |dq_y| {
                                rotation_offsets.into_par_iter().map(move |dq_z| {
                                    // Calculate the rotation quaternion with the specified offsets
                                    let rotation = self.pose.rotation
                                        * na::UnitQuaternion::from_euler_angles(dq_x, dq_y, dq_z);

                                    // Create the new pose with the modified translation and rotation
                                    let new_pose = Isometry3::from_parts(translation.into(), rotation);

                                    // Generate solutions using kinematics.inverse. 
                                    // This method only returns non-coliding solutions.
                                    ComplexAlternative {
                                        
                                        solutions: kinematics.inverse(&new_pose),
                                        pose: new_pose,
                                    }
                                })
                            })
                        }).collect::<Vec<_>>()  // Collect inner loop results into a Vec for each (dx, dy, dz)
                    })
                })
            })
            .collect();  // Collect all parallel results into the final Vec

        branches
    }

    pub fn is_goal(current: &ComplexAlternative, goal: &ComplexAlternative) -> bool {
        // Define tolerance constants for position and orientation differences
        const GRID_XYZ: f64 = 0.001;  // tolerance for positional differences
        const GRID_QUAT: f64 = 0.01;  // tolerance for rotational differences

        // Check if translations fall within the positional tolerance (GRID_XYZ)
        let current_translation = current.pose.translation.vector;
        let goal_translation = goal.pose.translation.vector;

        let translation_diff = (current_translation - goal_translation).abs();
        if translation_diff.x > GRID_XYZ || translation_diff.y > GRID_XYZ || translation_diff.z > GRID_XYZ {
            return false;
        }

        // Check if rotations fall within the orientation tolerance (GRID_QUAT)
        // We use the angle difference between the two rotation quaternions here.
        // Quaternions represent rotation in a way that allows us to compute the angular
        // difference directly using `angle_to`, which gives the smallest angle between
        // the orientations represented by `current.pose.rotation` and `goal.pose.rotation`.
        // If this angle is within the tolerance `GRID_QUAT`, the two rotations are considered equivalent.
        let rotation_diff = current.pose.rotation.angle_to(&goal.pose.rotation);
        if rotation_diff > GRID_QUAT {
            return false;
        }

        true
    }
}



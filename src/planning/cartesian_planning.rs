extern crate nalgebra as na;

use std::f64::consts::PI;
use std::f64::INFINITY;
use na::{Isometry3};
use std::hash::{Hash, Hasher};
use nalgebra::UnitQuaternion;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;

use rayon::prelude::*;

const GRID_XYZ: f64 = 0.005;  // positional grid cell size 5 mm
const GRID_QUAT: f64 = 0.5 * PI / 180.0;  // rotational grid cell size 0.5 degree

#[derive(Debug, Clone)]
pub struct ComplexAlternative {
    pub solutions: Solutions,
    pub pose: Pose,
}

impl ComplexAlternative {
    /// Constructs a `ComplexAlternative` from joint angles, using forward kinematics to compute the pose.
    pub fn from_joints(joints: &Joints, kinematics: &KinematicsWithShape) -> Self {
        // Compute the pose using forward kinematics
        let pose = kinematics.forward(joints);

        // Initialize with the computed pose and solutions from inverse kinematics
        ComplexAlternative {
            pose,
            solutions: kinematics.inverse(&pose),
        }
    }

    /// Constructs a `ComplexAlternative` directly from a pose, using inverse kinematics to find solutions.
    pub fn from_pose(pose: &Pose, kinematics: &KinematicsWithShape) -> Self {
        // Initialize with the provided pose and solutions from inverse kinematics
        ComplexAlternative {
            pose: *pose,
            solutions: kinematics.inverse(pose),
        }
    }

    pub fn distance(&self, other: &ComplexAlternative) -> f64 {
        // Calculate the Euclidean distance between translation vectors
        let translation1 = self.pose.translation.vector;
        let translation2 = other.pose.translation.vector;

        let dx = translation1.x - translation2.x;
        let dy = translation1.y - translation2.y;
        let dz = translation1.z - translation2.z;

        let translation_distance = (dx * dx + dy * dy + dz * dz).sqrt();
        let rotation_distance = self.pose.rotation.angle_to(&other.pose.rotation);

        // Combine translation and rotation distances, scaling both
        let total_distance = translation_distance + rotation_distance;

        // Convert the result to usize
        total_distance
    }
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
        // Initialize the minimum cost to a large value as usize::MAX
        let mut min_cost = f64::INFINITY;

        // Iterate over each solution in self and other to find the minimum transition cost
        for joints1 in &self.solutions {
            for joints2 in &other.solutions {
                // Calculate the transition cost between joints1 and joints2
                let cost: f64 = joints1.iter()
                    .zip(joints2.iter())
                    .map(|(a, b)| (a - b).abs() )
                    .sum();

                // Update the minimum cost if a lower cost is found
                if cost < min_cost {
                    min_cost = cost;
                }
            }
        }
        println!("Cost {:?}", min_cost);
        min_cost
    }


    pub fn generate_neighbors(&self, kinematics: &KinematicsWithShape) -> Vec<(ComplexAlternative, f64)> {
        // Define the grid values for position and rotation offsets
        let position_offsets = [-GRID_XYZ, GRID_XYZ];
        let rotation_offsets = [-GRID_QUAT, GRID_QUAT];

        // Generate a grid of positions and small rotations in parallel
        let branches: Vec<(ComplexAlternative, f64)> = position_offsets
            .into_par_iter()
            .flat_map(|dx| {
                position_offsets.into_par_iter().flat_map(move |dy| {
                    position_offsets.into_par_iter().flat_map(move |dz| {
                        rotation_offsets.into_par_iter().flat_map(move |dq_x| {
                            rotation_offsets.into_par_iter().flat_map(move |dq_y| {
                                rotation_offsets.into_par_iter().filter_map(move |dq_z| {
                                    let xyz_offset = na::Vector3::new(dx, dy, dz);
                                    let rotation_offset = UnitQuaternion::from_euler_angles(dq_x, dq_y, dq_z);
                                    let new_pose = Isometry3::from_parts(
                                        (self.pose.translation.vector + xyz_offset).into(),
                                        self.pose.rotation * rotation_offset);
                                    let solutions = kinematics.inverse(&new_pose);
                                    if solutions.is_empty() {
                                        // We cannot go there - collision or out of constraints
                                        None
                                    } else {
                                        // Calculate the transition cost
                                        let alternative =
                                            ComplexAlternative {
                                                solutions: solutions,
                                                pose: new_pose,
                                            };
                                        //let cost = self.transition_costs(&alternative);
                                        let cost = self.distance(&alternative)/4.;
                                        Some((alternative, cost))
                                    }
                                })
                            })
                        })
                    })
                })
            })
            .collect();  // Collect all parallel results into the final Vec

        branches
    }
}


pub fn is_goal(current: &ComplexAlternative, goal: &ComplexAlternative) -> bool {
    const angle_threshold: f64 = 2.0 * GRID_QUAT;
    const pos_threshold: f64 = 2.0 * GRID_XYZ;
    const pth_squared: f64 = pos_threshold * pos_threshold;

    // Extract the translation vectors
    let current_translation = &current.pose.translation.vector;
    let goal_translation = &goal.pose.translation.vector;

    // Check if translation difference is within threshold
    let d = current_translation - goal_translation;
    if d.x * d.x + d.y * d.y + d.z * d.z >= pth_squared {
        return false;
    }

    // Calculate the angular difference between the rotations
    let rotation_diff = current.pose.rotation.angle_to(&goal.pose.rotation);

    // Check if the rotation difference is within the angle threshold
    if rotation_diff > angle_threshold {
        return false;
    }

    true
}




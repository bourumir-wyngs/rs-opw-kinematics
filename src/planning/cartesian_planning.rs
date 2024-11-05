extern crate nalgebra as na;

use std::f64::consts::PI;
use na::{Isometry3};
use std::sync::Arc;
use nalgebra::UnitQuaternion;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Solutions};
use crate::kinematics_with_shape::KinematicsWithShape;

use nalgebra::Vector3;
use rayon::prelude::*;

use rayon::prelude::*;
use crate::idastar::idastar_algorithm;
use crate::utils;

#[derive(Debug)]
pub struct CartesianPlanner {
    grid_xyz: f64,
    grid_quat: f64,
    angle_threshold: f64,
    cost_coefficients: Joints,
    max_transition_cost: f64,
}

impl Default for CartesianPlanner {
    fn default() -> Self {
        CartesianPlanner {
            grid_xyz: 0.005, // Positional grid cell size 5 mm 
            grid_quat: 0.5 * PI / 180.0, // Rotational grid cell size 0.5 degree
            angle_threshold: 1.0 * PI / 180.0,
            cost_coefficients: [1.5, 1.5, 1.5, 1.0, 1.0, 0.8],
            max_transition_cost: 6.0_f64.to_radians(),
        }
    }
}

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

impl CartesianPlanner {
    pub fn plan_path(
        &self,
        start: &ComplexAlternative,
        end: &ComplexAlternative,
        kinematics: &Arc<KinematicsWithShape>,
    ) -> Solutions {
        let path = idastar_algorithm(
            start,
            |current| self.generate_neighbors(current, kinematics),
            |current| self.heuristic(current, end),
            |current| self.is_goal(current, end),
        );
        
        if let Some(path) = path {
            self.build_best_path(&path.0)
        } else {
            Solutions::new()
        }
    }

    /// Constructs the optimal path through a sequence of `ComplexAlternative` instances
    /// to minimize cumulative transition costs between consecutive nodes.
    ///
    /// This function iterates through each `ComplexAlternative` in the input vector `alternatives`
    /// and computes the cumulative cost to reach each node, storing only the minimum costs. It then
    /// reconstructs the best path by tracing backward from the last node with the lowest cumulative cost.
    fn build_best_path(&self, alternatives: &Vec<ComplexAlternative>) -> Solutions {
        let n = alternatives.len();
        if n == 0 {
            return Vec::new();
        }

        let mut min_costs = vec![f64::INFINITY; n];
        let mut previous_index = vec![None; n];
        min_costs[0] = 0.0;

        for i in 1..n {
            for j in 0..i {
                let transition_cost =
                    self.alternative_transition_costs(&alternatives[j], &alternatives[i]);
                let cumulative_cost = min_costs[j] + transition_cost;

                if cumulative_cost < min_costs[i] {
                    min_costs[i] = cumulative_cost;
                    previous_index[i] = Some(j);
                }
            }
        }

        let mut best_path = vec![alternatives[0].solutions[0].clone()];
        let mut current_index = 1;

        while current_index < n {
            match previous_index[current_index] {
                Some(prev_index) => {
                    best_path.push(alternatives[current_index].solutions[0].clone());
                    current_index += 1;
                }
                None => {
                    panic!("No feasible path to complete the full sequence of alternatives");
                }
            }
        }

        assert!(best_path.len() == alternatives.len());
        best_path
    }

    pub fn remove_excessive_rotations(
        &self,
        from: &ComplexAlternative,
        solutions: Solutions,
    ) -> Solutions {
        solutions
            .into_iter()
            .filter(|to_solution| {
                from.solutions.iter()
                    .map(|from_solution| {
                        (0..6).map(|i| (from_solution[i] - to_solution[i]).abs() * self.cost_coefficients[i])
                            .sum::<f64>()
                    })
                    .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                    .unwrap_or(f64::INFINITY) <= self.max_transition_cost
            })
            .collect()
    }

    fn joint_transition_costs(&self, from: &Joints, to: &Joints) -> f64 {
        utils::transition_costs(&from, &to, &self.cost_coefficients)
    }

    fn alternative_transition_costs(&self, from: &ComplexAlternative, to: &ComplexAlternative) -> f64 {
        // Initialize the minimum cost to a large value as usize::MAX
        let mut min_cost = f64::INFINITY;

        // Iterate over each solution in self and other to find the minimum transition cost
        for joints1 in &from.solutions {
            for joints2 in &to.solutions {
                // Calculate the transition cost between joints1 and joints2
                let cost: f64 = utils::transition_costs(joints1, joints2, &self.cost_coefficients);

                // Update the minimum cost if a lower cost is found
                if cost < min_cost {
                    min_cost = cost;
                }
            }
        }
        min_cost
    }


    fn generate_neighbors(&self, current: &ComplexAlternative, kinematics: &KinematicsWithShape) -> Vec<(ComplexAlternative, f64)> {
        let position_offsets = [-self.grid_xyz, self.grid_xyz];
        let rotation_offsets = [-self.grid_quat, self.grid_quat];

        position_offsets.into_par_iter()
            .flat_map(|dx| {
                position_offsets.into_par_iter().flat_map(move |dy| {
                    position_offsets.into_par_iter().flat_map(move |dz| {
                        rotation_offsets.into_par_iter().flat_map(move |dq_x| {
                            rotation_offsets.into_par_iter().flat_map(move |dq_y| {
                                rotation_offsets.into_par_iter().filter_map(move |dq_z| {
                                    let xyz_offset = Vector3::new(dx, dy, dz);
                                    let rotation_offset = UnitQuaternion::from_euler_angles(dq_x, dq_y, dq_z);
                                    let new_pose = Isometry3::from_parts(
                                        (current.pose.translation.vector + xyz_offset).into(),
                                        current.pose.rotation * rotation_offset);

                                    let solutions = kinematics.remove_collisions(
                                        self.remove_excessive_rotations(current, kinematics.inverse(&new_pose)),
                                        false,
                                    );

                                    if solutions.is_empty() {
                                        None
                                    } else {
                                        let alternative = ComplexAlternative {
                                            solutions,
                                            pose: new_pose,
                                        };
                                        let cost = current.distance(&alternative) / 4.;
                                        Some((alternative, cost))
                                    }
                                })
                            })
                        })
                    })
                })
            })
            .collect()
    }

    fn is_goal(&self, current: &ComplexAlternative, goal: &ComplexAlternative) -> bool {
        let current_translation = &current.pose.translation.vector;
        let goal_translation = &goal.pose.translation.vector;
        let d = current_translation - goal_translation;

        if d.x * d.x + d.y * d.y + d.z * d.z >= self.grid_xyz * self.grid_xyz {
            return false;
        }

        let rotation_diff = current.pose.rotation.angle_to(&goal.pose.rotation);
        rotation_diff <= self.angle_threshold
    }

    fn heuristic(&self, current: &ComplexAlternative, goal: &ComplexAlternative) -> f64 {
        current.distance(goal)
    }
}

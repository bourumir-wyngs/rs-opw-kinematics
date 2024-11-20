use rrt::dual_rrt_connect;
use std::time::Instant;
use crate::kinematic_traits::{Joints, Kinematics};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::utils::dump_joints;

#[derive(Debug)]
pub struct RRTPlanner {
    step_size_joint_space: f64,
    max_try: usize,
    debug: bool
}

impl Default for RRTPlanner {
    fn default() -> Self {
        Self {
            step_size_joint_space: 3_f64.to_radians(),
            max_try: 2000,
            debug: true, 
        }
    }
}

impl RRTPlanner {
    /// Plans a path from `start` to `goal` joint configuration, 
    /// using `KinematicsWithShape` for collision checking.
    /// start and goal are included into the returned path.
    fn plan_path(
        &self,
        kinematics: &KinematicsWithShape,
        start: &Joints, goal: &Joints,
    ) -> Result<Vec<Vec<f64>>, String> {
        let collision_free = |joint_angles: &[f64]| -> bool {
            let joints = &<Joints>::try_from(joint_angles).expect("Cannot convert vector to array");
            !kinematics.collides(joints)
        };

        // Constraint compliant random joint configuration generator.
        let random_joint_angles = || -> Vec<f64> {
            // RRT requires vector and we return array so convert
            return kinematics.constraints()
                .expect("Set joint ranges on kinematics").random_angles().to_vec();
        };

        // Plan the path with RRT
        dual_rrt_connect(
            start, goal, collision_free,
            random_joint_angles, self.step_size_joint_space, // Step size in joint space
            self.max_try,  // Max iterations
        )
    }

    fn convert_result(&self, data: Result<Vec<Vec<f64>>, String>) -> Result<Vec<Joints>, String> {
        data.and_then(|vectors| {
            vectors
                .into_iter()
                .map(|vec| {
                    if vec.len() == 6 {
                        // Convert Vec<f64> to [f64; 6] if length is 6
                        Ok([vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]])
                    } else {
                        Err("One of the inner vectors does not have 6 elements.".to_string())
                    }
                })
                .collect()
        })
    }

    fn print_summary(&self, planning_result: &Result<Vec<[f64; 6]>, String>) {
        match planning_result {
            Ok(path) => {
                println!("Steps:");
                for step in path {
                    dump_joints(&step);
                }
            }
            Err(error_message) => {
                println!("Error: {}", error_message);
            }
        }
    }

    pub fn plan_rrt(&self, start: &Joints, goal: &Joints, kinematics: &KinematicsWithShape)
                    -> Result<Vec<Joints>, String> {
        let started = Instant::now();
        let path = self.plan_path(&kinematics, start, goal);
        let spent = started.elapsed();
        let result = self.convert_result(path);
        if self.debug {
            self.print_summary(&result);
            println!("RRT Took {:?}", &spent);
        }
        result
    }
}    


use crate::kinematic_traits::{Joints, Kinematics};
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::rrt_to::{dual_rrt_connect};
use crate::utils::dump_joints;
use std::sync::atomic::{AtomicBool};
use std::time::Instant;

#[derive(Debug)]
/// Defines the RRT planner that relocates the robot between the two positions in a
/// collision free way.
pub struct RRTPlanner {
    /// Step size in the joint space (value in Radians). This should be small
    /// enough to prevent robot colliding with something while moving
    /// in possibly less predictable way between the joints.
    pub step_size_joint_space: f64,

    /// The "max try" parameter of RRT algorithm, reasonable values
    /// are in order 1000 ... 4000
    pub max_try: usize,

    /// If direct relocation is not successful, the planner tries to bridge the path
    /// to waypoint, and then from there. Waypoints must be strategically defined.
    pub waypoints: Vec<Joints>,

    /// Flag to print extra diagnostics if required.
    pub debug: bool,
}

impl Default for RRTPlanner {
    fn default() -> Self {
        Self {
            step_size_joint_space: 3_f64.to_radians(),
            max_try: 2000,
            debug: true,
            waypoints: Vec::new(),
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
        start: &Joints,
        goal: &Joints,
        stop: &AtomicBool,
    ) -> Result<Vec<Vec<f64>>, String> {
        //return Ok(vec![Vec::from(start.clone()), Vec::from(goal.clone())]);
        
        let collision_free = |joint_angles: &[f64]| -> bool {
            let joints = &<Joints>::try_from(joint_angles).expect("Cannot convert vector to array");
            !kinematics.collides(joints)
        };

        // Constraint compliant random joint configuration generator.
        let random_joint_angles = || -> Vec<f64> {
            // RRT requires vector and we return array so convert
            return kinematics
                .constraints()
                .expect("Set joint ranges on kinematics")
                .random_angles()
                .to_vec();
        };

        // Plan the path with RRT
        let path = dual_rrt_connect(
            start,
            goal,
            collision_free,
            random_joint_angles,
            self.step_size_joint_space, // Step size in joint space
            self.max_try,               // Max iterations
            &stop,
        );

        path
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

    #[allow(dead_code)]
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

    /// Plans collision - free relocation from 'start' into 'goal', using
    /// provided instance of KinematicsWithShape for both inverse kinematics and
    /// collision avoidance.
    pub fn plan_rrt(
        &self,
        start: &Joints,
        goal: &Joints,
        kinematics: &KinematicsWithShape,
        stop: &AtomicBool,
    ) -> Result<Vec<Joints>, String> {
        println!("RRT started {:?} -> {:?}", start, goal);
        let started = Instant::now();
        let path = self.plan_path(&kinematics, start, goal, stop);
        let spent = started.elapsed();
        let result = self.convert_result(path);

        match &result {
            Ok(path) => {
                println!("RRT steps: {}", &path.len());
            }
            Err(error_message) => {
                println!("Direct RRT failed: {}", error_message);

                // Probe waypoints
                let mut alternative = Vec::new();
                for waypoint in &self.waypoints {
                    let retrying = Instant::now();
                    let to_waypoint = self.plan_path(&kinematics, start, waypoint, stop);
                    let to_wp = self.convert_result(to_waypoint);
                    if let Ok(starting) = &to_wp {
                        println!(
                            "Waypoint reached, RRT steps: {} in {:?}",
                            &starting.len(),
                            retrying.elapsed()
                        );
                        let from_waypoint = self.plan_path(&kinematics, waypoint, goal, stop);
                        if let Ok(closing) = &self.convert_result(from_waypoint) {
                            println!(
                                "bridged, RRT steps: {}, alternative took {:?}",
                                &closing.len(),
                                retrying.elapsed()
                            );
                            alternative.reserve(starting.len());
                            alternative.extend(starting);
                            alternative.extend(closing);
                            return Ok(alternative);
                        } else {
                            println!(
                                "RRT failed to bridge to waypoint after {:?}",
                                retrying.elapsed()
                            );
                            return Err("RRT failed to bridge to waypoint".to_string());
                        }
                    } else {
                        println!(
                            "RRT failed to onboard waypoint after {:?}",
                            retrying.elapsed()
                        );
                    }
                }
            }
        }
        // self.print_summary(&result);
        println!("RRT Took {:?} for {:?} -> {:?}", &spent, start, goal);

        result
    }
}

use crate::kinematic_traits::{Joints, Solutions};


/// Trait for Stroke Planners
/// Defines the common interface for planners that compute a sequence of joint configurations.
pub trait StrokePlanner {
    /// Plan the sequence of Joints given the poses and their possible configurations.
    ///
    /// # Parameters
    /// - `poses`: A vector of poses, where each pose is a vector of possible joint configurations.
    ///
    /// # Returns
    /// A sequence of selected Joints. If planning fails, the returned vector will be empty.
    fn plan(&self, poses: Vec<Solutions>) -> Vec<Joints>;
}

use std::collections::{BinaryHeap};
use std::cmp::Ordering;
use std::collections::HashMap;

/// HiGHS-based planner for computing optimal joint configurations
pub struct DijkstraPlanner {
    /// Maximum allowed transition cost between Joints
    pub max_transition_cost: f64,
    /// Debug mode for logging
    pub debug: bool,
}

impl DijkstraPlanner {
    /// Create a new HiGHSPlanner
    ///
    /// # Parameters
    /// - `max_transition_cost`: The maximum allowable cost for a valid transition between Joints.
    /// - `debug`: Enables verbose output when true.
    ///
    /// # Returns
    /// A new instance of `HiGHSPlanner`.
    pub fn new(max_transition_cost: f64, debug: bool) -> Self {
        DijkstraPlanner {
            max_transition_cost,
            debug,
        }
    }

    /// Compute the transition cost between two Joints
    fn transition_cost(
        &self,
        from: &Joints,
        to: &Joints,
        cost_limit: f64,
        force_calculation: bool,
    ) -> Option<f64> {
        // Skip calculation for high-cost transitions unless forced
        let mut partial_cost = 0.0;
        for (a, b) in from.iter().zip(to) {
            partial_cost += (a - b).abs();
            if self.debug && partial_cost < cost_limit {
                println!(
                    "Transition from {:?} to {:?}, cost: {}",
                    from, to, partial_cost
                );
            }

            if !force_calculation && partial_cost > cost_limit {
                println!(
                    "Needs jump planning: from {:?} to {:?}, approximal cost: {}",
                    from, to, partial_cost
                );
                
                return None; // Early exit for high-cost transitions
            }
        }
        if self.debug && force_calculation && partial_cost > cost_limit {
            println!(
                "Expensive transition from {:?} to {:?}, cost: {}",
                from, to, partial_cost
            );
        }        
        Some(partial_cost) // Full calculation
    }    
}

#[derive(Debug, PartialEq)]
struct Node {
    cost: f64,                 // Accumulated cost to reach this node
    pose_idx: usize,           // Pose index in the sequence
    solution_idx: usize,          // Joint configuration index within the pose
}

// Priority queue requires ordering; smallest cost should be at the top
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for Node {    
}


impl DijkstraPlanner {
    pub fn plan(&self, poses: Vec<Solutions>) -> Vec<Joints> {
        let num_poses = poses.len();
        if num_poses == 0 {
            return vec![];
        }

        let mut heap = BinaryHeap::new();
        let mut best_cost: HashMap<(usize, usize), f64> = HashMap::new();
        let mut previous: HashMap<(usize, usize), Option<(usize, usize)>> = HashMap::new();

        // Closure to run Dijkstra with a specific cost limit
        let mut run_dijkstra = |cost_limit: f64, force_calculation: bool| {
            heap.clear();
            best_cost.clear();
            previous.clear();

            // Initialize the queue with the first pose
            for solution_idx in 0..poses[0].len() {
                let node = (0, solution_idx);
                best_cost.insert(node, 0.0);
                previous.insert(node, None);
                heap.push(Node {
                    cost: 0.0,
                    pose_idx: 0,
                    solution_idx,
                });
            }

            // Dijkstra's algorithm
            while let Some(Node { cost, pose_idx, solution_idx }) = heap.pop() {
                if pose_idx == num_poses - 1 {
                    break;
                }

                for (next_joint_idx, next_joint) in poses[pose_idx + 1].iter().enumerate() {
                    let transition_cost = self.transition_cost(
                        &poses[pose_idx][solution_idx],
                        next_joint,
                        cost_limit,
                        force_calculation,
                    );

                    if let Some(transition_cost) = transition_cost {
                        let next_node = (pose_idx + 1, next_joint_idx);
                        let new_cost = cost + transition_cost;

                        if best_cost.get(&next_node).map_or(true, |&c| new_cost < c) {
                            best_cost.insert(next_node, new_cost);
                            previous.insert(next_node, Some((pose_idx, solution_idx)));
                            heap.push(Node {
                                cost: new_cost,
                                pose_idx: pose_idx + 1,
                                solution_idx: next_joint_idx,
                            });
                        }
                    }
                }
            }

            // Backtrack to construct the result
            let mut result = Vec::new();
            let mut current = best_cost
                .keys()
                .filter(|&&(pose, _)| pose == num_poses - 1)
                .min_by(|a, b| best_cost[a].partial_cmp(&best_cost[b]).unwrap())
                .cloned();

            while let Some(node) = current {
                let (pose_idx, joint_idx) = node;
                result.push(poses[pose_idx][joint_idx]);
                current = previous[&node];
            }

            result.reverse();
            result
        };

        // First pass: exclude high-cost transitions
        let result = run_dijkstra(self.max_transition_cost, false);

        // If no solution is found, allow expensive transitions
        if result.is_empty() {
            println!("Fallback to allow high-cost transitions");
            run_dijkstra(f64::INFINITY, true)
        } else {
            result
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dijkstra_planner_basic() {
        let poses = vec![
            vec![
                [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
                [1.0, 1.5, 2.5, 3.5, 4.5, 5.5],
            ],
            vec![
                [2.0, 2.5, 3.0, 4.0, 5.0, 6.0],
                [1.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            ],
        ];

        let planner = DijkstraPlanner::new(100.0, true);
        let result = planner.plan(poses);

        assert!(!result.is_empty(), "Result should not be empty");
        assert_eq!(result.len(), 2, "Result should have one joint per pose");
    }    

    #[test]
    fn test_minimal() {
        // Define a simple test case with one pose and two configurations
        let poses = vec![
            vec![
                [0.0, 1.0, 2.0, 3.0, 4.0, 5.0], // Pose 0, Config 0
                [1.0, 1.5, 2.5, 3.5, 4.5, 5.5], // Pose 0, Config 1
            ],
        ];

        let planner = DijkstraPlanner::new(1000.0, true);
        let result = planner.plan(poses);

        assert!(!result.is_empty(), "Result should not be empty");
        assert_eq!(result.len(), 1, "Result should have one pose");
    }    
    
    #[test]
    fn test_basic() {
        // Define some test poses and their possible joint configurations
        let poses = vec![
            vec![
                [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],  // Pose 0, Configuration 0
                [1.0, 1.5, 2.5, 3.5, 4.5, 5.5],  // Pose 0, Configuration 1
            ],
            vec![
                [2.0, 2.5, 3.0, 4.0, 5.0, 6.0],  // Pose 1, Configuration 0
                [1.0, 1.0, 2.0, 3.0, 4.0, 5.0],  // Pose 1, Configuration 1
            ],
        ];

        // Initialize the HiGHS planner with a reasonable max transition cost
        let planner = DijkstraPlanner::new(1000.0, true);

        // Call the planner's plan method
        let result = planner.plan(poses);

        // Verify that the result is not empty
        assert!(!result.is_empty(), "Result should not be empty");

        // Verify that the result has the same number of poses as input
        assert_eq!(result.len(), 2, "Result should have the same number of poses as input");

        // Verify that each joint configuration in the result matches one from the input
        let expected_pose_0 = vec![
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
            [1.0, 1.5, 2.5, 3.5, 4.5, 5.5],
        ];
        let expected_pose_1 = vec![
            [2.0, 2.5, 3.0, 4.0, 5.0, 6.0],
            [1.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        ];

        assert!(expected_pose_0.contains(&result[0]), "Result pose 0 is invalid");
        assert!(expected_pose_1.contains(&result[1]), "Result pose 1 is invalid");
    }

    #[test]
    fn test_dijkstra_planner_longer() {
        // Define a longer test case with multiple poses and joint configurations
        let poses = vec![
            vec![
                [0.0, 1.0, 2.0, 3.0, 4.0, 5.0], // Pose 0, Config 0
                [1.0, 1.5, 2.5, 3.5, 4.5, 5.5], // Pose 0, Config 1
            ],
            vec![
                [2.0, 2.5, 3.0, 4.0, 5.0, 6.0], // Pose 1, Config 0
                [1.0, 1.0, 2.0, 3.0, 4.0, 5.0], // Pose 1, Config 1
                [3.0, 3.5, 4.0, 5.0, 6.0, 7.0], // Pose 1, Config 2
            ],
            vec![
                [4.0, 5.0, 6.0, 7.0, 8.0, 9.0], // Pose 2, Config 0
                [2.0, 2.5, 3.5, 4.5, 5.5, 6.5], // Pose 2, Config 1
            ],
            vec![
                [5.0, 6.0, 7.0, 8.0, 9.0, 10.0], // Pose 3, Config 0
            ],
        ];

        let planner = DijkstraPlanner::new(10.5, true); // Set a large transition cost threshold
        let result = planner.plan(poses.clone());

        // Ensure the result is not empty
        assert!(!result.is_empty(), "Result should not be empty");

        // Ensure the result length matches the number of poses
        assert_eq!(result.len(), poses.len(), "Result should have one joint per pose");

        // Check that the joints in the result exist in the corresponding poses
        for (pose_idx, joint) in result.iter().enumerate() {
            assert!(
                poses[pose_idx].contains(joint),
                "Result joint {:?} is not valid for pose {}",
                joint,
                pose_idx
            );
        }

        // Print the resulting path for debugging
        println!("Planned path of joints:");
        for (i, joint) in result.iter().enumerate() {
            println!("Pose {}: {:?}", i, joint);
        }
    }
    
}



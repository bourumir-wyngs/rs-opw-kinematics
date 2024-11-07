use kdtree::KdTree;
use nalgebra::{distance, Isometry3, Translation3, UnitQuaternion};
use rand::Rng;
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, JOINTS_AT_ZERO};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::collisions::CollisionBody;
use rs_opw_kinematics::utils;
use rs_opw_kinematics::utils::{dump_joints, transition_costs};

pub fn create_rx160_robot() -> KinematicsWithShape {
    use rs_opw_kinematics::read_trimesh::load_trimesh_from_stl;

    let monolith = load_trimesh_from_stl("src/tests/data/object.stl");

    KinematicsWithShape::new(
        Parameters {
            a1: 0.15,
            a2: 0.0,
            b: 0.0,
            c1: 0.55,
            c2: 0.825,
            c3: 0.625,
            c4: 0.11,
            ..Parameters::new()
        },
        Constraints::from_degrees(
            [
                -225.0..=225.0, -225.0..=225.0, -225.0..=225.0,
                -225.0..=225.0, -225.0..=225.0, -360.0..=360.0,
            ],
            BY_PREV,
        ),
        [
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_1.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_2.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_3.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_4.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_5.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_6.stl"),
        ],
        load_trimesh_from_stl("src/tests/data/staubli/rx160/base_link.stl"),
        Isometry3::from_parts(
            Translation3::new(0.4, 0.7, 0.0).into(),
            UnitQuaternion::identity(),
        ),
        load_trimesh_from_stl("src/tests/data/flag.stl"),
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.5).into(),
            UnitQuaternion::identity(),
        ),
        vec![
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(-1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., 1., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., -1., 0.) },
        ],
        true,
    )
}

use std::collections::HashMap;
use num_traits::float::FloatCore;

pub struct RRTConnectPlanner<'a> {
    kinematics: &'a KinematicsWithShape,
    max_iterations: usize,
    step_size: f64,
    connection_threshold: f64,
}

impl<'a> RRTConnectPlanner<'a> {
    pub fn new(kinematics: &'a KinematicsWithShape, max_iterations: usize, step_size: f64) -> Self {
        Self {
            kinematics,
            max_iterations,
            step_size,
            connection_threshold: 50., // This value is in degrees
        }
    }

    pub fn plan_path(&self, start: Joints, goal: Joints) -> Option<Vec<Joints>> {
        let mut tree_a = Tree::new(start); // Grows from start to goal
        let mut tree_b = Tree::new(goal);  // Grows from goal to start

        for iteration in 0..self.max_iterations {
            println!("Iteration: {}", iteration);

            // Calculate the closest distance between the two trees
            let closest_distance_deg = self.closest_distance_between_trees_deg(&tree_a, &tree_b);

            // Determine whether to use goal biasing based on the closest distance
            let use_goal_biasing = closest_distance_deg < 180.0;

            // Generate a random configuration, with goal biasing if trees are close
            let remote_random_a = self.random_configuration_with_goal_bias(&goal, &tree_b, use_goal_biasing);
            let remote_random_b = self.random_configuration_with_goal_bias(&start, &tree_a, use_goal_biasing);

            // Attempt to extend both trees towards the random configuration
            self.extend_towards(&mut tree_a, remote_random_b);
            self.extend_towards(&mut tree_b, remote_random_a);

            // Attempt to connect the trees if they are close enough
            if closest_distance_deg < self.connection_threshold {
                if self.connect(&mut tree_a, &mut tree_b) {
                    return Some(self.build_path(&tree_a, &tree_b));
                }
            }
        }

        println!("No collision-free path found within the iteration limit.");
        None
    }

    fn connect(&self, tree_a: &mut Tree, tree_b: &mut Tree) -> bool {
        println!("Connect?");

        // Find the closest node in tree_a to the nearest node in tree_b
        let nearest_index_a = tree_a.nearest_neighbor_index(&tree_b.nodes[0]); // Assuming root is goal
        let nearest_node_a = &tree_a.nodes[nearest_index_a];

        // Find the nearest node in tree_b to this node in tree_a
        let nearest_index_b = tree_b.nearest_neighbor_index(nearest_node_a);
        let nearest_node_b = &tree_b.nodes[nearest_index_b];

        assert!(!self.kinematics.collides(nearest_node_a));
        assert!(!self.kinematics.collides(nearest_node_b));

        // Attempt to step from the nearest node in tree_a toward the nearest node in tree_b
        let midpoint = self.try_joining_trees(nearest_node_a, nearest_node_b);
        if midpoint.is_none() {
            return false;
        }
        let connecting_node = midpoint.unwrap();

        // Check for collisions with the new node
        if self.kinematics.collides(&connecting_node) {
            println!("Midpoint collides:");
            dump_joints(&connecting_node);
            return false; // If there’s a collision, we cannot connect
        }

        // Add the new node to tree_a
        tree_a.add_node(connecting_node.clone(), Some(nearest_index_a));

        // Check if the new node reaches the goal (the start node of tree_b)
        if self.is_goal_reached(&connecting_node, nearest_node_b) {
            println!("Goal is reached with midpoint:");
            dump_joints(&connecting_node);
            return true; // Successfully connected to the goal
        } else {
            println!("Goal is not yet reached");
        }
        false
    }


    // Modified random configuration function with goal biasing
    fn random_configuration_with_goal_bias(
        &self,
        goal: &Joints,
        other_tree: &Tree,
        use_goal_biasing: bool,
    ) -> Joints {
        let mut rng = rand::thread_rng();
        if use_goal_biasing && rng.gen_bool(0.5) {
            if rng.gen_bool(0.1) {
                *goal  // Sample directly at the goal
            } else {
                // Sample near the closest node in the other tree
                let closest_node = other_tree.nodes[other_tree.nearest_neighbor_index(goal)];
                self.sample_near_node(&closest_node)
            }
        } else {
            // Normal random sampling
            self.kinematics.constraints()
                .expect("Set joint ranges on kinematics").random_angles()
        }
    }

    // Helper function to sample near a specific node (e.g., the closest node in the other tree)
    fn sample_near_node(&self, node: &Joints) -> Joints {
        let mut rng = rand::thread_rng();
        let mut perturbed_node = *node;

        // Apply random perturbation directly with indexing
        for i in 0..6 {
            let perturbation = rng.gen_range(-self.step_size..self.step_size);
            perturbed_node[i] += perturbation;
        }
        perturbed_node
    }

    fn closest_distance_between_trees_deg(&self, tree_a: &Tree, tree_b: &Tree) -> f64 {
        const cost_coefficients: Joints = [1.0, 1.0, 1.0, 1.0, 1.0, 0.8];
        let mut min_distance = f64::INFINITY; // Initialize minimum distance to infinity

        let mut a: Option<&Joints> = None;
        let mut b: Option<&Joints> = None;

        // Iterate through each node in tree_a
        for node_a in &tree_a.nodes {
            // Iterate through each node in tree_b
            for node_b in &tree_b.nodes {
                // Calculate the squared Euclidean distance between the nodes
                let distance = transition_costs(&node_a, &node_b, &cost_coefficients);
                // Update min_distance if a smaller distance is found
                if distance < min_distance {
                    min_distance = distance;
                    a = Some(node_a);
                    b = Some(node_b);
                }
            }
        }

        let distance = min_distance.to_degrees();
        println!("Closest distance: {:.4}", distance);
        if let (Some(a), Some(b)) = (a, b) {
            dump_joints(&a);
            dump_joints(&b);
            println!("**");
        };
        distance
    }

    // Extend towards some target.
    fn extend_towards(&self, tree: &mut Tree, direction_pointer: Joints) -> bool {
        let nearest_index = tree.nearest_neighbor_index(&direction_pointer);
        let suitable_start = tree.nodes[nearest_index];
        let new_node = self.step_towards(&suitable_start, &direction_pointer);
        if self.kinematics.collides(&new_node) {
            println!("Not extended - kinematics collides");
            return false;
        } else {
            println!("Extended:");
            dump_joints(&new_node);
        }
        tree.add_node(new_node, Some(nearest_index));
        true
    }

    fn build_path(&self, tree_a: &Tree, tree_b: &Tree) -> Vec<Joints> {
        let mut path = tree_a.build_path_to_root(tree_a.nodes.len() - 1);
        path.reverse();
        path.extend(tree_b.build_path_to_root(tree_b.nodes.len() - 1));
        path
    }

    fn step_towards(&self, start: &Joints, goal: &Joints) -> Joints {
        let mut distance_vector: Joints = [0.0; 6];
        let mut new_joint = *start;

        // Step 1: Compute the distance vector between start and goal
        for i in 0..6 {
            distance_vector[i] = goal[i] - start[i];
        }

        // Step 2: Calculate the Euclidean distance between start and goal
        let total_distance: f64 = distance_vector
            .iter()
            .map(|&d| d * d)
            .sum::<f64>()
            .sqrt();

        // Step 3: Determine the scaling factor
        let scaling_factor = if total_distance > self.step_size {
            self.step_size / total_distance
        } else {
            1.0 // If the distance is within step_size, move directly to the goal
        };

        // Step 4: Apply the scaling factor to the distance vector and update the new_joint position
        for i in 0..6 {
            new_joint[i] = start[i] + distance_vector[i] * scaling_factor;
        }

        new_joint
    }


    fn try_joining_trees(&self, a: &Joints, b: &Joints) -> Option<Joints> {
        let mut mid: Joints = JOINTS_AT_ZERO.clone();
        for i in 0..6 {
            mid[i] = (a[i] + b[i]) / 2.0;
        }

        const cost_coefficients: Joints = [1.5, 1.5, 1.5, 1.0, 1.0, 0.8];
        let ra = transition_costs(&mid, &a, &cost_coefficients);
        let rb = transition_costs(&mid, &b, &cost_coefficients);

        if ra + rb <= 10.0 * self.step_size {
            println!("Distances {} {}, step {}, midpoint valid", ra, rb, self.step_size);
            return Some(mid);
        } else {
            println!("Distances {} {}, step {}, midpoint NOT valid", ra, rb, self.step_size);
            dump_joints(&a);
            dump_joints(&mid);
            dump_joints(b);
        }
        None
    }

    fn is_goal_reached(&self, current: &Joints, goal: &Joints) -> bool {
        let mut all_within_tolerance = true;

        println!("Goal");
        dump_joints(goal);
        dump_joints(current);

        for (c, g) in current.iter().zip(goal) {
            let difference = (c - g).abs();
            if difference >= 20.0_f64.to_radians() {
                all_within_tolerance = false;
            }
        }

        all_within_tolerance
    }
}

struct Tree {
    nodes: Vec<Joints>,
    parents: HashMap<usize, Option<usize>>,
    kd_tree: KdTree<f64, usize, Joints>,
}

impl Tree {
    fn new(root: Joints) -> Self {
        let mut kd_tree = KdTree::new(6);
        kd_tree.add(root, 0).expect("Failed to add root node");
        let nodes = vec![root];
        let mut parents = HashMap::new();
        parents.insert(0, None);

        Self { nodes, parents, kd_tree }
    }

    fn add_node(&mut self, node: Joints, parent_index: Option<usize>) -> usize {
        let index = self.nodes.len();
        self.nodes.push(node);
        self.parents.insert(index, parent_index);
        self.kd_tree.add(self.nodes[index], index).expect("Failed to add node to KD-tree");
        index
    }

    fn nearest_neighbor_index(&self, target: &Joints) -> usize {
        self.kd_tree.nearest(target, 1, &squared_euclidean)
            .map(|nearest| *nearest[0].1)
            .unwrap_or(0)
    }

    fn build_path_to_root(&self, mut index: usize) -> Vec<Joints> {
        let mut path = vec![self.nodes[index].clone()];
        while let Some(&parent) = self.parents.get(&index) {
            if let Some(parent_index) = parent {
                path.push(self.nodes[parent_index].clone());
                index = parent_index;
            } else {
                break;
            }
        }
        path
    }
}

fn squared_euclidean(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| (x - y).powi(2)).sum()
}


fn main() {
    let initial_angles = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    let final_angles = utils::joints(&[40.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    let kinematics = create_rx160_robot();

    let constraints = kinematics.constraints().unwrap();
    assert!(constraints.compliant(&initial_angles));
    assert!(constraints.compliant(&final_angles));

    assert!(!kinematics.collides(&initial_angles));
    assert!(!kinematics.collides(&final_angles));

    let max_iterations = 10000;
    let step_size = 25_f64.to_radians();
    let planner = RRTConnectPlanner::new(&kinematics, max_iterations, step_size);

    match planner.plan_path(initial_angles, final_angles) {
        Some(path) => {
            println!();
            println!("Start:");
            dump_joints(&initial_angles);

            println!("Path found:");
            for joint_config in path {
                utils::dump_joints(&joint_config);
            }
            println!("Goal:");
            dump_joints(&final_angles);
        }
        None => {
            println!("No collision-free path found within the iteration limit.");
            println!("Start:");
            dump_joints(&initial_angles);

            println!("Goal:");
            dump_joints(&final_angles);
        }
    }
}

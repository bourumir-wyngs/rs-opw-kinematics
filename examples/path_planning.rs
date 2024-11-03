use pathfinding::prelude::{astar, fringe, idastar};
use std::sync::Arc;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_opw_kinematics::collisions::CollisionBody;
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use std::hash::{Hash, Hasher};
use std::ops::{Index, IndexMut};
use std::time::Instant;
use rs_opw_kinematics::utils;

#[derive(Debug, Clone, Copy)]
pub struct JointArray(pub [f64; 6]);

impl JointArray {
    /// Returns an iterator over the joint values.
    pub fn iter(&self) -> std::slice::Iter<f64> {
        self.0.iter()
    }

    /// Returns a mutable iterator over the joint values.
    pub fn iter_mut(&mut self) -> std::slice::IterMut<f64> {
        self.0.iter_mut()
    }

    /// Safe access to joint values by index with bounds checking.
    pub fn get(&self, index: usize) -> Option<f64> {
        self.0.get(index).copied()
    }
}

// Implement PartialEq and Eq for comparison
impl PartialEq for JointArray {
    fn eq(&self, other: &Self) -> bool {
        const EPSILON: f64 = 0.001;
        self.iter()
            .zip(other.iter())
            .all(|(a, b)| (a - b).abs() < EPSILON)
    }
}

impl Eq for JointArray {}

// Implement Hash for JointArray
impl Hash for JointArray {
    fn hash<H: Hasher>(&self, state: &mut H) {
        const SCALE: f64 = 0.001;
        for &value in &self.0 {
            let scaled = (value * SCALE) as i64;
            scaled.hash(state);
        }
    }
}

// Implement Index and IndexMut for bracket indexing
impl Index<usize> for JointArray {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl IndexMut<usize> for JointArray {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

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

fn main() {
    let kinematics = Arc::new(create_rx160_robot());

    let initial_angles = utils::joints(&[100., -7.44, -92.51, 18.42, 82.23, 189.35]);
    let final_angles = utils::joints(&[105., -7.44, -82.51, 18.42, 70.23, 188.35]);
    let start = JointArray(initial_angles);
    let end = JointArray(final_angles);

    // Record the start time
    let start_time = Instant::now();

    // Run path planning
    if let Some((path, cost)) = plan_path(&start, &end, &kinematics) {
        // Calculate the duration taken for path planning
        let duration = start_time.elapsed();
        for (i, joints) in path.iter().enumerate() {
            println!("Step {}: {:?}", i, utils::to_degrees(&joints.0));
        }
        println!("Path found with cost {}: (Time taken: {:?})", cost, duration);
    } else {
        let duration = start_time.elapsed();
        println!("No collision-free path found. (Time taken: {:?})", duration);
    }
}

fn plan_path(
    start: &JointArray,
    end: &JointArray,
    kinematics: &Arc<KinematicsWithShape>,
) -> Option<(Vec<JointArray>, usize)> {
    println!("Starting path planning...");
    idastar(
        start,
        |current| generate_neighbors(current, kinematics),
        |current| heuristic(current, end),
        |current| is_goal(current, end),
    )
}

fn is_goal(current: &JointArray, goal: &JointArray) -> bool {
    let threshold: f64 = 0.2_f64.to_radians();
    let is = current
        .iter()
        .zip(goal.iter())
        .all(|(c, g)| (c - g).abs() < threshold);
    is
}

fn heuristic(current: &JointArray, goal: &JointArray) -> usize {
    const SCALE: f64 = 10000.0; // Scale to retain precision after rounding
    current
        .iter()
        .zip(goal.iter())
        .map(|(c, g)| ((c - g).abs().to_degrees() * SCALE).round() as usize)
        .sum()
}


use rayon::prelude::*;

fn generate_neighbors(
    joints: &JointArray,
    kinematics: &Arc<KinematicsWithShape>,
) -> Vec<(JointArray, usize)> {
    let step_size = 0.1_f64.to_radians();

    // Generate all (index, delta) combinations
    let neighbor_combinations: Vec<_> = (0..6)
        .flat_map(|i| vec![(i, -step_size), (i, step_size)])
        .collect();

    // Process each combination in parallel, filtering out colliding neighbors
    neighbor_combinations
        .par_iter()
        .filter_map(|&(i, delta)| {
            let mut new_joints = *joints;
            new_joints.0[i] += delta;

            // Check for collision and return neighbor if it's valid
            // This is the most expensive step due what parallelization works.
            if !kinematics.collides(&new_joints.0) {
                Some((new_joints, 1)) // Cost of 1 per move
            } else {
                None
            }
        })
        .collect()
}


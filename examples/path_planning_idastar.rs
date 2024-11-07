#[cfg(feature = "collisions")]
use {
    // All would run
    pathfinding::prelude::{astar, fringe, idastar},
    std::collections::HashSet,
    std::hash::{Hash, Hasher},
    std::sync::Arc,
    std::time::Instant,
    nalgebra::{Isometry3, Translation3, UnitQuaternion},
    rs_opw_kinematics::collisions::CollisionBody,
    rs_opw_kinematics::constraints::{Constraints, BY_PREV},
    rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape,
    rs_opw_kinematics::parameters::opw_kinematics::Parameters,
    rs_opw_kinematics::utils,
};

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
}

// Implement PartialEq and Eq for comparison
impl PartialEq for JointArray {
    fn eq(&self, other: &Self) -> bool {
        let epsilon: f64 = 0.01_f64.to_radians();
        for (a, b) in self.iter().zip(other.iter()) {
            if (a - b).abs() > epsilon {
                return false;
            }
        }
        true
    }
}

impl Hash for JointArray {
    fn hash<H: Hasher>(&self, state: &mut H) {
        for value in self.0 {
            let rounded = value as i16;
            rounded.hash(state);
        }
    }
}

impl Eq for JointArray {}

#[cfg(feature = "collisions")]
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

#[cfg(feature = "collisions")]
fn plan_path(
    start: &JointArray,
    end: &JointArray,
    kinematics: &Arc<KinematicsWithShape>,
) -> Option<(Vec<JointArray>, usize)> {
    // While it works also without tracking with this hash set, it increases
    // the speed of path planning about three times.
    let mut explored: HashSet<JointArray> = HashSet::with_capacity(1000);
    // You can try astar, idastar or fringe here.
    // All 3 generally work but idastar is the fastest.
    idastar(
        start,
        |current| generate_neighbors(current, kinematics, &mut explored),
        |current| heuristic(current, end),
        |current| is_goal(current, end),
    )
}

#[cfg(feature = "collisions")]
fn is_goal(current: &JointArray, goal: &JointArray) -> bool {
    let threshold: f64 = 2.5_f64.to_radians();
    let is = current
        .iter()
        .zip(goal.iter())
        .all(|(c, g)| (c - g).abs() < threshold);
    is
}

#[cfg(feature = "collisions")]
fn heuristic(current: &JointArray, goal: &JointArray) -> usize {
    let estimation: f64 = current
        .iter()
        .zip(goal.iter())
        .map(|(c, g)| (c - g).abs())
        .sum();
    (estimation.to_degrees() * 100.) as usize
}

#[cfg(feature = "collisions")]
fn generate_neighbors(
    current: &JointArray,
    kinematics: &Arc<KinematicsWithShape>,
    explored: &mut HashSet<JointArray>,
) -> Vec<(JointArray, usize)> {
    let step_size = 2.5_f64.to_radians();

    // Map valid neighbors to the expected output format (JointArray, cost)
    let mut neighbors = Vec::with_capacity(2 * 6);

    // Prepare `from` and `to` joint configurations by adding/subtracting the step size
    let mut from = *current;
    let mut to = *current;
    for i in 0..6 {
        from.0[i] -= step_size;
        to.0[i] += step_size;
    }

    // Use non_colliding_offsets to generate valid, non-colliding neighbors
    let valid_offset_neighbors = kinematics.non_colliding_offsets(&current.0, &from.0, &to.0);
    for joints in valid_offset_neighbors {
        let new_joints = JointArray(joints);
        if !explored.contains(&new_joints) {
            neighbors.push((new_joints, 1)); // fixed transition cost.
            explored.insert(new_joints);
        }
    }
    neighbors
}

#[cfg(feature = "collisions")]
fn main() {
    let kinematics = Arc::new(create_rx160_robot());

    // This is less though as we give for rrt, but the robot still must raise the head up
    // to pass through the obstacle, and then lower it.
    let initial_angles = utils::joints(&[-120., -40., -92.51, 18.42, 82.23, 189.35]);
    let final_angles = utils::joints(&[40., -40., -92.51, 18.42, 82.23, 189.35]);
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

#[cfg(not(feature = "collisions"))]
fn main() {
    println!("Build configuration does not support this example")
}
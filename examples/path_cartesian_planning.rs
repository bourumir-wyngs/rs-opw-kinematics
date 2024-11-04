use std::f64::consts::PI;
use pathfinding::prelude::{astar, fringe, idastar};
use std::sync::Arc;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_opw_kinematics::collisions::CollisionBody;
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;

use std::time::Instant;
use rs_opw_kinematics::cartesian_planning::{ComplexAlternative};
use rs_opw_kinematics::{cartesian_planning, utils};

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

pub fn print_pose(isometry: &Isometry3<f64>) {
    // Extract translation components
    let translation = isometry.translation.vector;

    // Extract rotation components and convert to Euler angles in radians
    let rotation: UnitQuaternion<f64> = isometry.rotation;

    // Print translation and rotation
    println!(
        "x: {:.5}, y: {:.5}, z: {:.5},  quat: {:.5},{:.5},{:.5},{:.5}",
        translation.x, translation.y, translation.z, rotation.i, rotation.j, rotation.k, rotation.w
    );
}

fn main() {
    let kinematics = Arc::new(create_rx160_robot());

    let initial_angles = utils::joints(&[100., -7.44, -92.51, 18.42, 82.23, 189.35]);
    let final_angles = utils::joints(&[105., -7.44, -82.51, 18.42, 60.23, 188.35]);
    let start = ComplexAlternative::from_joints(&initial_angles, &kinematics);
    let end = ComplexAlternative::from_joints(&final_angles, &kinematics);

    // Record the start time
    let start_time = Instant::now();

    // Run path planning
    if let Some((path, cost)) = plan_path(&start, &end, &kinematics) {
        // Calculate the duration taken for path planning
        let duration = start_time.elapsed();
        for (i, step) in path.iter().enumerate() {
            print_pose(&step.pose);
        }
        println!("Goal:");            
        print_pose(&end.pose);
        println!("Path found with cost {}: (Time taken: {:?})", cost, duration);        
    } else {
        let duration = start_time.elapsed();
        println!("No collision-free path found. (Time taken: {:?})", duration);
    }
}

fn plan_path(
    start: &ComplexAlternative,
    end: &ComplexAlternative,
    kinematics: &Arc<KinematicsWithShape>,
) -> Option<(Vec<ComplexAlternative>, usize)> {
    println!("Starting path planning...");
    idastar(
        start,
        |current| generate_neighbors(current, kinematics),
        |current| heuristic(current, end),
        |current| cartesian_planning::is_goal(current, end),
    )
}

fn heuristic(current: &ComplexAlternative, goal: &ComplexAlternative) -> usize {
    //print_pose(&current.pose);
    //print_pose(&goal.pose);
    //println!("***");
    current.distance(goal)
}

fn generate_neighbors(
    current: &ComplexAlternative,
    kinematics: &Arc<KinematicsWithShape>,
) -> Vec<(ComplexAlternative, usize)> {
    current.generate_neighbors(&kinematics)
}


use std::sync::Arc;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_opw_kinematics::collisions::CollisionBody;
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;

use std::time::Instant;
use rs_opw_kinematics::cartesian_planning::{CartesianPlanner, ComplexAlternative};
use rs_opw_kinematics::kinematic_traits::Kinematics;
use rs_opw_kinematics::utils;
use rs_opw_kinematics::utils::{dump_pose, dump_solutions, dump_solutions_degrees};

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
    let final_angles = utils::joints(&[105., -7.44, -82.51, 18.42, 60.23, 188.35]);
    let start = ComplexAlternative::from_joints(&initial_angles, &kinematics);
    let end = ComplexAlternative::from_joints(&final_angles, &kinematics);

    // Record the start time
    let start_time = Instant::now();

    let planner = CartesianPlanner::default();

    // Run path planning
    println!("Starting path planning...");    
    let path = planner.plan_path(&start, &end, &kinematics);
    let took = start_time.elapsed();
    dump_solutions(&path);
    
    println!("Start pose:");
    dump_pose(&start.pose);
    println!("End pose:");
    dump_pose(&end.pose);
    println!("Planned pose:");
    let last = path.last().unwrap();
    dump_pose(&kinematics.forward(last));
    println!("Took {:?}. Joint steps:", took);    
}



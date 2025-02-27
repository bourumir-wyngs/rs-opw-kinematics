#[cfg(feature = "stroke_planning")]
use {
    rrt::dual_rrt_connect,
    std::time::Instant,
    nalgebra::{Isometry3, Translation3, UnitQuaternion},
    rs_opw_kinematics::kinematic_traits::{Joints, Kinematics},
    rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape,
    rs_opw_kinematics::parameters::opw_kinematics::Parameters,
    rs_opw_kinematics::constraints::{Constraints, BY_PREV},
    rs_opw_kinematics::collisions::CollisionBody,
    rs_opw_kinematics::utils,
    rs_opw_kinematics::utils::dump_joints,
};

#[cfg(feature = "stroke_planning")]
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
        // Constraints are used also to sample constraint-compliant random positions
        // as needed by this path planner.
        Constraints::from_degrees(
            [
                -225.0..=225.0, -225.0..=225.0, -225.0..=225.0,
                -225.0..=225.0, -225.0..=225.0, -225.0..=225.0,
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


/// Plans a path from `start` to `goal` joint configuration, using `KinematicsWithShape` for collision checking.
#[cfg(feature = "stroke_planning")]
fn plan_path(
    kinematics: &KinematicsWithShape,
    start: Joints, goal: Joints,
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
        &start, &goal, collision_free,
        random_joint_angles, 3_f64.to_radians(), // Step size in joint space
        2000,  // Max iterations
    )
}

#[cfg(feature = "stroke_planning")]
fn convert_result(data: Result<Vec<Vec<f64>>, String>) -> Result<Vec<Joints>, String> {
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

#[cfg(feature = "stroke_planning")]
fn print_summary(planning_result: &Result<Vec<[f64; 6]>, String>) {
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

#[cfg(feature = "stroke_planning")]
fn main() {
    // Initialize kinematics with your robot's specific parameters
    let kinematics = create_rx160_robot();

    // This is pretty tough path that requires to lift the initially low placed
    // tool over the obstacle and then lower again. Direct path is interrupted
    // by obstacle.
    println!("** Tough example **");
    let start = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    let goal = utils::joints(&[40.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    example(start, goal, &kinematics);

    // Simple short step
    println!("** Simple example **");
    let start = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    let goal = utils::joints(&[-120.0, -80.0, -90., 18.42, 82.23, 189.35]);
    example(start, goal, &kinematics);
}

#[cfg(feature = "stroke_planning")]
fn example(start: Joints, goal: Joints, kinematics: &KinematicsWithShape) {
    let started = Instant::now();
    let path = plan_path(&kinematics, start, goal);
    let spent = started.elapsed();
    let result = convert_result(path);
    print_summary(&result);
    println!("Took {:?}", &spent);
}

#[cfg(not(feature = "stroke_planning"))]
fn main() {
    println!("Build configuration does not support this example")
}
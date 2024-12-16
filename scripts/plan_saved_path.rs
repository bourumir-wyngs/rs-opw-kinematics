use std::f64::consts::PI;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use serde::Deserialize;
use std::fs;
use std::time::Instant;
use rs_opw_kinematics::cartesian::{Cartesian, DEFAULT_TRANSITION_COSTS};
use rs_opw_kinematics::collisions::{CheckMode, CollisionBody, SafetyDistances, NEVER_COLLIDES};
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::kinematic_traits::{Kinematics, Pose, J2, J3, J4, J6, J_BASE, J_TOOL};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::rrt::RRTPlanner;
use rs_opw_kinematics::utils;

const HOME: [f64; 6] = [0.0, 1.451, -1.642, 0.0, 0.0, 0.0];

#[derive(Deserialize, Debug)]
struct Transform {
    position: Position,
    rotation: Rotation,
}

#[derive(Deserialize, Debug)]
struct Position {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Deserialize, Debug)]
struct Rotation {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

/// Reads a JSON file and parses it into a vector of `nalgebra::Isometry3<f64>`.
fn read_isometries_from_file(file_path: &str) -> Result<Vec<Isometry3<f64>>, Box<dyn std::error::Error>> {
    const SCALE: f64 = 0.005;
    // Read the JSON file as a string
    let file_content = fs::read_to_string(file_path)?;

    // Parse the JSON content into a vector of `Transform`
    let transforms: Vec<Transform> = serde_json::from_str(&file_content)?;

    // Convert the parsed transformations into nalgebra isometries
    let isometries: Vec<Isometry3<f64>> = transforms
        .into_iter()
        .map(|transform| {
            // Create translation
            let translation = Translation3::new(transform.position.x * SCALE, transform.position.y * SCALE, transform.position.z * SCALE);

            // Create a quaternion from the provided rotation
            let raw_quaternion = Quaternion::new(
                transform.rotation.w, // Scalar part
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
            );

            // Normalize the quaternion and construct a UnitQuaternion
            let unit_quaternion = UnitQuaternion::from_quaternion(raw_quaternion);

            // Combine translation and unit quaternion into an Isometry3
            Isometry3::from_parts(translation, unit_quaternion)
        })
        .collect();

    Ok(isometries)
}

pub fn create_rx160_robot() -> KinematicsWithShape {
    use rs_opw_kinematics::read_trimesh::{load_trimesh_from_stl, load_trimesh_from_ply };

    // Environment object to collide with.
    let monolith = load_trimesh_from_stl("src/tests/data/object.stl");

    KinematicsWithShape::with_safety(
        // OPW parameters for Staubli RX 160
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
        // Define constraints directly in degrees, converting internally to radians.
        Constraints::from_degrees(
            [
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -360.0..=360.0,
            ],
            BY_PREV, // Prioritize previous joint position
        ),
        // Joint meshes
        [
            // If your meshes, if offset in .stl file, use Trimesh::transform_vertices,
            // you may also need Trimesh::scale in some extreme cases.
            // If your joints or tool consist of multiple meshes, combine these
            // with Trimesh::append
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_1.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_2.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_3.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_4.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_5.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_6.stl"),
        ],
        // Base link mesh
        load_trimesh_from_stl("src/tests/data/staubli/rx160/base_link.stl"),
        // Base transform, this is where the robot is standing
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.7).into(),
            UnitQuaternion::from_euler_angles(0.0, PI, 0.0)
        ),
        // Tool mesh. Load it from .ply file for feature demonstration
        load_trimesh_from_ply("src/tests/data/stick.ply"),
        // Tool transform, tip (not base) of the tool. The point past this
        // transform is known as tool center point (TCP).
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.125).into(), // 0.125
            UnitQuaternion::identity()
        ),
        // Objects around the robot, with global transforms for them.
        vec![
        ],
        SafetyDistances {
            to_environment: 0.05,   // Robot should not come closer than 5 cm to pillars
            to_robot_default: 0.05, // No closer than 5 cm to itself.
            special_distances: SafetyDistances::distances(&[
                // Due construction of this robot, these joints are very close, so
                // special rules are needed for them.
                ((J2, J_BASE), NEVER_COLLIDES), // base and J2 cannot collide
                ((J3, J_BASE), NEVER_COLLIDES), // base and J3 cannot collide
                ((J2, J4), NEVER_COLLIDES),
                ((J3, J4), NEVER_COLLIDES),
                ((J4, J_TOOL), 0.02_f32), // reduce distance requirement to 2 cm.
                ((J4, J6), 0.02_f32),     // reduce distance requirement to 2 cm.
            ]),
            // mode: CheckMode::AllCollsions, // we need to report all for visualization
            mode: CheckMode::NoCheck, // this is very fast but no collision check
        },
    )
}

fn translate_along_local_z(isometry: &Isometry3<f64>, dz: f64) -> Isometry3<f64> {
    // Extract the rotation component as a UnitQuaternion
    let rotation = isometry.rotation;

    // Determine the local Z-axis direction (quaternion's orientation)
    let local_z_axis = rotation.transform_vector(&Vector3::z());

    // Compute the new translation by adding dz along the local Z-axis
    let translation = isometry.translation.vector + dz * local_z_axis;

    // Return a new Isometry3 with the updated translation and the same rotation
    Isometry3::from_parts(translation.into(), rotation.clone())
}

fn main() {
    use rs_cxx_ros2_opw_bridge::sender::Sender;

    // Call the function and read the isometries from the JSON file
    let isometries; 
    match read_isometries_from_file("work/isometries.json") {
        Ok(isos) => {
            println!("Isometries read successfully.");
            isometries = isos;
        }
        Err(err) => {
            eprintln!("Failed to read: {}", err);
            panic!();
        }
    }

    // Print the parsed isometries
    for (i, isometry) in isometries.iter().enumerate() {
        println!("Isometry #{}:\n{}", i + 1, isometry);
    }

    let sender = Sender::new("127.0.0.1", 5555);

    // Handle the result of `send_pose_message`
    match sender.send_pose_message(&isometries) {
        Ok(_) => {
            println!("Pose message sent successfully.");
        }
        Err(err) => {
            eprintln!("Failed to send pose message: {}", err);
        }
    }

    // Initialize kinematics with your robot's specific parameters
    let k = create_rx160_robot();

    // Starting point, where the robot exists at the beginning of the task.
    let start = HOME;

    // In production, other poses are normally given in Cartesian, but here they are given
    // in joints as this way it is easier to craft when in rs-opw-kinematics IDE.

    // "Landing" pose close to the surface, from where Cartesian landing on the surface
    // is possible and easy. Robot will change into one of the possible alternative configurations 
    // between start and land.
    let land = HOME;

    let steps = isometries;

    // "Parking" pose, Cartesian lifting from the surface at the end of the stroke. Park where we landed.
    let park = HOME;

    // Creat Cartesian planner
    let planner = Cartesian {
        robot: &k, // The robot, instance of KinematicsWithShape
        check_step_m: 0.02, // Pose distance check accuracy in meters (for translation)
        check_step_rad: 3.0_f64.to_radians(), // Pose distance check accuracy in radians (for rotation)
        max_transition_cost: 3_f64.to_radians(), // Maximal transition costs (not tied to the parameter above)
        // (weighted sum of abs differences between 'from' and 'to' for all joints, radians).
        transition_coefficients: DEFAULT_TRANSITION_COSTS, // Joint weights to compute transition cost
        linear_recursion_depth: 8,

        // RRT planner that computes the non-Cartesian path from starting position to landing pose
        rrt: RRTPlanner {
            step_size_joint_space: 2.0_f64.to_radians(), // RRT planner step in joint space
            max_try: 1000,
            debug: true,
        },
        include_linear_interpolation: true, // If true, intermediate Cartesian poses are
        // included in the output. Otherwise, they are checked but not included in the output
        debug: true,
    };

    // plan path
    let started = Instant::now();
    
    // TODO this needs implementation
    let land = steps.first().unwrap();
    let park = steps.last().unwrap();
    
    let path = planner.plan(&start, &land, &steps, &park);
    
    let elapsed = started.elapsed();

    match path {
        Ok(path) => {
            for joints in &path {
                println!("{:?}", &joints);
            }
            let joints: Vec<[f64; 6]> = path.into_iter().map(|aj| aj.joints.clone()).collect();
            sender.send_joint_trajectory_message(&joints);            
        }
        Err(message) => {
            println!("Failed: {}", message);
        }
    }
    println!("Took {:?}", elapsed); 
}
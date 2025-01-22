use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use rs_opw_kinematics::cartesian::{Cartesian, DEFAULT_TRANSITION_COSTS};
use rs_opw_kinematics::collisions::{CheckMode, CollisionBody, SafetyDistances, NEVER_COLLIDES, TOUCH_ONLY};
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::kinematic_traits::{J2, J3, J4, J6, J_BASE, J_TOOL};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::rrt::RRTPlanner;
use serde::Deserialize;
use std::f64::consts::PI;
use std::fs;
use std::time::Instant;
use rs_read_trimesh::load_trimesh;

// The initial position of the robotic arm.
// const HOME: [f64; 6] = [-2.0, 1.451, -1.642, 0.0, 0.0, 0.0];
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
fn read_isometries_from_file(
    file_path: &str,
) -> Result<Vec<Isometry3<f64>>, Box<dyn std::error::Error>> {
    // Read the JSON file as a string
    let file_content = fs::read_to_string(file_path)?;

    // Parse the JSON content into a vector of `Transform`
    let transforms: Vec<Transform> = serde_json::from_str(&file_content)?;

    // Convert the parsed transformations into nalgebra isometries
    let isometries: Vec<Isometry3<f64>> = transforms
        .into_iter()
        .map(|transform| {
            // Create translation
            let translation = Translation3::new(
                transform.position.x,
                transform.position.y,
                transform.position.z,
            );

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

pub fn create_rx160_robot() -> Result<KinematicsWithShape, String> {
    Ok(KinematicsWithShape::with_safety(
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
            load_trimesh("src/tests/data/staubli/rx160/link_1.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_2.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_3.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_4.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_5.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_6.stl", 1.0)?,
        ],
        // Base link mesh
        load_trimesh("src/tests/data/staubli/rx160/base_link.stl", 1.0)?,
        // Base transform, this is where the robot is standing
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.7).into(),
            UnitQuaternion::from_euler_angles(0.0, PI, 0.0),
        ),
        // Tool mesh. Load it from .ply file for feature demonstration
        load_trimesh("src/tests/data/stick.ply", 1.0)?,
        // Tool transform, tip (not base) of the tool. The point past this
        // transform is known as tool center point (TCP).
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.125).into(), // 0.125
            UnitQuaternion::identity(),
        ),
        // We use the Goblet in this task. It is sitting in the origin of coordinates.
        vec![CollisionBody {
            mesh: Box::new(load_trimesh("src/tests/data/goblet/goblet.ply", 1.0)?),
            pose: Isometry3::identity(),
        }],
        SafetyDistances {
            to_environment: TOUCH_ONLY,   // Robot should not come closer than 5 cm to the goblet
            to_robot_default: 0.05, // No closer than 5 cm to itself.
            special_distances: SafetyDistances::distances(&[
                // Due construction of this robot, these joints are very close, so
                // special rules are needed for them.
                ((J2, J_BASE), NEVER_COLLIDES), // base and J2 cannot collide
                ((J3, J_BASE), NEVER_COLLIDES), // base and J3 cannot collide
                ((J2, J4), NEVER_COLLIDES),
                ((J3, J4), NEVER_COLLIDES),
                ((J4, J_TOOL), 0.02_f32), // reduce distance requirement to 2 cm.
                ((J4, J6), TOUCH_ONLY),     // reduce distance requirement to 2 cm.
            ]),
            // mode: CheckMode::AllCollsions, // we need to report all for visualization
            mode: CheckMode::FirstCollisionOnly, // good for planning
            //mode: CheckMode::NoCheck
        },
    ))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    use rs_cxx_ros2_opw_bridge::sender::Sender;

    // Call the function and read the isometries from the JSON file
    let isometries;
    match read_isometries_from_file("src/tests/data/projector/r_Z_cyl.json") {
        Ok(isos) => {
            println!("Isometries read successfully.");
            isometries = isos;
        }
        Err(err) => {
            eprintln!("Failed to read: {}", err);
            panic!();
        }
    }

    let sender = Sender::new("127.0.0.1", 5555);

    // Handle the result of `send_pose_message`
    match sender.send_pose_message(&isometries) {
        Ok(_) => {
            println!("Pose message sent successfully, {} isometries.", isometries.len());
        }
        Err(err) => {
            eprintln!("Failed to send pose message: {}", err);
        }
    }

    // Initialize kinematics with your robot's specific parameters
    let k = create_rx160_robot()?;

    // Starting point, where the robot exists at the beginning of the task.
    let start = HOME;

    // In production, other poses are normally given in Cartesian, but here they are given
    // in joints as this way it is easier to craft when in rs-opw-kinematics IDE.
    let steps = isometries;

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
            waypoints: vec![] // vec![[-2.0, 1.451, -1.642, 0.0, 0.0, 0.0]]
        },
        include_linear_interpolation: true, // If true, intermediate Cartesian poses are
        // included in the output. Otherwise, they are checked but not included in the output
        cartesian_excludes_tool: true,
        time_out_seconds: 60,
        debug: true,
    };

    // plan path
    let started = Instant::now();

    let path = planner.plan(
        &start,
        &Cartesian::elevated_z(steps.first(), 0.1),
        &steps,
        &Cartesian::elevated_z(steps.last(), 0.1),
    );

    let elapsed = started.elapsed();

    match path {
        Ok(path) => {
            for joints in &path {
                println!("{:?}", &joints);
            }
            let joints: Vec<[f64; 6]> = path.into_iter().map(|aj| aj.joints.clone()).collect();
            sender.send_joint_trajectory_message(&joints)?
        }
        Err(message) => {
            println!("Failed: {}", message);
        }
    }
    println!("Took {:?}", elapsed);
    Ok(())
}

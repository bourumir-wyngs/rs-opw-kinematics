use anyhow::{Result, anyhow};

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
    rs_read_trimesh::load_trimesh
};
use rs_opw_kinematics::collisions::{CheckMode, SafetyDistances, NEVER_COLLIDES};
use rs_opw_kinematics::kinematic_traits::{J2, J3, J4, J6, J_BASE, J_TOOL};

#[cfg(feature = "stroke_planning")]
pub fn create_rx160_robot() -> anyhow::Result<KinematicsWithShape, String> {
    // Environment object to collide with.
    let monolith = load_trimesh("src/tests/data/object.stl", 1.0)?;

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
            Translation3::new(0.4, 0.7, 0.0).into(),
            UnitQuaternion::identity(),
        ),
        // Tool mesh. Load it from .ply file for feature demonstration
        load_trimesh("src/tests/data/flag.ply", 1.0)?,
        // Tool transform, tip (not base) of the tool. The point past this
        // transform is known as tool center point (TCP).
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.5).into(),
            UnitQuaternion::identity(),
        ),
        // Objects around the robot, with global transforms for them.
        vec![
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(1., 0., 0.),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(-1., 0., 0.),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(0., 1., 0.),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(0., -1., 0.),
            },
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
            mode: CheckMode::AllCollsions, // we need to report all for visualization
            // mode: CheckMode::NoCheck, // this is very fast but no collision check
        },
    ))
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
fn main() -> Result<()>{
    // Initialize kinematics with your robot's specific parameters
    let kinematics = create_rx160_robot().map_err(|e| anyhow!("Failed to create robot: {}", e))?;

    // This is a pretty tough path that requires to lift the initially low placed
    // tool over the obstacle and then lower again. The direct path is interrupted
    // by an obstacle.
    println!("** Tough example **");
    let start = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    let goal = utils::joints(&[40.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    example(start, goal, &kinematics)?;

    // Simple short step
    println!("** Simple example **");
    let start = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    let goal = utils::joints(&[-120.0, -80.0, -90., 18.42, 82.23, 189.35]);
    example(start, goal, &kinematics)?;
    
    Ok(())
}

#[cfg(feature = "stroke_planning")]
fn example(start: Joints, goal: Joints, kinematics: &KinematicsWithShape) -> Result<()>{
    let started = Instant::now();
    let path = plan_path(&kinematics, start, goal);
    let spent = started.elapsed();
    let result = convert_result(path);
    print_summary(&result);
    println!("Took {:?}", &spent);
    Ok(())
}

#[cfg(not(feature = "stroke_planning"))]
fn main() -> Result<()> {
    println!("Build configuration does not support this example");
    Ok(())
}
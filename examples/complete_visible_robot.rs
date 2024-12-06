use rs_opw_kinematics::collisions::{SafetyDistances, NEVER_COLLIDES};
use rs_opw_kinematics::kinematic_traits::{J2, J3, J4, J6, J_BASE, J_TOOL};
#[cfg(feature = "collisions")]
use {
    nalgebra::{Isometry3, Translation3, UnitQuaternion},

    rs_opw_kinematics::collisions::CollisionBody,

    rs_opw_kinematics::constraints::{Constraints, BY_PREV},
    rs_opw_kinematics::kinematic_traits::Kinematics,
    // This example only makes sense with collisions feature enabled
    // Visualization can optionally be disabled.
    rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape,
    rs_opw_kinematics::parameters::opw_kinematics::Parameters,
    rs_opw_kinematics::utils::dump_solutions,
    std::ops::RangeInclusive,
};

/// Creates a sample robot for visualization. This function sets up
/// a Staubli RX160 robot using its specific parameter set.
///
/// Joint meshes are loaded from `.stl` files bundled in the test folder,
/// shared under the Apache license as part of the ROS Industrial project.
///
/// Additionally, four environment objects and a tool are created for the visualization.
#[cfg(feature = "collisions")]
pub fn create_rx160_robot() -> KinematicsWithShape {
    use rs_opw_kinematics::read_trimesh::load_trimesh_from_stl;

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
            // If your mesh if offset in .stl file, use Trimesh::transform_vertices,
            // you may also need Trimesh::scale on some extreme cases.
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
            Translation3::new(0.4, 0.7, 0.0).into(),
            UnitQuaternion::identity(),
        ),
        // Tool mesh
        load_trimesh_from_stl("src/tests/data/flag.stl"),
        // Tool transform, tip (not base) of the tool. The point past this
        // transform is known as tool center point (TCP).
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.5).into(),
            UnitQuaternion::identity(),
        ),
        // Objects arround the robot, with global transforms for them.
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
        if cfg!(debug_assertions) {
            panic!("*** This example needs RELEASE BUILD to run in acceptable speed, sorry *** ")
        } else {
            println!("Release build, distance check active");
            SafetyDistances {
                to_environment: 0.05,   // Robot should not come closer than 5 cm to pillars
                to_robot_default: 0.05, // No closer than 5 cm to itself.
                special_distances: SafetyDistances::distances(&[
                    // Due construction of this robot, these joints are very close so
                    // special rules are needed for them.
                    ((J2, J_BASE), NEVER_COLLIDES), // base and J2 cannot collide
                    ((J3, J_BASE), NEVER_COLLIDES), // base and J3 cannot collide
                    ((J2, J4), NEVER_COLLIDES),
                    ((J3, J4), NEVER_COLLIDES),
                    ((J4, J_TOOL), 0.02_f32), // reduce distance requirement to 2 cm.
                    ((J4, J6), 0.02_f32),     // reduce distance requirement to 2 cm.
                ]),
                first_collision_only: false, // we need to report all for visualization
            }
        },
    )
}

/// This example builds and visualizes a complete robot using Bevy.
///
/// The visualization includes control sliders to adjust joint
/// angles, with real-time updates to the robot’s pose.
/// This feature is not part of the main library; rather, it is an
/// example intended to demonstrate functionality and confirm that
/// everything works as expected. You can modify this example to test
/// your own robot configuration.
#[cfg(feature = "collisions")]
fn main() {
    // The robot itself.
    let robot = create_rx160_robot();

    // Do some inverse kinematics to show the concept.
    let pose = Isometry3::from_parts(Translation3::new(0.0, 0.0, 1.5), UnitQuaternion::identity());

    let solutions = robot.inverse(&pose);
    dump_solutions(&solutions);

    if robot.collides(&[173_f64.to_radians(), 0., -94_f64.to_radians(), 0., 0., 0.]) {
        println!("Collision detected");
    }

    // In which position to show the robot on startup
    let intial_angles = [173., -8., -94., 6., 83., 207.];

    // Boundaries for XYZ drawbars in visualizaiton GUI
    let tcp_box: [RangeInclusive<f64>; 3] = [-2.0..=2.0, -2.0..=2.0, 1.0..=2.0];

    visualize(robot, intial_angles, tcp_box);
}

#[cfg(not(feature = "collisions"))]
fn main() {
    println!("Build configuration does not support this example")
}

#[cfg(feature = "visualization")]
fn visualize(
    robot: KinematicsWithShape,
    intial_angles: [f32; 6],
    tcp_box: [RangeInclusive<f64>; 3],
) {
    use rs_opw_kinematics::visualization;
    visualization::visualize_robot(robot, intial_angles, tcp_box);
}

#[cfg(all(feature = "collisions", not(feature = "visualization")))]
fn visualize(
    robot: KinematicsWithShape,
    intial_angles: [f32; 6],
    tcp_box: [RangeInclusive<f64>; 3],
    distances: &SafetyDistances,
) {
    println!("Build configuration does not support visualization")
}

use std::ops::RangeInclusive;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::joint_body::{CollisionBody};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::{utils, visualization};
use rs_opw_kinematics::kinematic_traits::Kinematics;
use rs_opw_kinematics::read_trimesh::load_trimesh_from_stl;
use rs_opw_kinematics::utils::{as_radians, dump_solutions};

/// Create the sample robot we will visualize. This function creates 
/// Staubli RX160, using is parameter set. 
/// It loads the joint meshes from .stl files bundles in the test folder
/// where they are shared under the rights of Apache license (ROS Industrial project).
/// Four environment objects and tool are also created.
pub fn create_rx160_robot() -> KinematicsWithShape {
    // Environment object to collide with.
    let monolith = load_trimesh_from_stl("src/tests/data/object.stl");

    KinematicsWithShape::new(
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

        // Define constraints. Use utils::joints to give them in degrees, not radians.
        Constraints::new(utils::joints(&[-225., -225., -225., -225., -225., -360.]),
                         utils::joints(&[225., 225., 225., 225., 225., 360.]),
                         // Take priority to the previous joint position (0.). Center of constraints can
                         // also be prioritized by passing BY_CONSTRAINS (1.) or any weighted value                         
                         BY_PREV),

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
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_6.stl")
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
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(-1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., 1., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., -1., 0.) }
        ],
        true // First pose only (true) or all poses (false)
    )
}

/// This example builds and visualizes complete robot, using Bevy.
/// The visualization provides control bars to change the joint
/// angles, the visualization of the robot is updated accordingly.
/// This visualization is not part of the main library but
/// rather example intended to show that everything works as 
/// expected. You can use the modified version to test your own
/// robot.

fn main() {
    // The robot itself.
    let robot = create_rx160_robot();

    // Do some inverse kinematics to show the concept.
    let pose = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 1.5),
        UnitQuaternion::identity());
    
    let solutions = robot.inverse(&pose);
    dump_solutions(&solutions);
    
    if robot.collides(&[173_f64.to_radians(), 0., -94_f64.to_radians(), 0., 0., 0.]) {
        println!("OOPS!");
    }

    // In which position to show the robot on startup
    let intial_angles = [173., -8., -94., 6., 83., 207.];

    // Boundaries for XYZ drawbars in visualizaiton GUI
    let tcp_box: [RangeInclusive<f64>; 3] = [-2.0..=2.0, -2.0..=2.0, 1.0..=2.0];

    visualization::visualize_robot(robot, intial_angles, tcp_box);
}

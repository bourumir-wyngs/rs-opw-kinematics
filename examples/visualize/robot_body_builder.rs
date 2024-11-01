use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_opw_kinematics::constraints::Constraints;
use rs_opw_kinematics::joint_body::{CollisionBody};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils;
use crate::read_trimesh::load_trimesh_from_stl;

/// Create the sample robot we will visualize. This function creates 
/// Staubli RX160, using is parameter set. 
/// It loads the joint meshes from .stl files bundles in the test folder
/// where they are shared under the rights of Apache license (ROS Industrial project).
/// Four environment objects and tool are also created.
pub fn create_rx160_robot() -> KinematicsWithShape {
    // Environment object to collide with.
    let monolith = load_trimesh_from_stl("src/tests/data/object.stl");

    KinematicsWithShape::new(
        // OPW parameters
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
                         0.0),
        
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
            Translation3::new(0.0, 0.0, 0.8).into(),
            UnitQuaternion::identity(),
        ),
        
        // Objects arround the robot, with global transforms for them.
        vec![
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(-1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., 1., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., -1., 0.) }
        ],
    )
}

use std::sync::Arc;
use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use rs_opw_kinematics::collisions::RobotBody;
use rs_opw_kinematics::joint_body::CollisionBody;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use crate::read_trimesh::load_trimesh_from_stl;

/// Create robot body for Staubli RS160, loading joint meshes from .stl files bundles in the test folder.
fn create_Staubli_RS160_joint_bodies() -> [TriMesh; 6] {
    // For the case of this specific robot, all links have identity local transform. This is not always the case. 
    // If the link is offset in its STL / PLY file, it must be "planted" in place by specifying the 
    // appropriate local transform. The offsets can be found in URDF file. 
    // They are link (not joint!) origins. Use
    // `collisions::transform_mesh` to offset the mesh as needed
    [
        load_trimesh_from_stl("src/tests/data/staubli/rx160/link_1.stl").unwrap(),
        load_trimesh_from_stl("src/tests/data/staubli/rx160/link_2.stl").unwrap(),
        load_trimesh_from_stl("src/tests/data/staubli/rx160/link_3.stl").unwrap(),
        load_trimesh_from_stl("src/tests/data/staubli/rx160/link_4.stl").unwrap(),
        load_trimesh_from_stl("src/tests/data/staubli/rx160/link_5.stl").unwrap(),
        load_trimesh_from_stl("src/tests/data/staubli/rx160/link_6.stl").unwrap()
    ]
}

/// Create the sample robot we will visualize. This function creates 
/// Staubli RX160, using built-in parameter set for it and loading
/// configuration. It loads the joint meshes from .stl files bundles in the test folder
/// where they are shared under the rights of Apache license (ROS Industrial project)
pub fn create_sample_robot() -> KinematicsWithShape {
    // Environment object
    let monolith = load_trimesh_from_stl("src/tests/data/object.stl").unwrap();
    
    KinematicsWithShape {
        kinematics: Arc::new(OPWKinematics::new(Parameters::staubli_rx160())),
        body: RobotBody {
            joint_meshes: create_Staubli_RS160_joint_bodies(),
            tool: Some(load_trimesh_from_stl("src/tests/data/flag.stl").unwrap()),
            base: None,
            collision_tolerance: 0.01,
            detect_first_collision_only: true,
            collision_environment: vec![
                CollisionBody {
                    mesh: monolith.clone(),
                    pose: Isometry3::translation(1., 0., 0.)
                },
                CollisionBody {
                    mesh: monolith.clone(),
                    pose: Isometry3::translation(-1., 0., 0.)
                },
                CollisionBody {
                    mesh: monolith.clone(),
                    pose: Isometry3::translation(0., 1., 0.)
                },
                CollisionBody {
                    mesh: monolith.clone(),
                    pose: Isometry3::translation(0., -1., 0.)
                }
            ]
                
        },
    }
}


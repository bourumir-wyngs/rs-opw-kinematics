use std::sync::Arc;
use nalgebra::Isometry3;
use rs_opw_kinematics::collisions::RobotBody;
use rs_opw_kinematics::joint_body::JointBody;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use crate::read_trimesh::load_trimesh_from_stl;

/// Create robot body, loading joint meshes from .stl files bundles in the test folder.
fn create_Staubli_RS160_joint_bodies() -> [JointBody; 6] {
    // For the case of this specific robot, all links have identity local transform. This is not always the case. 
    // If the link is offset in its STL / PLY file, it must be "planted" in place by specifying the 
    // appropriate local transform. The offsets can be found in URDF file. 
    // They are link (not joint!) origins.
    let identity = Isometry3::identity();
    [
        JointBody::new(load_trimesh_from_stl("src/tests/data/staubli/rx160/link_1.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("src/tests/data/staubli/rx160/link_2.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("src/tests/data/staubli/rx160/link_3.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("src/tests/data/staubli/rx160/link_4.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("src/tests/data/staubli/rx160/link_5.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("src/tests/data/staubli/rx160/link_6.stl").unwrap(), identity),
    ]
}

/// Create the sample robot we will visualize. This function creates 
/// Staubli RX160, using built-in parameter set for it and loading
/// configuration. It loads the joint meshes from .stl files bundles in the test folder
/// where they are shared under the rights of Apache license (ROS Industrial project)
pub fn create_sample_robot() -> KinematicsWithShape {
    KinematicsWithShape {
        kinematics: Arc::new(OPWKinematics::new(Parameters::staubli_rx160())),
        body:     RobotBody::new(create_Staubli_RS160_joint_bodies(),
                                 0.01,
                                 false)
    }
}


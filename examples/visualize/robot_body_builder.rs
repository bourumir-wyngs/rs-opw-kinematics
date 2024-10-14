use std::sync::Arc;
use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use rs_opw_kinematics::collisions::RobotBody;
use rs_opw_kinematics::joint_body::JointBody;
use rs_opw_kinematics::kinematic_traits::{Kinematics, JOINTS_AT_ZERO};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use crate::read_trimesh::load_trimesh_from_stl;

pub fn create_robot_body(kinematics: &dyn Kinematics) -> RobotBody {
    // Create 6 joint bodies, each with a pyramid shape and identity transformation
    let joint_bodies = create_six_joint_bodies();

    // Define a collision tolerance (distance threshold for detecting collisions)
    let collision_tolerance: f32 = 0.01;

    // Define whether to stop after the first collision is detected
    let detect_first_collision_only = false;

    let joint_origins: [Isometry3<f64>; 6] = kinematics.forward_with_joint_poses(&JOINTS_AT_ZERO);
    let joint_origins = joint_origins.map(|pose| pose.cast::<f32>());        
    
    // Create a new RobotBody with the 6 joint bodies, tolerance, and early-stop flag
    RobotBody::new(joint_bodies,
                   joint_origins,
                   collision_tolerance, 
                   detect_first_collision_only)
}

pub fn create_six_joint_bodies() -> [JointBody; 6] {
    // Use the identity transform for the local transformation of each joint
    let identity_transform = Isometry3::identity();

    // Create 6 joint bodies, each initialized with the pyramid shape and identity transformation
    [
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link1.stl").unwrap(), identity_transform),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link2.stl").unwrap(), identity_transform),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link3.stl").unwrap(), identity_transform),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link4.stl").unwrap(), identity_transform),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link5.stl").unwrap(), identity_transform),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link6.stl").unwrap(), identity_transform),
    ]
}


pub fn create_sample_robot() -> KinematicsWithShape {
    let kinematics = Arc::new(OPWKinematics::new(Parameters::staubli_tx2_160l()));
    let body = create_robot_body(kinematics.as_ref());
    KinematicsWithShape {
        kinematics,
        body: body
    }
}


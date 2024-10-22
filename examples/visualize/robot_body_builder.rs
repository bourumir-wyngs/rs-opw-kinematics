use std::sync::Arc;
use nalgebra::Isometry3;
use rs_opw_kinematics::collisions::RobotBody;
use rs_opw_kinematics::joint_body::JointBody;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use crate::read_trimesh::load_trimesh_from_stl;

pub fn create_robot_body() -> RobotBody {
    // Create 6 joint bodies, each with a pyramid shape and identity transformation
    let joint_bodies = create_six_joint_bodies();

    // Define a collision tolerance (distance threshold for detecting collisions)
    let collision_tolerance: f32 = 0.01;

    // Define whether to stop after the first collision is detected
    let detect_first_collision_only = false;
    
    // Create a new RobotBody with the 6 joint bodies, tolerance, and early-stop flag
    RobotBody::new(joint_bodies,
                   collision_tolerance, 
                   detect_first_collision_only)
}

pub fn create_six_joint_bodies() -> [JointBody; 6] {
    // For the case of this robot, all links have identity local transform. This is not always the case. If the link is offset in its
    // STL / PLY file, it must be "planted" in place by specifying the appropriate local transform ('origin' in URDF).
    let identity = Isometry3::identity();

    // Create 6 joint bodies, each initialized with the pyramid shape and identity transformation
    [
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_rx160_support/meshes/rx160/visual/link_1.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_rx160_support/meshes/rx160/visual/link_2.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_rx160_support/meshes/rx160/visual/link_3.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_rx160_support/meshes/rx160/visual/link_4.stl").unwrap(), identity), // Isometry3::translation(0.0, 0.0, -0.625)), // Second robot already with this bug. Check.
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_rx160_support/meshes/rx160/visual/link_5.stl").unwrap(), identity),
        JointBody::new(load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_rx160_support/meshes/rx160/visual/link_6.stl").unwrap(), identity),
    ]
}

pub fn _create_six_joint_bodies() -> [JointBody; 6] {
    // For the case of this robot, all links have identity local transform. This is not always the case. If the link is offset in its
    // STL / PLY file, it must be "planted" in place by specifying the appropriate local transform ('origin' in URDF).

    // Create 6 joint bodies, each initialized with the pyramid shape and identity transformation
    [
        JointBody::new(
            load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link1.stl").unwrap(),
            Isometry3::translation(0.0, 0.0, -0.55)
        ),
        JointBody::new(
            load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link2.stl").unwrap(),
            Isometry3::translation(-0.15, 0.0, -0.55)
        ),
        JointBody::new(
            load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link3.stl").unwrap(),
            Isometry3::translation(-0.15, 0.0, -1.375)
        ),
        JointBody::new(
            load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link4.stl").unwrap(),
            Isometry3::translation(-0.15, 0.0, -1.375) 
        ),
        JointBody::new(
            load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link5.stl").unwrap(),
            Isometry3::translation(-0.15, 0.0, -2.3)
        ),
        JointBody::new(
            load_trimesh_from_stl("/home/audrius/opw/staubli/staubli_tx2_160l_support/meshes/tx2_160l/visual/link6.stl").unwrap(),
            Isometry3::translation(-0.15, 0.0, -2.41)
        ),
    ]
}


pub fn create_sample_robot() -> KinematicsWithShape {
    KinematicsWithShape {
        kinematics: Arc::new(OPWKinematics::new(Parameters::staubli_rx160())),
        body: create_robot_body()
    }
}


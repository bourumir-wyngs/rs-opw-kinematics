use std::sync::Arc;
use nalgebra::Isometry3;
use parry3d::shape::TriMesh;
use rs_opw_kinematics::collisions::RobotBody;
use rs_opw_kinematics::joint_body::JointBody;
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;

pub fn create_robot_body() -> RobotBody {
    // Create 6 joint bodies, each with a pyramid shape and identity transformation
    let joint_bodies = create_six_joint_bodies();

    // Define a collision tolerance (distance threshold for detecting collisions)
    let collision_tolerance: f32 = 0.01;

    // Define whether to stop after the first collision is detected
    let detect_first_collision_only = false;

    let joint_origins: [Isometry3<f32>; 6] = [
        Isometry3::translation(0.0, 0.0, 0.0),
        Isometry3::translation(2.0, 0.0, 2.0),
        Isometry3::translation(0.0, 4.0, 0.0),
        Isometry3::translation(0.3, 0.0, 8.0),
        Isometry3::translation(0.0, 0.0, 0.0),
        Isometry3::translation(0.0, 0.0, 0.0),
    ];

    // Create a new RobotBody with the 6 joint bodies, tolerance, and early-stop flag
    RobotBody::new(joint_bodies,
                   joint_origins,
                   collision_tolerance, 
                   detect_first_collision_only)
}

pub fn create_six_joint_bodies() -> [JointBody; 6] {
    // Create the pyramid shape to be used for each joint
    let pyramid_shape = create_pyramid();

    // Use the identity transform for the local transformation of each joint
    let identity_transform = Isometry3::identity();

    // Create 6 joint bodies, each initialized with the pyramid shape and identity transformation
    [
        JointBody::new(pyramid_shape.clone(), identity_transform),
        JointBody::new(pyramid_shape.clone(), identity_transform),
        JointBody::new(pyramid_shape.clone(), identity_transform),
        JointBody::new(pyramid_shape.clone(), identity_transform),
        JointBody::new(pyramid_shape.clone(), identity_transform),
        JointBody::new(pyramid_shape.clone(), identity_transform),
    ]
}

pub fn create_pyramid() -> TriMesh {
    let vertices = vec![
        nalgebra::Point3::new(1.0, 1.0, 1.0),     // V0
        nalgebra::Point3::new(-1.0, -1.0, 1.0),   // V1
        nalgebra::Point3::new(-1.0, 1.0, -1.0),   // V2
        nalgebra::Point3::new(1.0, -1.0, -1.0),   // V3
    ];

    let indices = vec![
        [0, 1, 2],  // Face 1: (V0, V1, V2)
        [0, 3, 1],  // Face 2: (V0, V3, V1)
        [0, 2, 3],  // Face 3: (V0, V2, V3)
        [1, 3, 2],  // Face 4: (V1, V3, V2)
    ];

    TriMesh::new(vertices, indices)
}


pub fn create_sample_robot() -> KinematicsWithShape {
    let kinematics = Arc::new(OPWKinematics::new(Parameters::staubli_tx2_160l()));
    KinematicsWithShape {
        kinematics,
        body: create_robot_body()
    }
}


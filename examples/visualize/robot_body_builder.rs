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
        // Reoriented vertices to lie in the x-y plane with the top at z = 1.0
        nalgebra::Point3::new(0.0, 0.0, 1.0),     // V0 (Top of the pyramid, facing z+)
        nalgebra::Point3::new(1.0, -1.0, -1.0),   // V1 (Base in the x-y plane)
        nalgebra::Point3::new(-1.0, -1.0, -1.0),  // V2 (Base in the x-y plane)
        nalgebra::Point3::new(0.0, 1.0, -1.0),    // V3 (Base in the x-y plane)
    ];

    let indices = vec![
        [0, 1, 2],  // Face 1: (V0, V1, V2)
        [0, 2, 3],  // Face 2: (V0, V2, V3)
        [0, 3, 1],  // Face 3: (V0, V3, V1)
        [1, 2, 3],  // Face 4: Base (V1, V2, V3)
    ];

    // To center the mass, calculate the centroid of the vertices
    let centroid = nalgebra::Point3::new(
        (0.0 + 1.0 + -1.0 + 0.0) / 4.0,  // Average x
        (0.0 + -1.0 + -1.0 + 1.0) / 4.0, // Average y
        (1.0 + -1.0 + -1.0 + -1.0) / 4.0 // Average z
    );

    // Translate the vertices so that the centroid is at (0, 0, 0)
    let centered_vertices: Vec<nalgebra::Point3<f32>> = vertices
        .into_iter()
        .map(|v| nalgebra::Point3::new(
            v.x - centroid.x,
            v.y - centroid.y,
            v.z - centroid.z
        ))
        .collect();

    TriMesh::new(centered_vertices, indices)
}


pub fn create_sample_robot() -> KinematicsWithShape {
    let kinematics = Arc::new(OPWKinematics::new(Parameters::staubli_tx2_160l()));
    KinematicsWithShape {
        kinematics,
        body: create_robot_body()
    }
}


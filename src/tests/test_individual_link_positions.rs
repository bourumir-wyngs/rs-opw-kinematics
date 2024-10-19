use std::sync::Arc;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use crate::frame::Frame;
use crate::kinematic_traits::{Kinematics, Pose};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;
use crate::tests::test_utils::are_isometries_approx_equal;
use crate::tool::{Base, Tool};

const SMALL: f64 = 1e-6;

#[test]
fn test_forward_kinematics_straight_up() {
    // Define the joint angles for pointing the robot straight up (all 0)
    let joints: [f64; 6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

    let parameters = Parameters {
        a1: 0.150,
        a2: 0.00017,
        b: 0.0002,
        c1: 0.551,
        c2: 0.552,
        c3: 0.600,
        c4: 0.110,
        ..Parameters::new() // Any other required fields
    };

    let opw_kinematics: OPWKinematics = OPWKinematics::new(parameters);
    let poses = opw_kinematics.forward_with_joint_poses(&joints);

    let (a1, a2, b, c1, c2, c3, c4) = (
        parameters.a1,
        parameters.a2,
        parameters.b,
        parameters.c1,
        parameters.c2,
        parameters.c3,
        parameters.c4
    );

    // Expected positions
    let expected_positions = [
        (0.0, 0.0, c1), // 1
        (a1, 0.0, c1),  // 2
        (a1, b, c1 + c2),  // 3 
        (a1 + a2, b, c1 + c2),  // 4
        (a1 + a2, b, c1 + c2 + c3),  // 5
        (a1 + a2, b, c1 + c2 + c3 + c4),  // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing: UnitQuaternion<f64> = UnitQuaternion::identity();

    // Check that the quaternion (rotation) is identity quaternion for all poses
    for (i, pose) in poses.iter().enumerate() {
        check_rotation(standing, i, &pose.rotation);
    }
    
    // Check also if the tcp-only version provides the same output
    let tcp = opw_kinematics.forward(&joints); 
    assert!(are_isometries_approx_equal(&tcp, &poses[5], SMALL));
}

#[test]
fn test_forward_kinematics_straight_up_2() {
    // Define the joint angles for pointing the robot straight up (all 0)
    let joints: [f64; 6] = [0.0, -std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    let parameters = Parameters {
        a1: 0.1,
        a2: -0.135,
        b: 0.,
        c1: 0.615,
        c2: 0.705,
        c3: 0.755,
        c4: 0.085,
        offsets: [0.0, -std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0],
        ..Parameters::new() // Any other required fields
    };

    let opw_kinematics: OPWKinematics = OPWKinematics::new(parameters);
    let poses = opw_kinematics.forward_with_joint_poses(&joints);

    let (a1, a2, b, c1, c2, c3, c4) = (
        parameters.a1,
        parameters.a2,
        parameters.b,
        parameters.c1,
        parameters.c2,
        parameters.c3,
        parameters.c4
    );

    // Expected positions
    let expected_positions = [
        (0.0, 0.0, c1), // 1
        (a1, 0.0, c1),  // 2
        (a1, b, c1 + c2),  // 3 
        (a1 + a2, b, c1 + c2),  // 4
        (a1 + a2, b, c1 + c2 + c3),  // 5
        (a1 + a2, b, c1 + c2 + c3 + c4),  // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing: UnitQuaternion<f64> = UnitQuaternion::identity();

    // Check that the quaternion (rotation) is identity quaternion for all poses
    for (i, pose) in poses.iter().enumerate() {
        check_rotation(standing, i, &pose.rotation);
    }

    // Check also if the tcp-only version provides the same output
    let tcp = opw_kinematics.forward(&joints);
    assert!(are_isometries_approx_equal(&tcp, &poses[5], SMALL));
}


#[test]
fn test_forward_kinematics_j2_rotated_90_degrees() {
    // Define the joint angles: J2 rotated 90 degrees (π/2), other joints are 0
    let joints = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    let parameters = Parameters {
        a1: 0.150,
        a2: 0.00017,
        b: 0.0002,
        c1: 0.551,
        c2: 0.552,
        c3: 0.600,
        c4: 0.110,
        ..Parameters::new() // Any other required fields
    };

    let opw_kinematics: OPWKinematics = OPWKinematics::new(parameters);

    let (a1, a2, b, c1, c2, c3, c4) = (
        parameters.a1,
        parameters.a2,
        parameters.b,
        parameters.c1,
        parameters.c2,
        parameters.c3,
        parameters.c4
    );
    let poses = opw_kinematics.forward_with_joint_poses(&joints);

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2
    let expected_positions = [
        (0.0, 0.0, c1), // 1
        (a1, 0.0, c1), // 2
        (a1 + c2, b, c1), // 3
        (a1 + c2, b, c1 - a2), // 4
        (a1 + c2 + c3, b, c1 - a2), // 5
        (a1 + c2 + c3 + c4, b, c1 - a2), // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing: UnitQuaternion<f64> = UnitQuaternion::identity();
    let lying = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);

    // Check quaternions for each pose (after J2 rotation)
    for (i, pose) in poses.iter().enumerate() {
        // Pose 1 and Pose 2 should still have identity quaternions (no rotation applied)
        if i == 0 {
            check_rotation(standing, i, &pose.rotation);
        } else {
            // From Pose 3 onwards, J2 rotation is applied (90 degrees around Y-axis)
            check_rotation(lying, i, &pose.rotation);
        }
    }

    // Check also if the tcp-only version provides the same output
    let tcp = opw_kinematics.forward(&joints);
    assert!(are_isometries_approx_equal(&tcp, &poses[5], SMALL));    
}

#[test]
fn test_forward_kinematics_j2_rotated_90_degrees_2() {
    // Define the joint angles: J2 rotated 90 degrees (π/2), other joints are 0
    let joints = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    let parameters = Parameters {
        a1: 0.1,
        a2: -0.135,
        b: 0.,
        c1: 0.615,
        c2: 0.705,
        c3: 0.755,
        c4: 0.085,
        ..Parameters::new() // Any other required fields
    };

    let opw_kinematics: OPWKinematics = OPWKinematics::new(parameters);

    let (a1, a2, b, c1, c2, c3, c4) = (
        parameters.a1,
        parameters.a2,
        parameters.b,
        parameters.c1,
        parameters.c2,
        parameters.c3,
        parameters.c4
    );
    let poses = opw_kinematics.forward_with_joint_poses(&joints);

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2
    let expected_positions = [
        (0.0, 0.0, c1), // 1
        (a1, 0.0, c1), // 2
        (a1 + c2, b, c1), // 3
        (a1 + c2, b, c1 - a2), // 4
        (a1 + c2 + c3, b, c1 - a2), // 5
        (a1 + c2 + c3 + c4, b, c1 - a2), // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing: UnitQuaternion<f64> = UnitQuaternion::identity();
    let lying = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);

    // Check quaternions for each pose (after J2 rotation)
    for (i, pose) in poses.iter().enumerate() {
        // Pose 1 and Pose 2 should still have identity quaternions (no rotation applied)
        if i == 0 {
            check_rotation(standing, i, &pose.rotation);
        } else {
            // From Pose 3 onwards, J2 rotation is applied (90 degrees around Y-axis)
            check_rotation(lying, i, &pose.rotation);
        }
    }

    // Check also if the tcp-only version provides the same output
    let tcp = opw_kinematics.forward(&joints);
    assert!(are_isometries_approx_equal(&tcp, &poses[5], SMALL));
}

#[test]
fn test_tool_forward_kinematics() {
    let tool_offset = 1.0;

    // Create an instance of OPWKinematics
    let parameters: Parameters = create_parameters();
    let robot_without_tool = OPWKinematics::new(parameters);

    // Tool extends 1 meter in the Z direction
    let tool_translation = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, tool_offset).into(),
        UnitQuaternion::identity(),
    );

    // Create the Tool instance with the transformation
    let robot_with_tool = Tool {
        robot: Arc::new(robot_without_tool),
        tool: tool_translation,
    };

    // Define the joint angles: J2 rotated 90 degrees (π/2), other joints are 0
    let joints = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    // Compute forward kinematics for the given joint angles
    let poses = robot_with_tool.forward_with_joint_poses(&joints);

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2, plus the tool offset in Z direction
    let expected_positions = [
        (0.0, 0.0, parameters.c1),
        (parameters.a1, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2 + parameters.c3, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2 + parameters.c3, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2 + parameters.c3 + parameters.c4 + tool_offset, 0.0, parameters.c1), // Tool adds 1 meter in X
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing: UnitQuaternion<f64> = UnitQuaternion::identity();
    let lying = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);

    // Check quaternions for each pose (after J2 rotation)
    for (i, pose) in poses.iter().enumerate() {
        // Pose 1 and Pose 2 should still have identity quaternions (no rotation applied)
        if i <= 1 {
            check_rotation(standing, i, &pose.rotation);
        } else {
            // From Pose 3 onwards, J2 rotation is applied (90 degrees around Y-axis)
            check_rotation(lying, i, &pose.rotation);
        }
    }
}

#[test]
fn test_frame_forward_kinematics() {
    let frame_offset = 1.0;

    // Create an instance of OPWKinematics
    let parameters: Parameters = create_parameters();
    let robot_without_tool = OPWKinematics::new(parameters);

    // Tool extends 1 meter in the Z direction
    let frame_translation = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, frame_offset).into(),
        UnitQuaternion::identity(),
    );

    // Create the Tool instance with the transformation
    let robot_with_tool = Frame {
        robot: Arc::new(robot_without_tool),
        frame: frame_translation,
    };

    // Define the joint angles: J2 rotated 90 degrees (π/2), other joints are 0
    let joints = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    // Compute forward kinematics for the given joint angles
    let poses = robot_with_tool.forward_with_joint_poses(&joints);

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2, plus the tool offset in Z direction
    let expected_positions = [
        (0.0, 0.0, parameters.c1),
        (parameters.a1, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2 + parameters.c3, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2 + parameters.c3, 0.0, parameters.c1),
        (parameters.a1 + parameters.c2 + parameters.c3 + parameters.c4 + frame_offset, 0.0, parameters.c1), // Frame adds 1 meter in X
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing: UnitQuaternion<f64> = UnitQuaternion::identity();
    let lying = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);

    // Check quaternions for each pose (after J2 rotation)
    for (i, pose) in poses.iter().enumerate() {
        // Pose 1 and Pose 2 should still have identity quaternions (no rotation applied)
        if i <= 1 {
            check_rotation(standing, i, &pose.rotation);
        } else {
            // From Pose 3 onwards, J2 rotation is applied (90 degrees around Y-axis)
            check_rotation(lying, i, &pose.rotation);
        }
    }
}

#[test]
fn test_base_forward_kinematics() {
    let base_height = 1.0;

    let parameters: Parameters = create_parameters();
    let robot_without_base = OPWKinematics::new(parameters);

    // 1 meter high pedestal
    let base_translation = Isometry3::from_parts(
        Translation3::new(0.0, 0.0, base_height).into(),
        UnitQuaternion::identity(),
    );

    // Create the Base instance with the transformation
    let robot_with_base = Base {
        robot: Arc::new(robot_without_base),
        base: base_translation,
    };

    // Define the joint angles: J2 rotated 90 degrees (π/2), other joints are 0
    let joints = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    // Compute forward kinematics for the given joint angles
    let poses = robot_with_base.forward_with_joint_poses(&joints);

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2, plus the base offset in Z direction
    let expected_positions = [
        (0.0, 0.0, parameters.c1 + base_height), // Base adds 1 meter in Z
        (parameters.a1, 0.0, parameters.c1 + base_height),
        (parameters.a1 + parameters.c2, 0.0, parameters.c1 + base_height),
        (parameters.a1 + parameters.c2 + parameters.c3, 0.0, parameters.c1 + base_height),
        (parameters.a1 + parameters.c2 + parameters.c3, 0.0, parameters.c1 + base_height),
        (parameters.a1 + parameters.c2 + parameters.c3 + parameters.c4, 0.0, parameters.c1 + base_height),
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing: UnitQuaternion<f64> = UnitQuaternion::identity();
    let lying = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);

    // Check quaternions for each pose (after J2 rotation)
    for (i, pose) in poses.iter().enumerate() {
        // Pose 1 and Pose 2 should still have identity quaternions (no rotation applied)
        if i <= 1 {
            check_rotation(standing, i, &pose.rotation);
        } else {
            // From Pose 3 onwards, J2 rotation is applied (90 degrees around Y-axis)
            check_rotation(lying, i, &pose.rotation);
        }
    }
}

fn create_parameters() -> Parameters {
    let parameters = Parameters {
        a1: 0.150,
        a2: 0.00017,
        b: 0.0002,
        c1: 0.551,
        c2: 0.552,
        c3: 0.600,
        c4: 0.110,
        ..Parameters::new() // Any other required fields
    };
    parameters
}

fn check_xyz(poses: [Pose; 6], expected_positions: [(f64, f64, f64); 6]) {
    for (i, &(expected_x, expected_y, expected_z)) in expected_positions.iter().enumerate() {
        let translation = poses[i].translation.vector;

        // X, Y, and Z coordinates should match the expected positions
        assert!((translation[0] as f64 - expected_x).abs() < SMALL, "Pose {} X- expected {}, got {}", i + 1, expected_x, translation[0]);
        assert!((translation[1] as f64 - expected_y).abs() < SMALL, "Pose {} Y- expected {}, got {}", i + 1, expected_y, translation[1]);
        assert!((translation[2] as f64 - expected_z).abs() < SMALL, "Pose {} Z- expected {}, got {}", i + 1, expected_z, translation[2]);
    }
}

fn check_rotation(standing: UnitQuaternion<f64>, i: usize, quaternion: &UnitQuaternion<f64>) {
    assert!(
        (quaternion.w as f64 - standing.w).abs() < SMALL,
        "Pose {} quaternion w mismatch", i + 1
    );
    assert!(
        (quaternion.i as f64 - standing.i).abs() < SMALL,
        "Pose {} quaternion i mismatch", i + 1
    );
    assert!(
        (quaternion.j as f64 - standing.j).abs() < SMALL,
        "Pose {} quaternion j mismatch", i + 1
    );
    assert!(
        (quaternion.k as f64 - standing.k).abs() < SMALL,
        "Pose {} quaternion k mismatch", i + 1
    );
}

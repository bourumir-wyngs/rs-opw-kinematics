use crate::frame::Frame;
use crate::kinematic_traits::{Kinematics, Pose};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;
use crate::tests::test_utils::are_poses_approx_equal;
use crate::tool::Base;
use glam::{DQuat, DVec3};
use std::sync::Arc;

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
        parameters.c4,
    );

    // Expected positions
    let expected_positions = [
        (0.0, 0.0, c1),                  // 1
        (a1, b, c1),                     // 2
        (a1, b, c1 + c2),                // 3
        (a1 + a2, b, c1 + c2),           // 4
        (a1 + a2, b, c1 + c2 + c3),      // 5
        (a1 + a2, b, c1 + c2 + c3 + c4), // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing = DQuat::IDENTITY;

    // Check that the quaternion (rotation) is identity quaternion for all poses
    for (i, pose) in poses.iter().enumerate() {
        check_rotation(standing, i, &pose.rotation);
    }

    // Check also if the tcp-only version provides the same output
    let tcp = opw_kinematics.forward(&joints);
    assert!(are_poses_approx_equal(&tcp, &poses[5], SMALL));
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
        parameters.c4,
    );

    // Expected positions
    let expected_positions = [
        (0.0, 0.0, c1),                  // 1
        (a1, b, c1),                     // 2
        (a1, b, c1 + c2),                // 3
        (a1 + a2, b, c1 + c2),           // 4
        (a1 + a2, b, c1 + c2 + c3),      // 5
        (a1 + a2, b, c1 + c2 + c3 + c4), // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing = DQuat::IDENTITY;

    // Check that the quaternion (rotation) is identity quaternion for all poses
    for (i, pose) in poses.iter().enumerate() {
        check_rotation(standing, i, &pose.rotation);
    }

    // Check also if the tcp-only version provides the same output
    let tcp = opw_kinematics.forward(&joints);
    assert!(are_poses_approx_equal(&tcp, &poses[5], SMALL));
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
        parameters.c4,
    );
    let poses = opw_kinematics.forward_with_joint_poses(&joints);

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2
    let expected_positions = [
        (0.0, 0.0, c1),                  // 1
        (a1, b, c1),                     // 2
        (a1 + c2, b, c1),                // 3
        (a1 + c2, b, c1 - a2),           // 4
        (a1 + c2 + c3, b, c1 - a2),      // 5
        (a1 + c2 + c3 + c4, b, c1 - a2), // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing = DQuat::IDENTITY;
    let lying = DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);

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
    assert!(are_poses_approx_equal(&tcp, &poses[5], SMALL));
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
        parameters.c4,
    );
    let poses = opw_kinematics.forward_with_joint_poses(&joints);

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2
    let expected_positions = [
        (0.0, 0.0, c1),                  // 1
        (a1, 0.0, c1),                   // 2
        (a1 + c2, b, c1),                // 3
        (a1 + c2, b, c1 - a2),           // 4
        (a1 + c2 + c3, b, c1 - a2),      // 5
        (a1 + c2 + c3 + c4, b, c1 - a2), // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing = DQuat::IDENTITY;
    let lying = DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);

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
    assert!(are_poses_approx_equal(&tcp, &poses[5], SMALL));
}

#[test]
fn test_frame_forward_kinematics() {
    let frame_offset = 1.0;

    // Create an instance of OPWKinematics
    let parameters: Parameters = create_parameters();
    let robot_without_frame = OPWKinematics::new(parameters);

    // Tool extends 1 meter in the Z direction
    let frame_translation = Frame::translation(DVec3::ZERO, DVec3::new(0.0, 0.0, frame_offset));

    // Create the Tool instance with the transformation
    let robot_with_frame = Frame {
        robot: Arc::new(robot_without_frame),
        frame: frame_translation,
    };

    // Define the joint angles: J2 rotated 90 degrees (π/2), other joints are 0
    let joints = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    // Compute forward kinematics for the given joint angles
    let poses = robot_with_frame.forward_with_joint_poses(&joints);

    let (a1, a2, b, c1, c2, c3, c4) = (
        parameters.a1,
        parameters.a2,
        parameters.b,
        parameters.c1,
        parameters.c2,
        parameters.c3,
        parameters.c4,
    );

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2
    let expected_positions = [
        (0.0, 0.0, c1),                                 // 1
        (a1, b, c1),                                    // 2
        (a1 + c2, b, c1),                               // 3
        (a1 + c2, b, c1 - a2),                          // 4
        (a1 + c2 + c3, b, c1 - a2),                     // 5
        (a1 + c2 + c3 + c4, b, c1 - a2 + frame_offset), // 6
    ];

    // Check all poses for correct X, Y, and Z translation
    check_xyz(poses, expected_positions);

    let standing = DQuat::IDENTITY;
    let lying = DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);

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
    let tcp = robot_with_frame.forward(&joints);
    assert!(are_poses_approx_equal(&tcp, &poses[5], SMALL));
}

#[test]
fn test_base_forward_kinematics() {
    let base_height = 1.0;

    let parameters: Parameters = create_parameters();
    let robot_without_base = OPWKinematics::new(parameters);

    // 1 meter high pedestal
    let base_translation = Pose::from_translation(DVec3::new(0.0, 0.0, base_height));

    // Create the Base instance with the transformation
    let robot_with_base = Base {
        robot: Arc::new(robot_without_base),
        base: base_translation,
    };

    // Define the joint angles: J2 rotated 90 degrees (π/2), other joints are 0
    let joints = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];

    // Compute forward kinematics for the given joint angles
    let poses = robot_with_base.forward_with_joint_poses(&joints);

    let (a1, a2, b, c1, c2, c3, c4) = (
        parameters.a1,
        parameters.a2,
        parameters.b,
        parameters.c1,
        parameters.c2,
        parameters.c3,
        parameters.c4,
    );

    // Expected positions: Robot should extend horizontally due to 90-degree rotation of J2, plus the base offset in Z direction
    let expected_positions = [
        (0.0, 0.0, c1 + base_height),                  // 1
        (a1, b, c1 + base_height),                     // 2
        (a1 + c2, b, c1 + base_height),                // 3
        (a1 + c2, b, c1 - a2 + base_height),           // 4
        (a1 + c2 + c3, b, c1 - a2 + base_height),      // 5
        (a1 + c2 + c3 + c4, b, c1 - a2 + base_height), // 6
    ];

    check_xyz(poses, expected_positions);

    let standing = DQuat::IDENTITY;
    let lying = DQuat::from_rotation_y(std::f64::consts::FRAC_PI_2);

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
    let tcp = robot_with_base.forward(&joints);
    assert!(are_poses_approx_equal(&tcp, &poses[5], SMALL));
}

fn create_parameters() -> Parameters {
    Parameters {
        a1: 0.150,
        a2: 0.00017,
        b: 0.0002,
        c1: 0.551,
        c2: 0.552,
        c3: 0.600,
        c4: 0.110,
        ..Parameters::new() // Any other required fields
    }
}

fn check_xyz(poses: [Pose; 6], expected_positions: [(f64, f64, f64); 6]) {
    for (i, &(expected_x, expected_y, expected_z)) in expected_positions.iter().enumerate() {
        let translation = poses[i].translation;

        // X, Y, and Z coordinates should match the expected positions
        assert!(
            (translation[0] - expected_x).abs() < SMALL,
            "Pose {} X- expected {}, got {}",
            i + 1,
            expected_x,
            translation[0]
        );
        assert!(
            (translation[1] - expected_y).abs() < SMALL,
            "Pose {} Y- expected {}, got {}",
            i + 1,
            expected_y,
            translation[1]
        );
        assert!(
            (translation[2] - expected_z).abs() < SMALL,
            "Pose {} Z- expected {}, got {}",
            i + 1,
            expected_z,
            translation[2]
        );
    }
}

fn check_rotation(standing: DQuat, i: usize, quaternion: &DQuat) {
    assert!(
        quaternion.angle_between(standing) < SMALL,
        "Pose {} quaternion mismatch",
        i + 1
    );
}

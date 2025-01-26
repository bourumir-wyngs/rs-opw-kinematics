use nalgebra::{Isometry3, Unit, Vector3};
use crate::cartesian::{AnnotatedPose, PathFlags};

/// Create a list of slightly altered poses
pub(crate) fn alter_poses(pose: &AnnotatedPose, angle_offset: f64) -> Vec<AnnotatedPose> {
    // Helper function to create a rotated pose along a specific axis
    fn rotated_pose(base_pose: &AnnotatedPose, axis: Vector3<f64>, angle: f64) -> AnnotatedPose {
        // Convert the axis into a unit vector
        let unit_axis = Unit::new_normalize(axis);
        let additional_rotation = nalgebra::UnitQuaternion::from_axis_angle(&unit_axis, angle);
        let new_pose = Isometry3 {
            rotation: base_pose.pose.rotation * additional_rotation,
            translation: base_pose.pose.translation,
        };
        AnnotatedPose {
            pose: new_pose,
            flags: base_pose.flags | PathFlags::ALTERED,
        }
    }

    // Generate individual alterations for roll (X), pitch (Y), and yaw (Z)
    let single_alterations = vec![
        rotated_pose(pose, Vector3::x(), angle_offset),  // +Roll
        rotated_pose(pose, Vector3::x(), -angle_offset), // -Roll
        rotated_pose(pose, Vector3::y(), angle_offset),  // +Pitch
        rotated_pose(pose, Vector3::y(), -angle_offset), // -Pitch
        rotated_pose(pose, Vector3::z(), angle_offset),  // +Yaw
        rotated_pose(pose, Vector3::z(), -angle_offset), // -Yaw
    ];

    let mut double_alterations = Vec::with_capacity(104);

    // Generate paired (two-step) alterations and consider both orders
    for i in 0..single_alterations.len() {
        for j in 0..single_alterations.len() {
            if i / 2 == j / 2 && i % 2 != j % 2 {
                // Skip if the two alterations cancel out (rotating the same axis in opposite directions)
                continue;
            }

            // First apply alteration i, then alteration j
            let first_then_second = rotated_pose(
                &single_alterations[i],
                match j {
                    0 | 1 => Vector3::x(), // Roll (X-axis)
                    2 | 3 => Vector3::y(), // Pitch (Y-axis)
                    _ => Vector3::z(),     // Yaw (Z-axis)
                },
                if j % 2 == 0 {
                    angle_offset // Positive rotation
                } else {
                    -angle_offset // Negative rotation
                },
            );

            // Then apply alteration j, then alteration i
            let second_then_first = rotated_pose(
                &single_alterations[j],
                match i {
                    0 | 1 => Vector3::x(), // Roll (X-axis)
                    2 | 3 => Vector3::y(), // Pitch (Y-axis)
                    _ => Vector3::z(),     // Yaw (Z-axis)
                },
                if i % 2 == 0 {
                    angle_offset // Positive rotation
                } else {
                    -angle_offset // Negative rotation
                },
            );

            // Add both poses to the alterations
            double_alterations.push(first_then_second);
            double_alterations.push(second_then_first);
        }
    }

    // Add 3-step combinations (48 cases) with unique, non-canceling directions
    let mut triple_alterations = Vec::with_capacity(48);

    // Define the three independent directions: +Roll, +Pitch, +Yaw
    let directions = vec![
        (Vector3::x(), angle_offset),  // +Roll
        (Vector3::y(), angle_offset),  // +Pitch
        (Vector3::z(), angle_offset),  // +Yaw
    ];

    // Generate all permutations of (Roll, Pitch, Yaw) applications in distinct orders
    for &first in &directions {
        for &second in &directions {
            if first.0 == second.0 {
                // Skip if two rotations involve the same axis
                continue;
            }
            for &third in &directions {
                if third.0 == first.0 || third.0 == second.0 {
                    // Skip if third rotation involves the same axis as the first or second
                    continue;
                }

                // Apply rotations in order: first → second → third
                let first_pose = rotated_pose(pose, first.0, first.1);
                let second_pose = rotated_pose(&first_pose, second.0, second.1);
                let third_pose = rotated_pose(&second_pose, third.0, third.1);

                // Add the final altered pose
                triple_alterations.push(third_pose);
            }
        }
    }

    // Combine single, two-step, and three-step alterations into the final result
    let mut alterations = Vec::with_capacity(104 + 48 + 3);
    alterations.extend(single_alterations);
    alterations.extend(double_alterations);
    alterations.extend(triple_alterations);

    alterations
}

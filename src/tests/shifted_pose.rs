#[cfg(test)]
mod tests {
    use crate::kinematic_traits::Pose;
    use crate::kinematics_impl::shifted_pose;
    use glam::{DQuat, DVec3};
    use std::f64::consts::PI;

    #[test]
    fn shifted_pose_uses_pose_axes() {
        const EPSILON: f64 = 1.0e-12;

        let unit_shifts = [
            ("+X", [1.0, 0.0, 0.0]),
            ("-X", [-1.0, 0.0, 0.0]),
            ("+Y", [0.0, 1.0, 0.0]),
            ("-Y", [0.0, -1.0, 0.0]),
            ("+Z", [0.0, 0.0, 1.0]),
            ("-Z", [0.0, 0.0, -1.0]),
        ];

        let angle = PI / 6.0;
        let sin_30 = angle.sin();
        let cos_30 = angle.cos();
        let origin = DVec3::new(1.0, 2.0, 3.0);
        let rotate_x = DQuat::from_rotation_x(angle);
        let rotate_y = DQuat::from_rotation_y(angle);
        let rotate_z = DQuat::from_rotation_z(angle);
        let orientations = [
            (
                "axis-aligned",
                DQuat::IDENTITY,
                [
                    origin + DVec3::X,
                    origin - DVec3::X,
                    origin + DVec3::Y,
                    origin - DVec3::Y,
                    origin + DVec3::Z,
                    origin - DVec3::Z,
                ],
            ),
            (
                "30 degrees around X",
                rotate_x,
                [
                    origin + DVec3::X,
                    origin - DVec3::X,
                    origin + DVec3::new(0.0, cos_30, sin_30),
                    origin - DVec3::new(0.0, cos_30, sin_30),
                    origin + DVec3::new(0.0, -sin_30, cos_30),
                    origin - DVec3::new(0.0, -sin_30, cos_30),
                ],
            ),
            (
                "30 degrees around X and Y",
                rotate_y * rotate_x,
                [
                    origin + DVec3::new(cos_30, 0.0, -sin_30),
                    origin - DVec3::new(cos_30, 0.0, -sin_30),
                    origin + DVec3::new(sin_30 * sin_30, cos_30, sin_30 * cos_30),
                    origin - DVec3::new(sin_30 * sin_30, cos_30, sin_30 * cos_30),
                    origin + DVec3::new(sin_30 * cos_30, -sin_30, cos_30 * cos_30),
                    origin - DVec3::new(sin_30 * cos_30, -sin_30, cos_30 * cos_30),
                ],
            ),
            (
                "30 degrees around X, Y, and Z",
                rotate_z * rotate_y * rotate_x,
                [
                    origin + DVec3::new(cos_30 * cos_30, sin_30 * cos_30, -sin_30),
                    origin - DVec3::new(cos_30 * cos_30, sin_30 * cos_30, -sin_30),
                    origin
                        + DVec3::new(
                            cos_30 * sin_30 * sin_30 - sin_30 * cos_30,
                            sin_30 * sin_30 * sin_30 + cos_30 * cos_30,
                            cos_30 * sin_30,
                        ),
                    origin
                        - DVec3::new(
                            cos_30 * sin_30 * sin_30 - sin_30 * cos_30,
                            sin_30 * sin_30 * sin_30 + cos_30 * cos_30,
                            cos_30 * sin_30,
                        ),
                    origin
                        + DVec3::new(
                            sin_30 * cos_30 * cos_30 + sin_30 * sin_30,
                            sin_30 * sin_30 * cos_30 - sin_30 * cos_30,
                            cos_30 * cos_30,
                        ),
                    origin
                        - DVec3::new(
                            sin_30 * cos_30 * cos_30 + sin_30 * sin_30,
                            sin_30 * sin_30 * cos_30 - sin_30 * cos_30,
                            cos_30 * cos_30,
                        ),
                ],
            ),
        ];

        for (orientation_name, rotation, expected_positions) in orientations {
            let pose = Pose::from_parts(origin, rotation);

            for ((shift_name, shift), expected_position) in
                unit_shifts.into_iter().zip(expected_positions)
            {
                let shifted = shifted_pose(&pose, shift);

                assert!(
                    (shifted.translation - expected_position).length() <= EPSILON,
                    "{orientation_name}, {shift_name}: expected position {expected_position:?}, \
                     got {:?}",
                    shifted.translation
                );
                assert!(
                    shifted.angular_distance(pose) <= f64::EPSILON,
                    "{orientation_name}, {shift_name}: shift changed the orientation"
                );
            }
        }
    }
}

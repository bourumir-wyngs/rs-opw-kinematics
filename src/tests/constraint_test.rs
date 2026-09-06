#[cfg(test)]
mod tests {
    use crate::constraints::{BY_CONSTRAINS, Constraints};
    use crate::kinematic_traits::{Joints, Kinematics};
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;
    use crate::utils::as_radians;
    use std::f64::consts::PI;

    #[test]
    fn test_constraints() {
        // This robot has both A and B type singularity
        // B type singularity two angles, maestro
        let parameters = Parameters::staubli_tx2_160l();

        let not_below = as_radians([9, 18, 28, 38, -5, 55]);
        let joints = as_radians([10, 20, 30, 40, 0, 60]);
        let not_above = as_radians([11, 22, 33, 44, 5, 65]);

        let unconstrained_robot = OPWKinematics::new(parameters);
        let pose = unconstrained_robot.forward(&joints);

        let unconstrained = unconstrained_robot.inverse_continuing(&pose, &joints);
        assert!(joints_close(
            unconstrained.first().expect("expected inverse solutions"),
            &joints,
            1e-7,
        ));

        // Recovering wrist poles preserves branches previously lost to roundoff.
        // Require the expected branches without fixing their count or complete order.
        let regular_branches: [Joints; 2] = [
            [10.00, 51.75, -30.00, 0.00, 28.25, 100.00],
            [10.00, 51.75, -30.00, 180.00, -28.25, -80.00],
        ];
        for expected in regular_branches {
            let expected = expected.map(f64::to_radians);
            assert!(
                unconstrained.iter().any(|solution| joints_close(
                    solution,
                    &expected,
                    0.01_f64.to_radians()
                )),
                "missing regular branch {expected:?}: {unconstrained:?}"
            );
        }

        let constraints = Constraints::new(not_below, not_above, BY_CONSTRAINS);
        assert!(constraints.compliant(&joints));
        assert!(
            unconstrained
                .iter()
                .any(|solution| !constraints.compliant(solution)),
            "fixture must include a branch rejected by joint limits"
        );
        let constrained_robot = OPWKinematics::new_with_constraints(parameters, constraints);
        let constrained = constrained_robot.inverse_continuing(&pose, &joints);
        assert!(joints_close(
            constrained
                .first()
                .expect("expected a constraint-compliant inverse solution"),
            &joints,
            1e-7,
        ));
        assert!(
            constrained
                .iter()
                .all(|solution| constraints.compliant(solution))
        );

        for (robot, solutions) in [
            (&unconstrained_robot, &unconstrained),
            (&constrained_robot, &constrained),
        ] {
            for solution in solutions {
                assert!(solution.iter().all(|angle| angle.is_finite()));
                let resolved = robot.forward(solution);
                assert!((resolved.translation - pose.translation).length() < 1e-6);
                assert!(resolved.angular_distance(pose) < 1e-6);
            }
        }
    }

    fn joints_close(actual: &Joints, expected: &Joints, tolerance: f64) -> bool {
        actual.iter().zip(expected).all(|(actual, expected)| {
            ((actual - expected + PI).rem_euclid(2.0 * PI) - PI).abs() < tolerance
        })
    }

    #[test]
    fn test_no_limits_accept_all_angles() {
        // from == to for each joint means no limits (tolerance is infinity)
        let from = [0.0; 6];
        let to = [0.0; 6];
        let constraints = Constraints::new(from, to, BY_CONSTRAINS);

        let samples = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [
                std::f64::consts::PI,
                -std::f64::consts::PI,
                2.0 * std::f64::consts::PI,
                -2.0 * std::f64::consts::PI,
                1.5 * std::f64::consts::PI,
                -1.5 * std::f64::consts::PI,
            ],
        ];

        for angles in samples.iter() {
            assert!(constraints.compliant(angles));
        }
    }
}

use crate::kinematic_traits::Pose;
use crate::kinematics_impl::arm_continuum_2d::{
    Compensated, add, multiply, roots, search, squared_norm_roots,
};
use crate::kinematics_impl::{ArmBranch, OPWKinematics, wrapped_angle};
use std::f64::consts::PI;

#[test]
fn compensated_elimination_preserves_a_narrow_root_pair() {
    // These two critical slices bound a four-nanoradian feasible interval.
    // Ordinary f64 coefficient subtraction collapses them to one double root.
    let half_width = 2.0e-9;
    assert_eq!(0.25 - half_width * half_width, 0.25);
    let factor = vec![(-0.5).into(), 1.0.into()];
    let polynomial = add(
        &multiply(&factor, &factor),
        &[Compensated::from(half_width).times(half_width.into())],
        -1.0,
    );
    let found = roots(polynomial, -1.0, 1.0);
    assert_eq!(found.len(), 2, "{found:?}");
    assert!((found[0] - (0.5 - half_width)).abs() < 4.0 * f64::EPSILON);
    assert!((found[1] - (0.5 + half_width)).abs() < 4.0 * f64::EPSILON);
}

#[test]
fn transverse_norm_roots_distinguish_tiny_bends_from_poles() {
    let bend = 1.0e-9;
    let found = squared_norm_roots([0.0, 0.0, 1.0], [0.0; 3], bend);
    for angle in [-PI + bend, -bend, bend, PI - bend] {
        assert!(
            found
                .iter()
                .any(|root| wrapped_angle(root - angle).abs() < 2.0e-15),
            "missing {angle}: {found:?}"
        );
    }
    assert!(squared_norm_roots([0.0, 0.0, 1.0], [2.0 * bend, 0.0, 0.0], bend).is_empty());
}

#[test]
fn root_isolation_keeps_tangencies_and_all_degree_eight_crossings() {
    for expected in [
        vec![-0.8, -0.4, -0.1, 0.1, 0.4, 0.6, 0.7, 0.9],
        vec![-0.8, -0.3, 0.2, 0.2, 0.7, 0.7],
    ] {
        let mut polynomial = vec![1.0.into()];
        for &root in &expected {
            polynomial = multiply(&polynomial, &[Compensated::from(-root), 1.0.into()]);
        }
        let found = roots(polynomial, -1.0, 1.0);
        for root in expected {
            assert!(
                found.iter().any(|value| (value - root).abs() < 2.0e-13),
                "missing {root}: {found:?}"
            );
        }
        assert!(
            found
                .iter()
                .all(|root| root.is_finite() && (-1.0..=1.0).contains(root))
        );
    }
}

#[test]
fn simultaneous_free_angles_find_a_narrow_persistent_pole_phase() {
    use crate::constraints::{BY_PREV, Constraints};
    use crate::parameters::opw_kinematics::Parameters;
    use glam::{DQuat, DVec3};

    let mut parameters = Parameters::staubli_tx2_140();
    parameters.a1 = 0.0;
    parameters.offsets = [0.0; 6];
    let q1 = 40.0_f64.to_radians();
    let q4 = 40.0_f64.to_radians();
    let q6 = 60.0_f64.to_radians();
    let constraints = Constraints::new(
        [
            0.0,
            -PI - 1.0e-10,
            PI - 1.0e-6,
            q4 - 1.0e-9,
            -1.0e-10,
            q6 - 1.0e-9,
        ],
        [
            100.0_f64.to_radians(),
            -PI + 1.0e-10,
            PI + 1.0e-6,
            q4 + 1.0e-9,
            1.0e-10,
            q6 + 1.0e-9,
        ],
        BY_PREV,
    );
    let robot = OPWKinematics::new_with_constraints(parameters, constraints);
    let pose = Pose::from_parts(
        DVec3::new(0.0, 0.0, parameters.c1 + parameters.c4),
        DQuat::from_rotation_z(q1 + q4 + q6),
    );
    let reference = constraints.centers;
    assert!(
        robot
            .arm_wrist_candidates(
                &pose,
                ArmBranch {
                    q1: reference[0],
                    q2: -PI,
                    q3: PI
                },
                &reference,
                None
            )
            .is_empty()
    );
    let found = search(&robot, &pose, PI, &reference, None);
    assert!(!found.is_empty(), "lost the interior pole phase interval");
    assert!(
        found
            .iter()
            .all(|arm| wrapped_angle(arm.q1 - q1).abs() < 5.0e-9)
    );
}

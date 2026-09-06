//! Regression tests for arm angles that are free at an exact singularity.

use crate::constraints::{BY_PREV, Constraints};
use crate::glam::DVec3;
use crate::kinematic_traits::{CONSTRAINT_CENTERED, Joints, Kinematics, Pose, Solutions};
use crate::kinematics_impl::{ANGULAR_TOLERANCE, OPWKinematics};
use crate::parameters::opw_kinematics::Parameters;
use std::f64::consts::{PI, TAU};

#[test]
fn folded_zero_or_short_flange_keeps_compliant_arm_family() {
    for flange in [0.0, 0.0001] {
        let mut parameters = Parameters::staubli_tx2_140();
        parameters.a1 = 0.0;
        parameters.c4 = flange;
        parameters.offsets = [0.0; 6];
        let witness = [0.4, 0.2, PI, 0.3, 0.5, 0.6];
        let constraints = Constraints::new(
            [-1.0, -1.0, PI - 0.01, 0.29, 0.49, 0.59],
            [1.0, 1.0, PI + 0.01, 0.31, 0.51, 0.61],
            BY_PREV,
        );
        assert!(constraints.compliant(&witness));
        let robot = OPWKinematics::new_with_constraints(parameters, constraints);
        let target = robot.forward(&witness);
        for (api, solutions) in all_inverse(&robot, &target, &witness).iter().enumerate() {
            check_solutions(
                &robot,
                &target,
                &witness,
                1.0,
                api,
                solutions,
                &format!("reported folded pose, flange={flange}"),
            );
        }
    }
}

#[test]
fn singular_candidates_are_bounded_before_validation() {
    let (parameters, witness) = fixture(0, 1.0, false, 50.0);
    let robot = OPWKinematics::new(parameters);
    let pose = robot.forward(&witness);
    let mut previous = witness;
    previous[0] += TAU * 1_000_000.0;
    let candidates: Vec<_> = robot.inverse_candidates(&pose, &previous, None).collect();
    assert!(
        candidates
            .iter()
            .any(|joints| joints.iter().all(|q| q.is_finite()))
    );
    for candidate in candidates {
        // Wrist flips may contain one extra turn, but the magnitude must not
        // grow with previous turns and feed billions of normalization steps.
        assert!(
            candidate
                .iter()
                .all(|q| !q.is_finite() || q.abs() <= 2.0 * TAU)
        );
    }
}

fn angular_error(a: f64, b: f64) -> f64 {
    ((a - b + PI).rem_euclid(TAU) - PI).abs()
}

fn fixture(kind: usize, scale: f64, transformed: bool, q5: f64) -> (Parameters, Joints) {
    let mut p = if kind == 2 {
        Parameters::staubli_tx40()
    } else {
        Parameters::staubli_tx2_140()
    };
    if kind == 3 {
        // Effective distal length hypot(a2, c3) equals c2, even though c3 does not.
        p.a2 = 0.375;
        p.c3 = 0.5;
    } else if kind == 4 {
        // With the folded wrist center on the base axis, J1 and J2 are both free.
        p.a1 = 0.0;
    }
    p.a1 *= scale;
    p.a2 *= scale;
    p.b *= scale;
    p.c1 *= scale;
    p.c2 *= scale;
    p.c3 *= scale;
    p.c4 *= scale;
    if transformed {
        p.offsets = [20.0, -30.0, 15.0, 30.0, -10.0, -20.0].map(f64::to_radians);
        p.sign_corrections = [-1, -1, -1, -1, 1, -1];
    }
    let mut model = [40.0, 20.0, 0.0, 40.0, q5, 60.0].map(f64::to_radians);
    if kind == 0 {
        model[1] = 0.0;
        model[2] = (-p.a1 / p.c3).asin();
    } else {
        model[0] = if kind == 4 { 40.0 } else { 10.0_f64 }.to_radians();
        model[2] = PI - p.a2.atan2(p.c3);
    }
    let joints =
        std::array::from_fn(|i| (model[i] + p.offsets[i]) * f64::from(p.sign_corrections[i]));
    (p, joints)
}

fn all_inverse(robot: &OPWKinematics, target: &Pose, previous: &Joints) -> [Solutions; 4] {
    [
        robot.inverse(target),
        robot.inverse_continuing(target, previous),
        robot.inverse_5dof(target, previous[5]),
        robot.inverse_continuing_5dof(target, previous),
    ]
}

fn check_solutions(
    robot: &OPWKinematics,
    target: &Pose,
    previous: &Joints,
    scale: f64,
    api: usize,
    solutions: &Solutions,
    case: &str,
) {
    assert!(
        !solutions.is_empty(),
        "{case}, API {api}: lost reachable pose"
    );
    for joints in solutions {
        assert!(joints.iter().all(|q| q.is_finite()), "{case}, API {api}");
        if let Some(constraints) = robot.constraints() {
            assert!(constraints.compliant(joints), "{case}, API {api}: limits");
        }
        let actual = robot.forward(joints);
        assert!(
            actual.translation.is_finite() && actual.rotation.is_finite(),
            "{case}, API {api}: nonfinite forward pose"
        );
        assert!(
            (actual.translation - target.translation).length() <= robot.distance_tolerance,
            "{case}, API {api}, scale={scale}: position"
        );
        let orientation_error = if api < 2 {
            actual.angular_distance(*target)
        } else {
            // Five-axis inversion preserves direction, allowing different tool roll.
            assert!(angular_error(joints[5], previous[5]) < 1.0e-12);
            (actual.rotation * DVec3::Z - target.rotation * DVec3::Z).length()
        };
        assert!(
            orientation_error <= ANGULAR_TOLERANCE,
            "{case}, API {api}: orientation"
        );
    }
}

#[test]
fn exact_free_arm_poses_preserve_previous_joints() {
    for kind in 0..5 {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for transformed in [false, true] {
                for q5 in [0.0, 50.0, 180.0, -180.0] {
                    let (p, mut previous) = fixture(kind, scale, transformed, q5);
                    // Continuation must preserve the selected turns as well as the pose.
                    previous[0] += TAU;
                    previous[1] -= TAU;
                    let robot = OPWKinematics::new(p);
                    let target = robot.forward(&previous);
                    for (api, solutions) in [
                        (1, robot.inverse_continuing(&target, &previous)),
                        (3, robot.inverse_continuing_5dof(&target, &previous)),
                    ] {
                        let case = format!(
                            "kind={kind}, scale={scale}, transformed={transformed}, J5={q5}"
                        );
                        check_solutions(&robot, &target, &previous, scale, api, &solutions, &case);
                        for (i, q) in solutions[0].iter().enumerate() {
                            assert!(
                                (q - previous[i]).abs() < 1.0e-7,
                                "{case}, API {api}, J{} jumped: {} -> {}",
                                i + 1,
                                previous[i],
                                q
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn narrow_arm_limits_retain_each_free_joint_family() {
    for kind in 0..5 {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for transformed in [false, true] {
                for q5 in [0.0, 50.0, 180.0] {
                    let (p, joints) = fixture(kind, scale, transformed, q5);
                    let mut from = [0.0; 6];
                    let mut to = [0.0; 6];
                    for i in 0..3 {
                        from[i] = joints[i] - 1.0_f64.to_radians();
                        to[i] = joints[i] + 1.0_f64.to_radians();
                    }
                    let constraints = Constraints::new(from, to, BY_PREV);
                    assert!(constraints.compliant(&joints));
                    let robot = OPWKinematics::new_with_constraints(p, constraints);
                    let target = robot.forward(&joints);
                    let case =
                        format!("kind={kind}, scale={scale}, transformed={transformed}, J5={q5}");
                    for (api, solutions) in all_inverse(&robot, &target, &joints).iter().enumerate()
                    {
                        check_solutions(&robot, &target, &joints, scale, api, solutions, &case);
                    }
                }
            }
        }
    }
}

#[test]
fn wrist_limits_can_require_interior_free_arm_angles() {
    for kind in [0, 1, 4] {
        for transformed in [false, true] {
            for half_width in [1.0_f64.to_radians(), 5.0e-7, 5.0e-9] {
                let (p, mut witness) = fixture(kind, 1.0, transformed, 50.0);
                witness[0] = 40.1234567_f64.to_radians();
                if kind != 0 {
                    witness[1] = 20.7654321_f64.to_radians();
                }
                witness[3..].copy_from_slice(&[40.31415, 50.27182, 60.1618].map(f64::to_radians));
                let mut from = witness.map(|q| q - 1.0e-3);
                let mut to = witness.map(|q| q + 1.0e-3);
                let free = [kind == 0 || kind == 4, kind != 0];
                for i in 0..2 {
                    if free[i] {
                        from[i] = 0.0;
                        to[i] = if i == 0 { 100.0_f64 } else { 90.0_f64 }.to_radians();
                    }
                }
                for i in 3..6 {
                    from[i] = witness[i] - half_width;
                    to[i] = witness[i] + half_width;
                }
                let constraints = Constraints::new(from, to, BY_PREV);
                assert!(constraints.compliant(&witness));
                let previous = constraints.centers;
                let robot = OPWKinematics::new_with_constraints(p, constraints);
                let target = robot.forward(&witness);
                let case = format!(
                    "interior kind={kind}, transformed={transformed}, wrist half width={half_width}"
                );
                for (api, solutions) in all_inverse(&robot, &target, &previous).iter().enumerate() {
                    check_solutions(&robot, &target, &previous, 1.0, api, solutions, &case);
                    // At these narrow wrist limits, keeping either free angle at
                    // the previous/center value cannot produce a feasible solution.
                    if half_width < 1.0e-5 {
                        for i in 0..2 {
                            if free[i] {
                                assert!(angular_error(solutions[0][i], previous[i]) > 0.05);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn constraint_centered_continuation_resolves_the_fixed_j6_reference() {
    for kind in [0, 1, 4] {
        for transformed in [false, true] {
            for q5 in [0.0, 50.0, 180.0] {
                for dof in [5, 6] {
                    let (mut p, witness) = fixture(kind, 1.0, transformed, q5);
                    p.dof = dof;
                    let constraints = Constraints::new(
                        witness.map(|q| q - 0.01),
                        witness.map(|q| q + 0.01),
                        BY_PREV,
                    );
                    let reference = constraints.centers;
                    assert!(reference[5].abs() > 0.1);
                    let robot = OPWKinematics::new_with_constraints(p, constraints);
                    let target = robot.forward(&witness);
                    for (api, solutions) in [
                        (
                            if dof == 5 { 3 } else { 1 },
                            robot.inverse_continuing(&target, &CONSTRAINT_CENTERED),
                        ),
                        (
                            3,
                            robot.inverse_continuing_5dof(&target, &CONSTRAINT_CENTERED),
                        ),
                    ] {
                        let case = format!(
                            "constraint centered kind={kind}, dof={dof}, transformed={transformed}, J5={q5}"
                        );
                        check_solutions(&robot, &target, &reference, 1.0, api, &solutions, &case);
                    }
                }
            }
        }
    }
}

#[test]
fn free_arm_search_handles_wrapped_ranges() {
    for kind in [0, 1, 4] {
        let (p, mut witness) = fixture(kind, 1.0, false, 50.0);
        let free = [kind == 0 || kind == 4, kind != 0];
        for i in 0..2 {
            if free[i] {
                witness[i] = (177.1234567 - i as f64).to_radians();
            }
        }
        let mut from = witness.map(|q| q - 1.0e-5);
        let mut to = witness.map(|q| q + 1.0e-5);
        for i in 0..2 {
            if free[i] {
                from[i] = 170.0_f64.to_radians();
                to[i] = -160.0_f64.to_radians();
            }
        }
        let constraints = Constraints::new(from, to, BY_PREV);
        assert!(constraints.compliant(&witness));
        let previous = constraints.centers;
        let robot = OPWKinematics::new_with_constraints(p, constraints);
        let target = robot.forward(&witness);
        for (api, solutions) in all_inverse(&robot, &target, &previous).iter().enumerate() {
            check_solutions(
                &robot,
                &target,
                &previous,
                1.0,
                api,
                solutions,
                "wrapped limits",
            );
        }
    }
}

#[test]
fn free_j1_at_a_wrist_pole_respects_the_combined_wrist_phase_limits() {
    let mut p = Parameters::staubli_tx2_140();
    p.a1 = 0.0;
    p.c3 = 0.4;
    for q5 in [0.0, 180.0] {
        let witness = [45.1234567, 0.0, 0.0, 40.0, q5, 60.0].map(f64::to_radians);
        let mut from = witness.map(|q| q - 1.0e-5);
        let mut to = witness.map(|q| q + 1.0e-5);
        from[0] = 0.0;
        to[0] = 100.0_f64.to_radians();
        let constraints = Constraints::new(from, to, BY_PREV);
        let previous = constraints.centers;
        let robot = OPWKinematics::new_with_constraints(p, constraints);
        let target = robot.forward(&witness);
        // The entire free-J1 curve has J5 at a pole. The usual wrist atan2
        // equations vanish throughout the curve; only the coupled J4/J6 phase
        // limits identify its narrow feasible interval for six-axis inversion.
        for (api, solutions) in all_inverse(&robot, &target, &previous).iter().enumerate() {
            check_solutions(
                &robot,
                &target,
                &previous,
                1.0,
                api,
                solutions,
                "free J1 stays at wrist pole",
            );
        }
    }
}

#[test]
fn tiny_nonzero_wrist_bend_limits_remain_reachable_along_free_arm_families() {
    for kind in [1, 0, 4] {
        for near_pi in [false, true] {
            for limited_wrist_phase in [false, true] {
                let (p, mut witness) = fixture(kind, 1.0, false, 0.0);
                witness[4] = if near_pi { PI - 1.5e-9 } else { 1.5e-9 };
                let mut from = witness.map(|q| q - 1.0_f64.to_radians());
                let mut to = witness.map(|q| q + 1.0_f64.to_radians());
                if kind == 0 || kind == 4 {
                    from[0] = 0.0;
                    to[0] = 100.0_f64.to_radians();
                }
                if kind != 0 {
                    from[1] = 0.0;
                    to[1] = 90.0_f64.to_radians();
                }
                for i in [3, 5] {
                    if limited_wrist_phase {
                        from[i] = witness[i] - 0.1_f64.to_radians();
                        to[i] = witness[i] + 0.1_f64.to_radians();
                    } else {
                        from[i] = 0.0;
                        to[i] = 0.0;
                    }
                }
                (from[4], to[4]) = if near_pi {
                    (PI - 2.0e-9, PI - 1.0e-9)
                } else {
                    (1.0e-9, 2.0e-9)
                };
                let constraints = Constraints::new(from, to, BY_PREV);
                assert!(constraints.compliant(&witness));
                let previous = constraints.centers;
                let robot = OPWKinematics::new_with_constraints(p, constraints);
                let target = robot.forward(&witness);
                // This excludes the exact wrist pole, although every allowed
                // bend has a cosine that rounds to +/-1. Tiny tilt direction
                // still matters when J4/J6 are restricted around the witness.
                let case = format!(
                    "tiny bend kind={kind}, near pi={near_pi}, wrist phase limited={limited_wrist_phase}"
                );
                for (api, solutions) in all_inverse(&robot, &target, &previous).iter().enumerate() {
                    check_solutions(&robot, &target, &previous, 1.0, api, solutions, &case);
                    for joints in solutions {
                        assert!(
                            joints[4].sin().abs() > 0.9e-9,
                            "{case}, API {api}: erased the required nonzero wrist bend"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn free_arm_search_rejects_incompatible_wrist_limits() {
    for kind in [0, 1, 4] {
        let (p, witness) = fixture(kind, 1.0, false, 50.0);
        let mut from = witness.map(|q| q - 1.0_f64.to_radians());
        let mut to = witness.map(|q| q + 1.0_f64.to_radians());
        // The small allowed arm motion cannot rotate J4 by sixty degrees
        // while retaining the target tool direction and this J5 range.
        from[3] = 100.0_f64.to_radians();
        to[3] = 110.0_f64.to_radians();
        let robot = OPWKinematics::new_with_constraints(p, Constraints::new(from, to, BY_PREV));
        let target = robot.forward(&witness);
        for (api, solutions) in all_inverse(&robot, &target, &witness).iter().enumerate() {
            assert!(
                solutions.is_empty(),
                "kind={kind}, API {api}: incompatible wrist limits"
            );
        }
    }
}

#[test]
fn nearby_axis_and_folded_poses_keep_their_determined_arm_angles() {
    for kind in [0, 1] {
        for scale in [1.0e-3, 1.0, 1.0e3] {
            for delta in [-1.0e-7, 1.0e-7] {
                let (p, mut witness) = fixture(kind, scale, false, 50.0);
                witness[2] += delta;
                let mut previous = witness;
                let joint = if kind == 0 { 0 } else { 1 };
                previous[joint] += 0.2;
                let robot = OPWKinematics::new(p);
                let target = robot.forward(&witness);
                for (api, solutions) in [
                    (1, robot.inverse_continuing(&target, &previous)),
                    (3, robot.inverse_continuing_5dof(&target, &previous)),
                ] {
                    let case = format!("nearby kind={kind}, delta={delta}, scale={scale}");
                    check_solutions(&robot, &target, &previous, scale, api, &solutions, &case);
                    assert!(
                        angular_error(solutions[0][joint], witness[joint]) < 1.0e-5,
                        "{case}, API {api}: replaced determined angle with a free choice"
                    );
                }
            }
        }
    }
}

fn short_flange_fixture(
    scale: f64,
    flange: f64,
    geometry: usize,
    transformed: bool,
) -> (Parameters, Joints, Constraints) {
    let mut parameters = Parameters::staubli_tx2_140();
    parameters.a1 = 0.0;
    parameters.c4 = flange;
    parameters.offsets = [0.0; 6];
    if geometry == 1 || geometry == 3 {
        // The distal effective length hypot(0.375, 0.5) is still 0.625.
        parameters.a2 = 0.375;
        parameters.c3 = 0.5;
    }
    if geometry >= 2 {
        // With no base height, vertical cancellation has no c1 error floor.
        parameters.c1 = 0.0;
    }
    parameters.a2 *= scale;
    parameters.c1 *= scale;
    parameters.c2 *= scale;
    parameters.c3 *= scale;
    parameters.c4 *= scale;
    if transformed {
        parameters.offsets = [0.2, -0.3, 0.15, 0.3, -0.1, -0.2];
        parameters.sign_corrections = [-1, -1, -1, -1, 1, -1];
    }
    let folded_q3 = PI - parameters.a2.atan2(parameters.c3);
    let to_user = |model: Joints| {
        std::array::from_fn(|i| {
            (model[i] + parameters.offsets[i]) * f64::from(parameters.sign_corrections[i])
        })
    };
    // Also exercise vertical roundoff when no base-height error floor exists.
    let q2 = if geometry >= 2 { 0.3 } else { 0.2 };
    let witness = to_user([0.4, q2, folded_q3, 0.3, 0.5, 0.6]);
    let lower = to_user([-1.0, -1.0, folded_q3 - 0.01, 0.29, 0.49, 0.59]);
    let upper = to_user([1.0, 1.0, folded_q3 + 0.01, 0.31, 0.51, 0.61]);
    let constraints = Constraints::new(
        std::array::from_fn(|i| lower[i].min(upper[i])),
        std::array::from_fn(|i| lower[i].max(upper[i])),
        BY_PREV,
    );
    (parameters, witness, constraints)
}

#[test]
fn folded_short_flange_recovery_scales_and_preserves_exact_previous() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for flange in [0.0, 0.0001] {
            for geometry in 0..4 {
                for transformed in [false, true] {
                    let (parameters, witness, constraints) =
                        short_flange_fixture(scale, flange, geometry, transformed);
                    assert!(constraints.compliant(&witness));
                    let robot = OPWKinematics::new(parameters);
                    let target = robot.forward(&witness);
                    if geometry == 2 && scale == 1.0 && flange == 0.0 && !transformed {
                        assert_ne!(target.translation.z, 0.0);
                    }
                    let case = format!(
                        "short flange={flange}, scale={scale}, geometry={geometry}, transformed={transformed}"
                    );
                    for (api, solutions) in [
                        (1, robot.inverse_continuing(&target, &witness)),
                        (3, robot.inverse_continuing_5dof(&target, &witness)),
                    ] {
                        check_solutions(&robot, &target, &witness, scale, api, &solutions, &case);
                        for (joint, (&actual, &expected)) in
                            solutions[0].iter().zip(witness.iter()).enumerate()
                        {
                            assert!(
                                (actual - expected).abs() < 1.0e-7,
                                "{case}, API {api}: J{} jumped from {expected} to {actual}",
                                joint + 1,
                            );
                        }
                    }

                    let robot = OPWKinematics::new_with_constraints(parameters, constraints);
                    // Neither free-arm center equals the witness. All four
                    // APIs must find the interior solution permitted by the
                    // wrist limits, even with zero/short flange translation.
                    let previous = constraints.centers;
                    for (api, solutions) in
                        all_inverse(&robot, &target, &previous).iter().enumerate()
                    {
                        check_solutions(&robot, &target, &previous, scale, api, solutions, &case);
                    }
                }
            }
        }
    }
}

#[test]
fn nearby_short_flange_fold_keeps_both_determined_arm_angles() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for flange in [0.0, 0.0001] {
            for delta in [-1.0e-7, 1.0e-7] {
                let (parameters, mut witness, _) = short_flange_fixture(scale, flange, 2, false);
                witness[2] += delta;
                let mut previous = witness;
                previous[0] += 0.2;
                previous[1] += 0.2;
                let robot = OPWKinematics::new(parameters);
                let target = robot.forward(&witness);
                let case = format!("nearby short flange={flange}, scale={scale}, delta={delta}");
                for (api, solutions) in [
                    (1, robot.inverse_continuing(&target, &previous)),
                    (3, robot.inverse_continuing_5dof(&target, &previous)),
                ] {
                    check_solutions(&robot, &target, &previous, scale, api, &solutions, &case);
                    // This resolvable bend is much larger than link roundoff.
                    // A broad pose tolerance must not make J1 or J2 free and
                    // let either one remain at the deliberately wrong reference.
                    for joint in [0, 1] {
                        assert!(
                            angular_error(solutions[0][joint], witness[joint]) < 2.0e-6,
                            "{case}, API {api}: replaced determined J{} with a free choice: {:?}",
                            joint + 1,
                            solutions[0],
                        );
                    }
                }
            }
        }
    }
}

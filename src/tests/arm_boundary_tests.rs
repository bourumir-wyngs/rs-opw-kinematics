//! Regression tests for numerical domains at the arm workspace boundaries.

use super::scale_geometry;
use crate::constraints::{BY_PREV, Constraints};
use crate::kinematic_traits::{Joints, Kinematics, Pose};
use crate::kinematics_impl::{ANGULAR_TOLERANCE, OPWKinematics};
use crate::parameters::opw_kinematics::Parameters;
use glam::DVec3;
use std::f64::consts::PI;

fn isolate_shoulder(p: Parameters, joints: &Joints) -> OPWKinematics {
    // Otherwise a reachable alternate shoulder can hide the missing branch,
    // or reach a target inside this shoulder's inner workspace hole.
    let mut from = [0.0; 6];
    let mut to = [0.0; 6];
    from[0] = joints[0] - 1.0_f64.to_radians();
    to[0] = joints[0] + 1.0_f64.to_radians();
    OPWKinematics::new_with_constraints(p, Constraints::new(from, to, BY_PREV))
}

fn assert_inverse_result(
    robot: &OPWKinematics,
    target: &Pose,
    reference: &Joints,
    scale: f64,
    reachable: bool,
    case: &str,
) {
    for api in 0..4 {
        let (name, solutions) = match api {
            0 => ("inverse", robot.inverse(target)),
            1 => (
                "inverse_continuing",
                robot.inverse_continuing(target, reference),
            ),
            2 => ("inverse_5dof", robot.inverse_5dof(target, reference[5])),
            _ => (
                "inverse_continuing_5dof",
                robot.inverse_continuing_5dof(target, reference),
            ),
        };
        assert_eq!(
            !solutions.is_empty(),
            reachable,
            "{case}, {name}, scale={scale}: unexpected reachability"
        );
        for joints in solutions {
            assert!(
                joints.iter().all(|angle| angle.is_finite()),
                "{case}, {name}"
            );
            if let Some(constraints) = robot.constraints() {
                assert!(
                    constraints.compliant(&joints),
                    "{case}, {name}: joint limits"
                );
            }
            if api >= 2 {
                assert!(
                    (joints[5] - reference[5]).abs() < 1.0e-12,
                    "{case}, {name}: fixed J6 changed"
                );
            }
            let actual = robot.forward(&joints);
            assert!(
                actual.translation.is_finite() && actual.rotation.is_finite(),
                "{case}, {name}: nonfinite forward pose"
            );
            let position_error = (actual.translation - target.translation).length();
            assert!(
                position_error <= robot.distance_tolerance,
                "{case}, {name}: position error {position_error}"
            );
            // Five-axis IK preserves the tool direction but permits tool roll.
            let orientation_error = if api < 2 {
                actual.angular_distance(*target)
            } else {
                (actual.rotation * DVec3::Z - target.rotation * DVec3::Z).length()
            };
            assert!(
                orientation_error <= ANGULAR_TOLERANCE,
                "{case}, {name}: orientation error {orientation_error}"
            );
        }
    }
}

#[test]
fn supplied_irb2400_outer_boundary_remains_reachable() {
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    let joints = [0.0, 0.0, -79.86219614691485, 40.0, 50.0, 60.0].map(f64::to_radians);
    assert_inverse_result(
        &robot,
        &robot.forward(&joints),
        &joints,
        1.0,
        true,
        "reported IRB2400 outer boundary",
    );
}

#[test]
fn irb2400_inner_and_outer_boundaries_preserve_reachability() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        let p = scale_geometry(Parameters::irb2400_10(), scale);
        for inner in [false, true] {
            for q2_degrees in [-20.0, 20.0] {
                for q5_degrees in [0.0, 50.0, 180.0, -180.0] {
                    let mut joints =
                        [10.0, q2_degrees, 0.0, 40.0, q5_degrees, 60.0].map(f64::to_radians);
                    // The effective distal link is parallel or antiparallel
                    // to c2 at the outer or inner reach, respectively.
                    joints[2] = if inner { PI } else { 0.0 } - p.a2.atan2(p.c3) + p.offsets[2];
                    let robot = isolate_shoulder(p, &joints);
                    let pose = robot.forward(&joints);
                    let link_direction = DVec3::new(
                        joints[1].sin() * joints[0].cos(),
                        joints[1].sin() * joints[0].sin(),
                        joints[1].cos(),
                    );
                    // IRB2400's distal link is longer than c2, so its folded
                    // wrist center points opposite the first arm link.
                    let radial_direction = if inner {
                        -link_direction
                    } else {
                        link_direction
                    };
                    for delta in [-1.0e-5, -1.0e-8, 0.0, 1.0e-8, 1.0e-5] {
                        let mut target = pose;
                        target.translation += radial_direction * (delta * scale);
                        let reachable = if inner { delta >= 0.0 } else { delta <= 0.0 };
                        let case = format!(
                            "IRB2400 inner={inner}, J2={q2_degrees}, J5={q5_degrees}, delta={delta}"
                        );
                        assert_inverse_result(&robot, &target, &joints, scale, reachable, &case);
                    }
                }
            }
        }
    }
}

#[test]
fn tx40_shoulder_cylinder_preserves_reachability() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        let robot = OPWKinematics::new(scale_geometry(Parameters::staubli_tx40(), scale));
        for q1_degrees in [0.0, 90.0, 180.0] {
            for q5_degrees in [0.0, 50.0, 180.0, -180.0] {
                // Equal link radial contributions cancel at this pose. The
                // wrist center is exactly |b| away from the base axis.
                let joints =
                    [q1_degrees, -30.0, -30.0, 40.0, q5_degrees, 60.0].map(f64::to_radians);
                let pose = robot.forward(&joints);
                let radial_direction = DVec3::new(-joints[0].sin(), joints[0].cos(), 0.0);
                for delta in [-1.0e-5, -1.0e-8, 0.0, 1.0e-8, 1.0e-5] {
                    let mut target = pose;
                    target.translation += radial_direction * (delta * scale);
                    let case = format!("TX40 J1={q1_degrees}, J5={q5_degrees}, delta={delta}");
                    assert_inverse_result(&robot, &target, &joints, scale, delta >= 0.0, &case);
                }
            }
        }
    }
}

#[test]
fn unequal_links_with_small_inner_radius_remain_reachable() {
    // These radii are nonzero: this exercises cancellation in the cosine
    // numerator, rather than the separate equal-link folded continuum.
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for inner_radius in [1.0e-4, 1.0e-6, 1.0e-8] {
            let mut p = Parameters::staubli_tx2_140();
            p.c3 = p.c2 - inner_radius;
            let p = scale_geometry(p, scale);
            for q2_degrees in [2.0, 45.0, 135.0, 178.0] {
                for q5_degrees in [0.0, 50.0, 180.0] {
                    let joints =
                        [10.0, q2_degrees, 180.0, 40.0, q5_degrees, 60.0].map(f64::to_radians);
                    let robot = isolate_shoulder(p, &joints);
                    let case =
                        format!("inner radius={inner_radius}, J2={q2_degrees}, J5={q5_degrees}");
                    assert_inverse_result(
                        &robot,
                        &robot.forward(&joints),
                        &joints,
                        scale,
                        true,
                        &case,
                    );
                }
            }
        }
    }
}

#[test]
fn nearly_equal_links_do_not_make_j2_free() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for inner_radius in [1.0e-6, 1.0e-8] {
            let mut p = Parameters::staubli_tx2_140();
            p.c3 = p.c2 - inner_radius;
            let p = scale_geometry(p, scale);
            for q2_degrees in [45.0, 135.0] {
                let joints = [10.0, q2_degrees, 180.0, 40.0, 50.0, 60.0].map(f64::to_radians);
                let robot = isolate_shoulder(p, &joints);
                let target = robot.forward(&joints);
                let mut previous = joints;
                previous[1] += 0.2;
                let case = format!("determined J2, inner radius={inner_radius}, J2={q2_degrees}");
                assert_inverse_result(&robot, &target, &previous, scale, true, &case);

                // At this tiny inner radius a different J2 can still pass FK
                // validation. It is nevertheless not a free joint: constraints
                // around the displaced reference must reject the target.
                let mut from = [0.0; 6];
                let mut to = [0.0; 6];
                from[0] = joints[0] - 0.01;
                to[0] = joints[0] + 0.01;
                from[1] = previous[1] - 0.01;
                to[1] = previous[1] + 0.01;
                let constrained =
                    OPWKinematics::new_with_constraints(p, Constraints::new(from, to, BY_PREV));
                assert_inverse_result(&constrained, &target, &previous, scale, false, &case);
            }
        }
    }
}

#[test]
fn combined_shoulder_and_elbow_boundary_with_both_offsets_is_reachable() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for a1 in [-0.1, 0.1] {
            let mut p = Parameters::staubli_tx40();
            p.a1 = a1;
            p.c2 = 0.6;
            p.c3 = 0.3;
            let p = scale_geometry(p, scale);
            for inner in [false, true] {
                let radius = if inner { p.c2 - p.c3 } else { p.c2 + p.c3 };
                for q1_degrees in [0.0, 37.0, 90.0] {
                    for q5_degrees in [0.0, 50.0, 180.0] {
                        let mut joints =
                            [q1_degrees, 0.0, 0.0, 40.0, q5_degrees, 60.0].map(f64::to_radians);
                        // Extend or fold the links while cancelling a1 in
                        // their radial component. Both the shoulder sqrt and
                        // elbow acos lie on their domain boundaries.
                        joints[1] = (-p.a1 / radius).asin();
                        joints[2] = if inner { PI } else { 0.0 } + p.offsets[2];
                        let robot = isolate_shoulder(p, &joints);
                        let pose = robot.forward(&joints);
                        let radial_direction = DVec3::new(-joints[0].sin(), joints[0].cos(), 0.0);
                        for delta in [-1.0e-8, 0.0, 1.0e-8] {
                            let mut target = pose;
                            target.translation += radial_direction * (delta * scale);
                            let case = format!(
                                "combined a1={a1}, inner={inner}, J1={q1_degrees}, J5={q5_degrees}, delta={delta}"
                            );
                            assert_inverse_result(
                                &robot,
                                &target,
                                &joints,
                                scale,
                                delta >= 0.0,
                                &case,
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn shoulder_root_error_reaches_the_constrained_elbow_branch() {
    let mut p = Parameters::staubli_tx40();
    p.a1 = -0.1;
    p.c2 = 0.6;
    p.c3 = 0.3;
    let mut joints = [37.0, 0.0, -90.0, 40.0, 50.0, 60.0].map(f64::to_radians);
    joints[1] = (-p.a1 / (p.c2 + p.c3)).asin();
    let mut from = [0.0; 6];
    let mut to = [0.0; 6];
    from[2] = joints[2] - 1.0e-7;
    to[2] = joints[2] + 1.0e-7;
    let robot = OPWKinematics::new_with_constraints(p, Constraints::new(from, to, BY_PREV));

    // The alternate shoulder slightly bends its elbow to absorb sqrt error.
    // Restrict J3 so that it cannot hide the fully extended branch: recovering
    // that branch requires propagating shoulder sqrt uncertainty into acos.
    assert_inverse_result(
        &robot,
        &robot.forward(&joints),
        &joints,
        1.0,
        true,
        "constrained elbow at the shoulder cylinder",
    );
}

#[test]
fn elbow_boundaries_with_joint_offsets_and_reversed_axes() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for proximal_longer in [false, true] {
            let mut p = Parameters::irb2400_10();
            let distal_length = p.a2.hypot(p.c3);
            // Exercise both signs of the folded link direction. Neither case
            // has equal links, so J2 remains determined at the inner boundary.
            p.c2 = distal_length * if proximal_longer { 1.25 } else { 0.75 };
            p.offsets = [20.0, -30.0, 15.0, 30.0, -10.0, -20.0].map(f64::to_radians);
            p.sign_corrections = [-1, 1, -1, -1, -1, 1];
            let p = scale_geometry(p, scale);
            for inner in [false, true] {
                for q2_degrees in [-25.0, 25.0] {
                    for q5_degrees in [0.0, 50.0, 180.0, -180.0] {
                        let mut model =
                            [37.0, q2_degrees, 0.0, 40.0, q5_degrees, 60.0].map(f64::to_radians);
                        model[2] = if inner { PI } else { 0.0 } - p.a2.atan2(p.c3);
                        let joints = std::array::from_fn(|i| {
                            (model[i] + p.offsets[i]) * f64::from(p.sign_corrections[i])
                        });
                        let robot = isolate_shoulder(p, &joints);
                        let pose = robot.forward(&joints);
                        let link_direction = DVec3::new(
                            model[1].sin() * model[0].cos(),
                            model[1].sin() * model[0].sin(),
                            model[1].cos(),
                        );
                        let outward = if inner && !proximal_longer {
                            -link_direction
                        } else {
                            link_direction
                        };
                        for delta in [-1.0e-8, 0.0, 1.0e-8] {
                            let mut target = pose;
                            target.translation += outward * (delta * scale);
                            let reachable = if inner { delta >= 0.0 } else { delta <= 0.0 };
                            let case = format!(
                                "transformed elbow, proximal_longer={proximal_longer}, inner={inner}, J2={q2_degrees}, J5={q5_degrees}, delta={delta}"
                            );
                            assert_inverse_result(
                                &robot, &target, &joints, scale, reachable, &case,
                            );
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn shoulder_boundary_with_signed_lateral_offset_and_reversed_axes() {
    for scale in [1.0e-3, 1.0, 1.0e3] {
        for b_sign in [-1.0, 1.0] {
            let mut p = Parameters::staubli_tx40();
            p.b *= b_sign;
            p.offsets = [20.0, -30.0, 15.0, 30.0, -10.0, -20.0].map(f64::to_radians);
            p.sign_corrections = [-1, -1, 1, -1, -1, 1];
            let p = scale_geometry(p, scale);
            for q1_degrees in [-143.0, 37.0] {
                for q5_degrees in [0.0, 50.0, 180.0, -180.0] {
                    // In model coordinates the equal links have opposite radial
                    // components, leaving only the signed lateral offset b.
                    let model =
                        [q1_degrees, -30.0, 60.0, 40.0, q5_degrees, 60.0].map(f64::to_radians);
                    let joints = std::array::from_fn(|i| {
                        (model[i] + p.offsets[i]) * f64::from(p.sign_corrections[i])
                    });
                    let robot = isolate_shoulder(p, &joints);
                    let pose = robot.forward(&joints);
                    let outward = DVec3::new(-model[0].sin(), model[0].cos(), 0.0) * b_sign;
                    for delta in [-1.0e-8, 0.0, 1.0e-8] {
                        let mut target = pose;
                        target.translation += outward * (delta * scale);
                        let case = format!(
                            "transformed shoulder, b_sign={b_sign}, J1={q1_degrees}, J5={q5_degrees}, delta={delta}"
                        );
                        assert_inverse_result(&robot, &target, &joints, scale, delta >= 0.0, &case);
                    }
                }
            }
        }
    }
}

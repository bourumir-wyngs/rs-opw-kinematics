use crate::constraints::{BY_PREV, Constraints};
use crate::ik_result::{
    Branch, Elbow, GeometryStatus, Joint, JointLimitStatus, Recovery, Shoulder, Singularity, Wrist,
};
use crate::kinematic_traits::{J1, J2, J3, J4, J5, J6, Joints, Kinematics, Pose};
use crate::kinematics_impl::OPWKinematics;
use crate::parallelogram::Parallelogram;
use crate::parameters::opw_kinematics::Parameters;
use crate::tool::{Base, Tool};
use glam::{DQuat, DVec3};
use std::f64::consts::{PI, TAU};
use std::sync::Arc;

const SEED: Joints = [0.35, 0.4, 0.8, 0.2, 0.6, -0.7];

fn equal_links() -> Parameters {
    Parameters {
        c2: 1.0,
        c3: 1.0,
        ..Parameters::new()
    }
}

fn assert_pose(actual: Pose, target: Pose) {
    assert!(actual.translation.distance(target.translation) < 1e-8);
    assert!(actual.angular_distance(target) < 1e-7);
}

#[test]
fn each_slot_matches_the_physical_branch_for_both_references() {
    let parameters = Parameters {
        offsets: [0.1, 0.0, 0.25, 0.0, -0.3, 0.0],
        sign_corrections: [-1, 1, -1, 1, -1, 1],
        ..equal_links()
    };
    let robot = OPWKinematics::new(parameters);
    let user_seed: Joints = std::array::from_fn(|i| {
        (SEED[i] + parameters.offsets[i]) * f64::from(parameters.sign_corrections[i])
    });
    let target = robot.forward(&user_seed);
    let near_last_branch = robot.ik(&target, None)[7].joints.map(|angle| angle + TAU);
    for reference in [None, Some(&near_last_branch)] {
        for (index, slot) in robot.ik(&target, reference).iter().enumerate() {
            let meta = slot.meta;
            let branch = meta.branch;
            assert_eq!(branch.index(), index);
            assert_eq!(meta.geometry, GeometryStatus::Verified);
            assert_pose(robot.forward(&slot.joints), target);
            // On this equal-link arm, the shoulder choices face in opposite directions,
            // and the elbow and wrist choices differ by the sign of the model bend.
            let model: Joints = std::array::from_fn(|i| {
                slot.joints[i] * f64::from(parameters.sign_corrections[i]) - parameters.offsets[i]
            });
            assert_eq!(
                (model[J1] - SEED[J1]).cos() > 0.0,
                branch.shoulder() == Shoulder::First
            );
            assert_eq!(model[J3].sin() > 0.0, branch.elbow() == Elbow::Positive);
            assert_eq!(model[J5].sin() > 0.0, branch.wrist() == Wrist::Principal);
            assert_eq!(meta.family.iter().collect::<Vec<_>>(), [branch]);
            assert!(meta.singularities.is_empty() && meta.recovery.is_empty());
            assert!(meta.limits.checked().is_empty());
            assert_eq!(meta.limiting_joint, None);
            assert_eq!(slot.limit_margin_rad, f64::INFINITY);
        }
    }
    let near = robot.ik(&target, Some(&near_last_branch))[7].joints;
    assert!(
        near.iter()
            .zip(near_last_branch)
            .all(|(a, b)| (a - b).abs() < 1e-8)
    );
}

#[test]
fn unreachable_shoulders_leave_holes_between_verified_slots() {
    let robot = OPWKinematics::new(Parameters {
        a1: 1.0,
        ..equal_links()
    });
    let target = robot.forward(&SEED);
    for (index, slot) in robot.ik(&target, None).iter().enumerate() {
        assert_eq!(slot.meta.branch.index(), index);
        if slot.meta.branch.shoulder() == Shoulder::First {
            assert_eq!(slot.meta.geometry, GeometryStatus::Verified);
            assert_pose(robot.forward(&slot.joints), target);
        } else {
            assert_eq!(slot.meta.geometry, GeometryStatus::NoRealRoot);
            assert!(slot.joints.iter().all(|angle| angle.is_nan()));
            assert!(slot.limit_margin_rad.is_nan());
            assert!(slot.meta.limits.checked().is_empty());
        }
    }
}

#[test]
fn checked_limits_retain_all_solutions_through_composed_wrappers() {
    let mut lower = [0.0; 6];
    let mut upper = [0.0; 6];
    lower[J3] = 0.5;
    upper[J3] = 1.25;
    let bare = Arc::new(OPWKinematics::new_with_constraints(
        equal_links(),
        Constraints::new(lower, upper, BY_PREV),
    ));
    let robot = Tool {
        robot: Arc::new(Base {
            robot: Arc::new(Parallelogram {
                robot: bare.clone(),
                driven: J2,
                coupled: J3,
                scaling: 1.0,
            }),
            base: Pose::from_parts(DVec3::new(0.3, -0.2, 0.1), DQuat::from_rotation_z(1.1)),
        }),
        tool: Pose::from_parts(DVec3::new(0.2, 0.1, 0.3), DQuat::from_rotation_x(0.6)),
    };
    let mut coupled = SEED;
    coupled[J3] += coupled[J2];
    let target = robot.forward(&coupled);
    let inner = bare.ik(&bare.forward(&SEED), Some(&SEED));
    for (slot, underlying) in robot.ik(&target, Some(&coupled)).iter().zip(inner) {
        assert_eq!(slot.meta.branch, underlying.meta.branch);
        assert_eq!(slot.meta.geometry, GeometryStatus::Verified);
        assert_pose(robot.forward(&slot.joints), target);
        let positive = slot.meta.branch.elbow() == Elbow::Positive;
        let expected = if positive {
            JointLimitStatus::WithinLimits
        } else {
            JointLimitStatus::Violated
        };
        assert_eq!(slot.meta.limits.status(Joint::J3), expected);
        assert_eq!(slot.meta.limiting_joint, Some(Joint::J3));
        let margin = if positive { 0.3 } else { -1.3 };
        assert!((slot.limit_margin_rad - margin).abs() < 1e-8);
        assert!((slot.joints[J3] - slot.joints[J2] - underlying.joints[J3]).abs() < 1e-8);
    }
}

#[test]
fn wrist_poles_share_a_family_and_record_phase_recovery() {
    let robot = OPWKinematics::new(equal_links());
    let principal = Branch::new(Shoulder::First, Elbow::Positive, Wrist::Principal);
    let flipped = Branch::new(Shoulder::First, Elbow::Positive, Wrist::Flipped);
    for (angle, singularity) in [(0.0, Singularity::WristZero), (PI, Singularity::WristPi)] {
        let mut seed = SEED;
        seed[J5] = angle;
        let target = robot.forward(&seed);
        let mut previous = seed;
        previous[J4] += 0.4;
        previous[J6] += if angle == 0.0 { -0.4 } else { 0.4 };
        let result = robot.ik(&target, Some(&previous));
        for branch in [principal, flipped] {
            let slot = &result[branch.index()];
            assert_eq!(slot.meta.branch, branch);
            assert_eq!(slot.meta.geometry, GeometryStatus::Verified);
            assert!(slot.meta.singularities.contains(singularity));
            assert!(slot.meta.recovery.contains(Recovery::WristPhase));
            assert_eq!(
                slot.meta.family.iter().collect::<Vec<_>>(),
                [principal, flipped]
            );
            assert_pose(robot.forward(&slot.joints), target);
        }
        let kept = &result[principal.index()].joints;
        assert!((kept[J4] - previous[J4]).abs() < 1e-7);
        assert!((kept[J6] - previous[J6]).abs() < 1e-7);
    }
}

#[test]
fn a_fully_folded_arm_keeps_its_free_joint_families() {
    let robot = OPWKinematics::new(equal_links());
    let mut reference = SEED;
    reference[J3] = PI;
    // The equal links cancel exactly. Specify the coincident wrist and base
    // center directly so FK rounding cannot obscure the two free joints.
    let target = Pose::from_parts(DVec3::ZERO, robot.forward(&reference).rotation);
    for slot in robot.ik(&target, Some(&reference)) {
        let meta = slot.meta;
        assert_eq!(meta.geometry, GeometryStatus::Verified);
        assert_pose(robot.forward(&slot.joints), target);
        for singularity in [
            Singularity::FreeJ1,
            Singularity::FreeJ2,
            Singularity::FoldedElbow,
        ] {
            assert!(meta.singularities.contains(singularity));
        }
        assert!(meta.recovery.contains(Recovery::FreeJ1Search));
        assert!(meta.recovery.contains(Recovery::FreeJ2Search));
        for shoulder in [Shoulder::First, Shoulder::Second] {
            for elbow in [Elbow::Positive, Elbow::Negative] {
                assert!(
                    meta.family
                        .contains(Branch::new(shoulder, elbow, meta.branch.wrist()))
                );
            }
        }
    }
}

#[test]
fn malformed_inputs_and_five_axis_models_return_labelled_empty_slots() {
    let robot = OPWKinematics::new(equal_links());
    let five_axis = OPWKinematics::new(Parameters {
        dof: 5,
        ..equal_links()
    });
    let invalid = Pose {
        rotation: DQuat::NAN,
        translation: DVec3::ZERO,
    };
    for (results, reason) in [
        (robot.ik(&invalid, None), GeometryStatus::InvalidInput),
        (
            robot.ik(&Pose::IDENTITY, Some(&[f64::NAN; 6])),
            GeometryStatus::InvalidInput,
        ),
        (
            five_axis.ik(&Pose::IDENTITY, None),
            GeometryStatus::Unsupported,
        ),
    ] {
        for (index, slot) in results.iter().enumerate() {
            assert_eq!(slot.meta.branch.index(), index);
            assert_eq!(slot.meta.geometry, reason);
            assert!(slot.joints.iter().all(|q| q.is_nan()));
            assert!(slot.meta.family.is_empty());
        }
    }
}

#[test]
fn arm_branch_coalescence_groups_only_the_coincident_roots() {
    let mut extended = SEED;
    extended[J1] = 0.0;
    extended[J2] = PI / 2.0;
    extended[J3] = 0.0;
    let mut on_cylinder = SEED;
    on_cylinder[J1] = 0.0;
    on_cylinder[J2] = -on_cylinder[J3] / 2.0;
    for (parameters, seed, singularity) in [
        (equal_links(), extended, Singularity::ExtendedElbow),
        (
            Parameters {
                b: 0.25,
                ..equal_links()
            },
            on_cylinder,
            Singularity::ShoulderCylinderBoundary,
        ),
    ] {
        let robot = OPWKinematics::new(parameters);
        let target = robot.forward(&seed);
        for slot in robot.ik(&target, Some(&seed)) {
            let meta = slot.meta;
            assert_eq!(meta.geometry, GeometryStatus::Verified);
            assert_pose(robot.forward(&slot.joints), target);
            assert!(meta.singularities.contains(singularity));
            assert_eq!(meta.family.iter().count(), 2);
            let branch = meta.branch;
            for other in meta.family.iter() {
                assert_eq!(other.wrist(), branch.wrist());
                if singularity == Singularity::ExtendedElbow {
                    assert_eq!(other.shoulder(), branch.shoulder());
                } else {
                    assert_eq!(other.elbow(), branch.elbow());
                }
            }
        }
    }
}

#[test]
fn a_resolvable_wrist_bend_keeps_the_two_families_distinct() {
    let robot = OPWKinematics::new(equal_links());
    let mut seed = SEED;
    seed[J5] = 1e-7;
    let solutions = robot.ik(&robot.forward(&seed), Some(&seed));
    for wrist in [Wrist::Principal, Wrist::Flipped] {
        let branch = Branch::new(Shoulder::First, Elbow::Positive, wrist);
        let slot = solutions[branch.index()];
        assert_eq!(slot.meta.geometry, GeometryStatus::Verified);
        assert!(!slot.meta.singularities.contains(Singularity::WristZero));
        assert!(!slot.meta.recovery.contains(Recovery::WristPhase));
        assert_eq!(slot.meta.family.iter().collect::<Vec<_>>(), [branch]);
        assert!((slot.joints[J5].abs() - seed[J5]).abs() < 1e-12);
    }
}

#[test]
fn finite_joint_angles_that_fail_fk_remain_available_for_diagnosis() {
    let robot = OPWKinematics::new(equal_links());
    let target = robot.forward(&SEED);
    // At this magnitude the ULP exceeds an entire turn. Choosing turns near
    // this reference loses all the low angle bits on the way back through FK.
    let mut reference = SEED;
    reference[J1] = 1e20;
    for slot in robot.ik(&target, Some(&reference)) {
        assert_eq!(slot.meta.geometry, GeometryStatus::FailedPoseCheck);
        assert!(slot.joints.iter().all(|q| q.is_finite()));
        assert!(
            robot
                .forward(&slot.joints)
                .translation
                .distance(target.translation)
                > 1e-6
        );
    }
}

#[test]
fn an_infeasible_wrist_phase_keeps_the_labelled_singular_family() {
    let mut seed = SEED;
    seed[J5] = 0.0;
    // At this wrist pole J4 + J6 = -0.5. The two allowed arcs
    // require a sum in [0.2, 0.4], even after a whole turn.
    let mut lower = [0.0; 6];
    let mut upper = [0.0; 6];
    for joint in [J4, J6] {
        lower[joint] = 0.1;
        upper[joint] = 0.2;
    }
    let robot =
        OPWKinematics::new_with_constraints(equal_links(), Constraints::new(lower, upper, BY_PREV));
    let solutions = robot.ik(&robot.forward(&seed), None);
    for wrist in [Wrist::Principal, Wrist::Flipped] {
        let branch = Branch::new(Shoulder::First, Elbow::Positive, wrist);
        let slot = solutions[branch.index()];
        assert_eq!(slot.meta.geometry, GeometryStatus::NoRepresentativeFound);
        assert!(slot.meta.singularities.contains(Singularity::WristZero));
        assert_eq!(slot.meta.family.iter().count(), 2);
        assert!(slot.meta.recovery.is_empty());
        assert!(slot.joints.iter().all(|q| q.is_nan()));
        assert!(slot.limit_margin_rad.is_nan());
    }
}

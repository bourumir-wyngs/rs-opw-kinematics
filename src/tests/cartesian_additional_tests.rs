//! Boundary, failure, and concurrency regressions for Cartesian planning.

use super::tests::{
    annotated_pose_at, joint_step, joints, linear_robot, pose_at, test_planner, test_robot,
    test_trimesh,
};
use super::*;
use crate::collisions::{CheckMode, CollisionBody, transform_mesh};
use crate::constraints::Constraints;
use crate::kinematic_traits::Kinematics;
use glam::{DQuat, DVec3};
use rayon::prelude::*;
use std::sync::atomic::AtomicUsize;

#[test]
fn zero_search_limits_use_defaults_and_nonzero_limits_are_preserved() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, 0);
    for value in [0, 1, 7, usize::MAX] {
        planner.max_cartesian_layer_states = value;
        planner.max_reconfiguration_prefix_candidates = value;
        planner.preferred_onboarding_suffix_candidates = value;
        planner.max_solutions_await = value;
        for (actual, default) in [
            (
                planner.cartesian_layer_state_limit(),
                DEFAULT_CARTESIAN_LAYER_STATES,
            ),
            (
                planner.reconfiguration_prefix_candidate_limit(),
                DEFAULT_RECONFIGURATION_PREFIX_CANDIDATES,
            ),
            (
                planner.preferred_onboarding_suffix_candidate_limit(),
                DEFAULT_PREFERRED_ONBOARDING_SUFFIX_CANDIDATES,
            ),
            (
                planner.max_solutions_await_limit(),
                DEFAULT_MAX_SOLUTIONS_AWAIT,
            ),
        ] {
            assert_eq!(actual, if value == 0 { default } else { value });
        }
    }
}

#[test]
fn pose_sampling_respects_translation_and_rotation_spacing() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, usize::MAX);
    planner.check_step_m = 0.26;
    planner.check_step_rad = 0.26;
    let start = Pose::from_parts(DVec3::new(2.0, 3.0, 4.0), DQuat::from_rotation_z(0.3));

    for (translation, rotation, segments) in [
        (1.0, 0.0, 4),
        (0.0, 2.0, 8),
        (1.0, 2.0, 8),
        (0.0, 0.0, 1),
        (0.1, 0.1, 1),
    ] {
        let end = Pose::from_parts(
            start.translation + DVec3::X * translation,
            DQuat::from_rotation_z(0.3 + rotation),
        );
        let mut samples = Vec::new();
        planner.add_intermediate_poses(&start, &end, PathFlags::LANDING, &mut samples);
        assert_eq!(samples.len(), segments - 1);
        for (index, sample) in samples.iter().enumerate() {
            let fraction = (index + 1) as f64 / segments as f64;
            assert!(
                (sample.pose.translation - start.translation.lerp(end.translation, fraction))
                    .length()
                    < 1e-12
            );
            let expected_rotation = start.rotation.slerp(end.rotation, fraction);
            assert!((sample.pose.rotation.dot(expected_rotation).abs() - 1.0).abs() < 1e-12);
            assert_eq!(
                sample.flags.bits(),
                (PathFlags::LIN_INTERP | PathFlags::LANDING).bits()
            );
            assert_eq!(sample.split_depth, 0);
        }
        let chain: Vec<_> = std::iter::once(start)
            .chain(samples.iter().map(|sample| sample.pose))
            .chain(std::iter::once(end))
            .collect();
        for edge in chain.windows(2) {
            assert!(
                (edge[1].translation - edge[0].translation).length()
                    <= planner.check_step_m + 1e-12
            );
            let angle = (edge[1].rotation * edge[0].rotation.inverse())
                .to_scaled_axis()
                .length();
            assert!(angle <= planner.check_step_rad + 1e-12);
        }
    }
}

#[test]
fn repeated_trace_poses_keep_their_original_roles() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, usize::MAX);
    planner.check_step_m = 0.5;
    let poses = planner.with_intermediate_poses(
        &pose_at(0.0),
        &[pose_at(1.0), pose_at(1.0)],
        &pose_at(2.0),
    );
    let expected = [
        (0.0, PathFlags::LAND),
        (0.5, PathFlags::LIN_INTERP | PathFlags::LANDING),
        (1.0, PathFlags::TRACE),
        (1.0, PathFlags::TRACE),
        (1.5, PathFlags::LIN_INTERP | PathFlags::PARKING),
        (2.0, PathFlags::PARK),
    ];
    assert_eq!(poses.len(), expected.len());
    for (actual, (x, flags)) in poses.iter().zip(expected) {
        assert_eq!(actual.pose.translation.x, x);
        assert_eq!(actual.flags.bits(), flags.bits());
        assert_eq!(actual.split_depth, 0);
    }
    let path = planner
        .plan(
            &joints(0.0),
            &pose_at(0.0),
            vec![pose_at(1.0), pose_at(1.0)],
            &pose_at(2.0),
        )
        .unwrap();
    assert_eq!(
        path.iter()
            .filter(|step| step.flags.contains(PathFlags::TRACE))
            .count(),
        2
    );
}

#[test]
fn empty_strokes_preserve_land_parking_and_park_in_both_modes() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, usize::MAX);
    planner.check_step_m = 0.25;
    for approximate in [false, true] {
        for include_interpolation in [false, true] {
            planner.include_linear_interpolation = include_interpolation;
            for park in [0.0, 1.0] {
                let path = run_plan(&planner, approximate, Vec::new(), &pose_at(park)).unwrap();
                let expected_len = if include_interpolation && park != 0.0 {
                    5
                } else {
                    2
                };
                assert_eq!(path.len(), expected_len);
                assert_eq!(path[0].joints, joints(0.0));
                assert_eq!(path[0].flags.bits(), PathFlags::LAND.bits());
                assert_eq!(path[0].move_into, MoveKind::Joint);
                assert_eq!(path.last().unwrap().joints, joints(park));
                assert_eq!(path.last().unwrap().flags.bits(), PathFlags::PARK.bits());
                assert!(
                    path[1..]
                        .iter()
                        .all(|step| step.move_into == MoveKind::Cartesian)
                );
                for (index, sample) in path[1..path.len() - 1].iter().enumerate() {
                    assert_eq!(sample.joints, joints((index + 1) as f64 * 0.25));
                    assert_eq!(
                        sample.flags.bits(),
                        (PathFlags::LIN_INTERP | PathFlags::PARKING).bits()
                    );
                }
            }
        }
    }
}

#[test]
fn refinement_uses_both_parent_depths_and_preserves_midpoint_geometry() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, usize::MAX);
    for (limit, left_depth, right_depth, permitted) in [
        (0, 0, 0, false),
        (2, 2, 0, false),
        (2, 0, 2, false),
        (3, 1, 2, true),
    ] {
        planner.linear_recursion_depth = limit;
        let mut left = annotated_pose_at(0.0, PathFlags::LAND | PathFlags::FORWARDS);
        left.split_depth = left_depth;
        let mut right = annotated_pose_at(2.0, PathFlags::TRACE | PathFlags::BACKWARDS);
        right.pose.rotation = DQuat::from_rotation_y(1.0);
        right.split_depth = right_depth;
        let mut poses = vec![left, right];
        assert!(!planner.refine_transition(&mut poses, 0));
        assert_eq!(planner.refine_transition(&mut poses, 1), permitted);
        assert_eq!(poses.len(), if permitted { 3 } else { 2 });
        assert_eq!(poses[0].pose, left.pose);
        assert_eq!(poses.last().unwrap().pose, right.pose);
        if permitted {
            let midpoint = &poses[1];
            assert_eq!(midpoint.pose.translation.x, 1.0);
            assert!((midpoint.pose.rotation.dot(DQuat::from_rotation_y(0.5)) - 1.0).abs() < 1e-12);
            assert_eq!(midpoint.split_depth, 3);
            assert_eq!(
                midpoint.flags.bits(),
                (PathFlags::LIN_INTERP
                    | PathFlags::LANDING
                    | PathFlags::FORWARDS
                    | PathFlags::BACKWARDS)
                    .bits()
            );
            assert!(!planner.refine_transition(&mut poses, 1));
            assert_eq!(poses.len(), 3);
        }
    }
}

#[test]
fn adaptive_refinement_stops_at_the_configured_budget() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, usize::MAX);
    planner.check_step_m = 10.0;
    planner.max_transition_cost = 1.5; // Six joints may each move by at most 0.25 here.
    for approximate in [false, true] {
        for depth in 0..=2 {
            planner.linear_recursion_depth = depth;
            let result = run_plan(&planner, approximate, Vec::new(), &pose_at(1.0));
            if depth < 2 {
                assert!(
                    result
                        .unwrap_err()
                        .contains("No Cartesian suffix worked out")
                );
            } else {
                let path = result.unwrap();
                assert_eq!(
                    path.iter().map(|step| step.joints[0]).collect::<Vec<_>>(),
                    vec![0.0, 0.25, 0.5, 0.75, 1.0]
                );
                assert!(
                    path[1..]
                        .iter()
                        .all(|step| step.move_into == MoveKind::Cartesian)
                );
            }
        }
    }
}

#[test]
fn hidden_interpolation_still_rejects_collision_between_clear_endpoints() {
    let mut robot = linear_robot();
    add_test_obstacle(&mut robot);
    assert!(!robot.collides(&joints(0.0)));
    assert!(!robot.collides(&joints(3.0)));
    assert!(!robot.collides(&joints(4.0)));
    assert!(robot.collides(&joints(1.0)));
    let mut planner = test_planner(&robot, usize::MAX);
    planner.check_step_m = 0.5;
    planner.debug = true; // Also exercise failure diagnostics against active geometry.
    for approximate in [false, true] {
        for include_interpolation in [false, true] {
            planner.include_linear_interpolation = include_interpolation;
            let err =
                run_plan(&planner, approximate, vec![pose_at(3.0)], &pose_at(4.0)).unwrap_err();
            assert!(err.contains("No Cartesian suffix worked out"));
        }
    }
}

#[test]
fn unreachable_landing_is_reported_before_suffix_planning() {
    let calls = Arc::new(AtomicUsize::new(0));
    let callback_calls = calls.clone();
    let robot = scripted_robot(move |pose, _| {
        assert_eq!(pose.translation.x, 0.0);
        callback_calls.fetch_add(1, Ordering::SeqCst);
        Vec::new()
    });
    let planner = test_planner(&robot, usize::MAX);
    for approximate in [false, true] {
        let err = run_plan(&planner, approximate, vec![pose_at(1.0)], &pose_at(2.0)).unwrap_err();
        assert_eq!(err, "Unable to start from onboarding point");
    }
    assert_eq!(calls.load(Ordering::SeqCst), 2);
}

#[test]
fn empty_cartesian_graph_does_not_request_ik() {
    let robot = scripted_robot(|_, _| panic!("empty graph must not request IK"));
    let planner = test_planner(&robot, usize::MAX);
    let path = planner
        .plan_cartesian_graph(
            &joints(0.0),
            &annotated_pose_at(0.0, PathFlags::LAND),
            &[],
            &AtomicBool::new(false),
            usize::MAX,
            usize::MAX,
            &PlanningCollisionCache::new(&robot),
        )
        .unwrap_or_else(|_| panic!("empty graph is a successful empty extension"));
    assert!(path.is_empty());
}

#[test]
fn cancellation_during_suffix_planning_discards_the_partial_extension() {
    let stop = Arc::new(AtomicBool::new(false));
    let callback_stop = stop.clone();
    let robot = scripted_robot(move |pose, _| {
        assert_eq!(
            pose.translation.x, 1.0,
            "no IK should run after cancellation"
        );
        callback_stop.store(true, Ordering::SeqCst);
        vec![joints(1.0)]
    });
    let planner = test_planner(&robot, usize::MAX);
    let poses = [
        annotated_pose_at(0.0, PathFlags::LAND),
        annotated_pose_at(1.0, PathFlags::TRACE),
        annotated_pose_at(2.0, PathFlags::PARK),
    ];
    let err = planner
        .probe_cartesian_suffix(
            &joints(0.0),
            &poses,
            &stop,
            usize::MAX,
            usize::MAX,
            &PlanningCollisionCache::new(&robot),
        )
        .unwrap_err();
    assert_eq!(err, "Stopped");
    // Cancellation also applies when there is no remaining edge to expand.
    let err = planner
        .probe_cartesian_suffix(
            &joints(0.0),
            &poses[..1],
            &stop,
            usize::MAX,
            usize::MAX,
            &PlanningCollisionCache::new(&robot),
        )
        .unwrap_err();
    assert_eq!(err, "Stopped");
}

#[test]
fn plan_rank_comparison_honors_every_tie_breaker() {
    let reference = PlanRank {
        stroke_reconfigurations: 2,
        stroke_reconfiguration_steps: 3,
        total_transition_cost: 4.0,
        output_steps: 5,
    };
    let better = [
        PlanRank {
            stroke_reconfigurations: 1,
            stroke_reconfiguration_steps: 100,
            total_transition_cost: 100.0,
            output_steps: 100,
        },
        PlanRank {
            stroke_reconfiguration_steps: 2,
            total_transition_cost: 100.0,
            output_steps: 100,
            ..reference
        },
        PlanRank {
            total_transition_cost: 3.0,
            output_steps: 100,
            ..reference
        },
        PlanRank {
            output_steps: 4,
            ..reference
        },
    ];
    assert!(!reference.is_better_than(&reference));
    for rank in better {
        assert!(rank.is_better_than(&reference));
        assert!(!reference.is_better_than(&rank));
    }
}

#[test]
fn onboarding_keeps_the_best_fallback_while_searching_for_an_uninterrupted_path() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, usize::MAX);
    planner.debug = true;
    let fallback = |values: &[f64]| {
        SuffixPlanningOutcome::new(
            joints(0.0),
            values
                .iter()
                .map(|&q| {
                    joint_step(
                        q,
                        PathFlags::TRACE | PathFlags::RECONFIGURING,
                        MoveKind::Joint,
                    )
                })
                .collect(),
            &[1.0; 6],
        )
    };
    let mut best = None;
    let mut attempts = 0;
    let candidates = [
        fallback(&[2.0, 1.0]),
        fallback(&[1.0]),
        fallback(&[3.0, 2.0]),
    ];
    assert!(
        planner
            .try_onboarding_candidates(&joints(0.0), &candidates, &mut best, &mut attempts)
            .is_none()
    );
    assert_eq!(attempts, 3);
    assert_eq!(best.as_ref().unwrap().path.len(), 2);
    assert_eq!(best.as_ref().unwrap().path[1].joints, joints(1.0));
    let good = SuffixPlanningOutcome::new(
        joints(0.0),
        vec![joint_step(0.0, PathFlags::PARK, MoveKind::Cartesian)],
        &[1.0; 6],
    );
    let result = planner
        .try_onboarding_candidates(&joints(0.0), &[good], &mut best, &mut attempts)
        .unwrap();
    assert!(result.is_good_enough());
    assert_eq!(attempts, 4);
}

#[test]
fn onboarding_tries_remaining_collected_suffixes_before_reprobing_ik() {
    for approximate in [false, true] {
        let calls = Arc::new(AtomicUsize::new(0));
        let callback_calls = calls.clone();
        let robot = scripted_robot(move |pose, previous| {
            callback_calls.fetch_add(1, Ordering::SeqCst);
            if pose.translation.x == 0.0 {
                vec![joints(1.0), joints(0.0)]
            } else {
                vec![*previous]
            }
        });
        let mut planner = test_planner(&robot, usize::MAX);
        planner.check_step_m = 10.0;
        planner.max_solutions_await = 2;
        planner.preferred_onboarding_suffix_candidates = 1;
        planner.rrt.max_try = 0;
        planner.debug = true;
        let path = run_plan(&planner, approximate, Vec::new(), &pose_at(1.0)).unwrap();
        assert_eq!(calls.load(Ordering::SeqCst), 3); // Landing plus two suffix graphs; no re-probe.
        assert_eq!(path.len(), 2);
        assert_eq!(path[0].joints, joints(0.0));
        assert_eq!(path[1].flags.bits(), PathFlags::PARK.bits());
    }
}

#[test]
fn feasible_suffix_with_failed_onboarding_has_a_distinct_error() {
    let robot = scripted_robot(|_, _| vec![joints(1.0)]);
    let mut planner = test_planner(&robot, usize::MAX);
    planner.check_step_m = 10.0;
    planner.max_reconfiguration_prefix_candidates = usize::MAX;
    planner.rrt.max_try = 0;
    for approximate in [false, true] {
        let err = run_plan(&planner, approximate, Vec::new(), &pose_at(1.0)).unwrap_err();
        assert!(err.contains("No onboarding RRT worked out"));
        assert!(!err.contains("No Cartesian suffix worked out"));
    }
}

#[test]
fn onboarding_marks_only_its_endpoint_as_land() {
    let robot = scripted_robot(|_, _| Vec::new());
    let mut planner = test_planner(&robot, usize::MAX);
    // An empty cell and a step spanning the bounded sampling box guarantee
    // connection in one attempt, independent of the sampled configuration.
    planner.rrt.step_size_joint_space = 100.0;
    let mut path = Vec::new();
    planner
        .append_onboarding(
            &joints(0.0),
            &joints(1.0),
            &AtomicBool::new(false),
            &mut path,
        )
        .unwrap();
    assert!(path.len() >= 2);
    assert_eq!(path[0].joints, joints(0.0));
    let (land, transit) = path.split_last().unwrap();
    assert_eq!(land.joints, joints(1.0));
    assert_eq!(
        land.flags.bits(),
        (PathFlags::ONBOARDING | PathFlags::LAND).bits()
    );
    assert!(
        transit
            .iter()
            .all(|step| step.flags.bits() == PathFlags::ONBOARDING.bits())
    );
    assert!(path.iter().all(|step| step.move_into == MoveKind::Joint));
}

#[test]
fn failed_prefix_is_rolled_back_before_a_later_reconfiguration_succeeds() {
    let robot = linear_robot();
    let mut planner = test_planner(&robot, usize::MAX);
    planner.include_linear_interpolation = false;
    planner.debug = true;
    for prefix_flags in [PathFlags::TRACE, PathFlags::LIN_INTERP | PathFlags::LANDING] {
        let prefix = annotated_pose_at(0.5, prefix_flags);
        let target = annotated_pose_at(1.0, PathFlags::TRACE);
        let candidates: Vec<_> = [0.25, 0.5]
            .into_iter()
            .map(|q| CartesianGraphFailureCandidate {
                planned_prefix: vec![joints(q)],
                transition: Transition {
                    from: prefix,
                    to: target,
                    previous: joints(q),
                    solutions: if q == 0.25 {
                        Vec::new()
                    } else {
                        vec![joints(q)]
                    },
                },
                prefix_cost: q * 6.0,
            })
            .collect();
        let mut path = vec![joint_step(0.0, PathFlags::LAND, MoveKind::Joint)];
        let mut previous = joints(0.0);
        let mut step = 7;
        assert!(planner.append_reconfiguration_candidates(
            &candidates,
            &[prefix],
            &target,
            &mut ReconfigurationAppendState {
                stop: &AtomicBool::new(false),
                trace: &mut path,
                previous_joints: &mut previous,
                step: &mut step,
            }
        ));
        assert_eq!(
            path.iter().map(|step| step.joints[0]).collect::<Vec<_>>(),
            vec![0.0, 0.5, 0.5]
        );
        assert_eq!(path[0].flags.bits(), PathFlags::LAND.bits());
        assert_eq!(path[1].move_into, MoveKind::Cartesian);
        assert_eq!(
            path[1].flags.bits(),
            (prefix_flags & !PathFlags::LIN_INTERP).bits()
        );
        assert_eq!(path[2].move_into, MoveKind::Joint);
        assert_eq!(
            path[2].flags.bits(),
            (PathFlags::TRACE | PathFlags::RECONFIGURING).bits()
        );
        assert_eq!(previous, joints(0.5));
        assert_eq!(
            step,
            if prefix_flags.contains(PathFlags::TRACE) {
                8
            } else {
                7
            }
        );
    }
}

#[test]
fn exhaustive_retry_replaces_a_fast_stroke_fallback_with_cartesian_motion() {
    let robot = scripted_robot(|pose, previous| match pose.translation.x as i32 {
        0 => vec![joints(0.0)],
        1 => vec![joints(0.1), joints(0.2)],
        2 if previous[0] == 0.1 => vec![joints(0.6)],
        2 => vec![joints(0.3)],
        3 => vec![*previous],
        _ => Vec::new(),
    });
    let mut planner = test_planner(&robot, 1);
    planner.check_step_m = 10.0;
    planner.max_transition_cost = 1.3;
    planner.allow_reconfigure = true;
    // Every point in the bounded, empty cell connects in one RRT extension.
    // Assertions do not depend on the sampled intermediate configuration.
    planner.rrt.step_size_joint_space = 100.0;
    planner.debug = true;
    let steps = vec![pose_at(1.0), pose_at(2.0)];
    let fast = run_plan(&planner, true, steps.clone(), &pose_at(3.0)).unwrap();
    assert!(!PlanRank::from_path(&fast, &[1.0; 6]).is_good_enough());
    assert!(
        fast.iter()
            .any(|step| step.flags.contains(PathFlags::RECONFIGURING))
    );
    let complete = run_plan(&planner, false, steps, &pose_at(3.0)).unwrap();
    assert_eq!(
        complete
            .iter()
            .map(|step| step.joints[0])
            .collect::<Vec<_>>(),
        vec![0.0, 0.2, 0.3, 0.3]
    );
    assert!(
        complete
            .iter()
            .all(|step| !step.flags.contains(PathFlags::RECONFIGURING))
    );
    assert!(
        complete[1..]
            .iter()
            .all(|step| step.move_into == MoveKind::Cartesian)
    );
}

#[test]
fn concurrent_cold_cache_misses_respect_capacity_and_collision_results() {
    let mut robot = linear_robot();
    add_test_obstacle(&mut robot);
    let expected = [0.0, 1.0, 2.0, 3.0].map(|q| robot.collides(&joints(q)));
    assert_eq!(expected, [false, true, true, false]);
    let mut cache = PlanningCollisionCache::new(&robot);
    cache.max_entries = 2;
    rayon::ThreadPoolBuilder::new()
        .num_threads(4)
        .build()
        .unwrap()
        .install(|| {
            (0..128usize).into_par_iter().for_each(|index| {
                let q = index % 4;
                assert_eq!(cache.collides(&joints(q as f64)), expected[q]);
            });
        });
    assert_eq!(cache.results.lock().unwrap().len(), 2);
    for (q, expected) in expected.into_iter().enumerate() {
        assert_eq!(cache.collides(&joints(q as f64)), expected);
    }
    assert_eq!(cache.results.lock().unwrap().len(), 2);
}

#[test]
fn real_opw_kinematics_reaches_every_retained_cartesian_pose() {
    use crate::constraints::BY_PREV;
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;

    let start = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8];
    for parameters in [Parameters::staubli_tx2_160(), Parameters::irb2400_10()] {
        // This exercises real IK/FK and constraints; the separate obstacle tests
        // exercise geometry. Narrow limits isolate the branch containing start.
        let mut robot = linear_robot();
        robot.kinematics = Arc::new(OPWKinematics::new_with_constraints(
            parameters,
            Constraints::new(start.map(|q| q - 0.15), start.map(|q| q + 0.15), BY_PREV),
        ));
        let land = robot.forward(&start);
        let target = Pose::from_parts(
            land.translation + DVec3::new(0.005, 0.004, 0.003),
            DQuat::from_rotation_z(0.005) * land.rotation,
        );
        let mut planner = test_planner(&robot, usize::MAX);
        planner.check_step_m = 0.002;
        planner.check_step_rad = 0.004;
        planner.max_transition_cost = 0.1;
        planner.rrt.max_try = 0; // A Cartesian stroke from this branch needs no RRT.
        let sampled = planner.with_intermediate_poses(&land, &[target], &land);
        for approximate in [false, true] {
            for include_interpolation in [false, true] {
                planner.include_linear_interpolation = include_interpolation;
                let path = if approximate {
                    planner.plan_fast_approximate(&start, &land, vec![target], &land)
                } else {
                    planner.plan(&start, &land, vec![target], &land)
                }
                .expect("a short nonsingular stroke should stay on the constrained branch");
                let expected: Vec<_> = sampled
                    .iter()
                    .filter(|pose| {
                        include_interpolation || !pose.flags.contains(PathFlags::LIN_INTERP)
                    })
                    .collect();
                assert_eq!(path.len(), expected.len());
                assert_eq!(path[0].move_into, MoveKind::Joint);
                for (index, (waypoint, expected_pose)) in path.iter().zip(expected).enumerate() {
                    let actual_pose = robot.forward(&waypoint.joints);
                    assert!(
                        (actual_pose.translation - expected_pose.pose.translation).length() < 1e-8
                    );
                    assert!(
                        (actual_pose.rotation.dot(expected_pose.pose.rotation).abs() - 1.0).abs()
                            < 1e-10
                    );
                    assert!(waypoint.joints.iter().all(|q| q.is_finite()));
                    assert!(
                        robot
                            .constraints()
                            .as_ref()
                            .unwrap()
                            .compliant(&waypoint.joints)
                    );
                    assert_eq!(waypoint.flags.bits(), expected_pose.flags.bits());
                    if index > 0 {
                        assert_eq!(waypoint.move_into, MoveKind::Cartesian);
                    }
                }
                if include_interpolation {
                    for edge in path.windows(2) {
                        assert!(
                            transition_costs(
                                &edge[0].joints,
                                &edge[1].joints,
                                &planner.transition_coefficients
                            ) <= planner.max_transition_cost + 1e-12
                        );
                    }
                }
            }
        }
    }
}

fn run_plan(
    planner: &Cartesian<'_>,
    approximate: bool,
    steps: Vec<Pose>,
    park: &Pose,
) -> Result<Vec<AnnotatedJoints>, String> {
    if approximate {
        planner.plan_fast_approximate(&joints(0.0), &pose_at(0.0), steps, park)
    } else {
        planner.plan(&joints(0.0), &pose_at(0.0), steps, park)
    }
}

fn add_test_obstacle(robot: &mut KinematicsWithShape) {
    robot.body.safety.mode = CheckMode::FirstCollisionOnly;
    robot.body.safety.to_robot_default = 0.1;
    robot.body.safety.to_environment = 0.1;
    // The linear fixture gives every joint the same transform. Separate their
    // local meshes so only the first joint can reach the obstacle.
    robot.body.joint_meshes = std::array::from_fn(|index| {
        transform_mesh(
            &test_trimesh(),
            &Pose::from_translation(DVec3::Y * (10.0 * index as f64)).to_f32(),
        )
    });
    robot.body.collision_environment.push(CollisionBody {
        mesh: test_trimesh(),
        pose: pose_at(1.5).to_f32(),
    });
}

struct ScriptedKinematics<F> {
    solve: F,
    constraints: Option<Constraints>,
}

fn scripted_robot<F>(solve: F) -> KinematicsWithShape
where
    F: Fn(&Pose, &Joints) -> Solutions + Send + Sync + 'static,
{
    let mut robot = test_robot();
    robot.kinematics = Arc::new(ScriptedKinematics {
        solve,
        constraints: Some(Constraints::new([-2.0; 6], [2.0; 6], 0.0)),
    });
    robot
}

impl<F> Kinematics for ScriptedKinematics<F>
where
    F: Fn(&Pose, &Joints) -> Solutions + Send + Sync,
{
    fn inverse(&self, pose: &Pose) -> Solutions {
        (self.solve)(pose, &joints(0.0))
    }
    fn inverse_continuing(&self, pose: &Pose, previous: &Joints) -> Solutions {
        (self.solve)(pose, previous)
    }
    fn forward(&self, joints: &Joints) -> Pose {
        pose_at(joints[0])
    }
    fn inverse_5dof(&self, pose: &Pose, _j6: f64) -> Solutions {
        self.inverse(pose)
    }
    fn inverse_continuing_5dof(&self, pose: &Pose, previous: &Joints) -> Solutions {
        self.inverse_continuing(pose, previous)
    }
    fn constraints(&self) -> &Option<Constraints> {
        &self.constraints
    }
    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        [self.forward(joints); 6]
    }
}

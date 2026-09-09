use super::{RRTPlanner, joint_space_bounds};
use crate::collisions::{CheckMode, CollisionBody, RobotBody, SafetyDistances};
use crate::constraints::{BY_PREV, Constraints};
use crate::kinematic_traits::{Joints, Kinematics};
use crate::kinematics_impl::OPWKinematics;
use crate::kinematics_with_shape::KinematicsWithShape;
use crate::parameters::opw_kinematics::Parameters;
use parry3d::math::Vector;
use parry3d::shape::TriMesh;
use std::f64::consts::{PI, TAU};
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

fn robot_with_constraints(constraints: Constraints) -> KinematicsWithShape {
    KinematicsWithShape {
        kinematics: Arc::new(OPWKinematics::new_with_constraints(
            Parameters::new(),
            constraints,
        )),
        body: RobotBody {
            joint_meshes: std::array::from_fn(|_| {
                TriMesh::new(
                    vec![
                        Vector::new(0.0, 0.0, 0.0),
                        Vector::new(0.001, 0.0, 0.0),
                        Vector::new(0.0, 0.001, 0.0),
                    ],
                    vec![[0, 1, 2]],
                )
                .expect("test mesh should be valid")
            }),
            tool: None,
            base: None,
            collision_environment: Vec::new(),
            safety: SafetyDistances::standard(CheckMode::NoCheck),
        },
    }
}

fn first_joint_constraints(from: f64, to: f64) -> Constraints {
    let mut lower = [-0.25; 6];
    let mut upper = [0.25; 6];
    lower[0] = from;
    upper[0] = to;
    Constraints::new(lower, upper, BY_PREV)
}

fn first_joint(value: f64) -> Joints {
    let mut joints = [0.0; 6];
    joints[0] = value;
    joints
}

fn robot_with_obstacle() -> KinematicsWithShape {
    let mut robot = robot_with_constraints(Constraints::new([-0.25; 6], [0.25; 6], BY_PREV));
    // Place a small tetrahedron one unit from J1's rotation axis. The obstacle
    // occupies its zero-angle position; rotating J1 by 0.1 radians clears it.
    // With zero-length links all rotations preserve distance from the origin.
    // Distinct radii keep the link meshes apart with self-collision checks on.
    robot.body.joint_meshes = std::array::from_fn(|joint| {
        let radius = 1.0 + joint as f32;
        TriMesh::new(
            vec![
                Vector::new(radius, 0.0, 0.0),
                Vector::new(radius + 0.01, 0.0, 0.0),
                Vector::new(radius, 0.01, 0.0),
                Vector::new(radius, 0.0, 0.01),
            ],
            vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        )
        .expect("test tetrahedron should be valid")
    });
    robot.body.collision_environment.push(CollisionBody {
        mesh: robot.body.joint_meshes[0].clone(),
        pose: robot.kinematics.forward_with_joint_poses(&[0.0; 6])[0].to_f32(),
    });
    robot.body.safety = SafetyDistances::standard(CheckMode::FirstCollisionOnly);
    robot
}

#[test]
fn colliding_endpoints_are_rejected_including_stationary_requests() {
    let robot = robot_with_obstacle();
    let blocked = first_joint(0.0);
    let free = first_joint(0.2);
    assert!(robot.constraints().as_ref().unwrap().compliant(&blocked));
    assert!(robot.collides(&blocked));
    assert!(!robot.collides(&free));

    for (start, goal) in [(blocked, free), (free, blocked), (blocked, blocked)] {
        for smooth in [0, 8] {
            let planner = RRTPlanner {
                max_try: 1,
                smooth,
                ..RRTPlanner::default()
            };
            assert_eq!(
                planner.plan_rrt(&start, &goal, &robot, &AtomicBool::new(false)),
                Err("failed".to_string())
            );
        }
    }
}

#[test]
fn collision_checked_paths_preserve_endpoints_and_avoid_obstacles() {
    let mut robot = robot_with_obstacle();
    // Every sample lies in an interval clear of the obstacle, so success does
    // not depend on drawing a lucky random sample, even with a one-try budget.
    let constraints = first_joint_constraints(0.1, 0.25);
    robot.kinematics = Arc::new(OPWKinematics::new_with_constraints(
        Parameters::new(),
        constraints,
    ));
    assert!(robot.collides(&first_joint(0.0)));
    let start = first_joint(0.1);
    let goal = first_joint(0.2);

    for smooth in [0, 8] {
        let planner = RRTPlanner {
            step_size_joint_space: 0.05,
            max_try: 1,
            smooth,
            debug: false,
        };
        let path = planner
            .plan_rrt(&start, &goal, &robot, &AtomicBool::new(false))
            .expect("the allowed interval clears the obstacle");

        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
        for joints in &path {
            assert!(constraints.compliant(joints));
            assert!(!robot.collides(joints), "colliding waypoint: {joints:?}");
        }
        for edge in path.windows(2) {
            let distance = edge[0]
                .iter()
                .zip(edge[1])
                .map(|(from, to)| (to - from).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(distance <= planner.step_size_joint_space + 1e-12);
            for step in 1..10 {
                let p = step as f64 / 10.0;
                let joints = std::array::from_fn(|j| edge[0][j] + p * (edge[1][j] - edge[0][j]));
                assert!(
                    !robot.collides(&joints),
                    "colliding edge sample: {joints:?}"
                );
            }
        }
    }
}

#[test]
fn zero_search_budget_only_allows_valid_stationary_requests() {
    let robot = robot_with_obstacle();
    let start = first_joint(0.1);
    let goal = first_joint(0.2);
    assert!(!robot.collides(&start));
    assert!(!robot.collides(&goal));

    for smooth in [0, 8] {
        let planner = RRTPlanner {
            max_try: 0,
            smooth,
            ..RRTPlanner::default()
        };
        assert_eq!(
            planner.plan_rrt(&start, &goal, &robot, &AtomicBool::new(false)),
            Err("failed".to_string())
        );
        assert_eq!(
            planner.plan_rrt(&start, &start, &robot, &AtomicBool::new(false)),
            Ok(vec![start])
        );
    }
}

#[test]
fn overflowing_sampling_intervals_are_rejected() {
    let robot = robot_with_constraints(Constraints::new([0.0; 6], [0.0; 6], BY_PREV));
    for (start, goal) in [
        (first_joint(-f64::MAX), first_joint(f64::MAX)),
        (first_joint(f64::MAX), first_joint(-f64::MAX)),
    ] {
        for smooth in [0, 8] {
            let planner = RRTPlanner {
                max_try: 1,
                smooth,
                ..RRTPlanner::default()
            };
            assert_eq!(
                planner.plan_rrt(&start, &goal, &robot, &AtomicBool::new(false)),
                Err("failed".to_string())
            );
        }
    }
}

fn assert_valid_path(constraints: Constraints, start: Joints, goal: Joints) {
    assert!(constraints.compliant(&start));
    assert!(constraints.compliant(&goal));
    let robot = robot_with_constraints(constraints);

    for smooth in [0, 8] {
        let planner = RRTPlanner {
            step_size_joint_space: 0.1,
            max_try: 1,
            smooth,
            debug: false,
        };
        let path = planner
            .plan_rrt(&start, &goal, &robot, &AtomicBool::new(false))
            .expect("a collision-free connected constraint interval should be plannable");

        assert_eq!(path.first(), Some(&start));
        assert_eq!(path.last(), Some(&goal));
        for joints in &path {
            assert!(joints.iter().all(|angle| angle.is_finite()));
            assert!(constraints.compliant(joints), "invalid state: {joints:?}");
        }
        for pair in path.windows(2) {
            let distance = pair[0]
                .iter()
                .zip(pair[1])
                .map(|(from, to)| (to - from).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(distance <= planner.step_size_joint_space + 1e-12);
            for sample in 1..10 {
                let p = sample as f64 / 10.0;
                let joints = std::array::from_fn(|j| pair[0][j] + (pair[1][j] - pair[0][j]) * p);
                assert!(
                    constraints.compliant(&joints),
                    "edge leaves the allowed interval: {pair:?}, sample: {joints:?}"
                );
            }
        }
    }
}

#[test]
fn example_preserves_configured_sampling_ranges_for_unrestricted_joints() {
    let constraints = Constraints::from_degrees(
        [
            -225.0..=225.0,
            -225.0..=225.0,
            -225.0..=225.0,
            -225.0..=225.0,
            -225.0..=225.0,
            -360.0..=360.0,
        ],
        BY_PREV,
    );
    let start = [-120.0_f64, -90.0, -92.51, 18.42, 82.23, 189.35].map(f64::to_radians);
    let mut goal = start;
    goal[0] = 40_f64.to_radians();

    assert_eq!(
        joint_space_bounds(&constraints, &start, &goal),
        Ok((constraints.from, constraints.to))
    );
}

#[test]
fn unrestricted_sampling_ranges_expand_to_include_requested_turns() {
    let constraints = Constraints::new([-PI; 6], [PI; 6], BY_PREV);
    let low = first_joint(-2.0 * TAU);
    let high = first_joint(3.0 * TAU);
    let mut expected_from = [-PI; 6];
    expected_from[0] = low[0];
    let mut expected_to = [PI; 6];
    expected_to[0] = high[0];

    for (start, goal) in [(low, high), (high, low)] {
        assert_eq!(
            joint_space_bounds(&constraints, &start, &goal),
            Ok((expected_from, expected_to))
        );
    }
}

#[test]
fn equal_configured_bounds_keep_a_nonzero_sampling_range() {
    let start = first_joint(-TAU);
    let goal = first_joint(2.0 * TAU);

    for bound in [0.0, 9.0] {
        let constraints = Constraints::new([bound; 6], [bound; 6], BY_PREV);
        let (from, to) = joint_space_bounds(&constraints, &start, &goal)
            .expect("equal configured bounds mean unrestricted joints");

        for joint in 0..6 {
            assert!(from[joint].is_finite() && to[joint].is_finite());
            assert!(to[joint] - from[joint] >= TAU);
            assert!(from[joint] <= start[joint] && start[joint] <= to[joint]);
            assert!(from[joint] <= goal[joint] && goal[joint] <= to[joint]);
        }
    }
}

#[test]
fn out_of_bounds_endpoints_are_rejected_including_stationary_requests() {
    let robot = robot_with_constraints(Constraints::new([-0.25; 6], [0.25; 6], BY_PREV));
    let invalid = first_joint(1.0);
    for (start, goal) in [(invalid, [0.0; 6]), ([0.0; 6], invalid), (invalid, invalid)] {
        for smooth in [0, 8] {
            let planner = RRTPlanner {
                max_try: 1,
                smooth,
                ..RRTPlanner::default()
            };
            assert!(
                planner
                    .plan_rrt(&start, &goal, &robot, &AtomicBool::new(false))
                    .is_err()
            );
        }
    }
}

#[test]
fn different_allowed_components_are_rejected_even_with_a_large_step() {
    for (constraints, start, goal) in [
        (
            first_joint_constraints(170_f64.to_radians(), -170_f64.to_radians()),
            first_joint(175_f64.to_radians()),
            first_joint(-175_f64.to_radians()),
        ),
        (
            first_joint_constraints(-10_f64.to_radians(), 10_f64.to_radians()),
            first_joint(0.0),
            first_joint(TAU),
        ),
        (
            first_joint_constraints(TAU, 0.0),
            first_joint(0.0),
            first_joint(TAU),
        ),
    ] {
        assert!(constraints.compliant(&start));
        assert!(constraints.compliant(&goal));
        let robot = robot_with_constraints(constraints);
        for smooth in [0, 8] {
            let planner = RRTPlanner {
                step_size_joint_space: 100.0,
                max_try: 1,
                smooth,
                debug: false,
            };
            assert!(
                planner
                    .plan_rrt(&start, &goal, &robot, &AtomicBool::new(false))
                    .is_err()
            );
        }
    }
}

#[test]
fn wrapped_bounds_allow_paths_inside_the_same_component() {
    let constraints = first_joint_constraints(170_f64.to_radians(), -170_f64.to_radians());
    for (start, goal) in [(175_f64, 185_f64), (-185_f64, -175_f64)] {
        assert_valid_path(
            constraints,
            first_joint(start.to_radians()),
            first_joint(goal.to_radians()),
        );
    }
}

#[test]
fn ordinary_bounds_preserve_endpoints_on_another_turn() {
    let constraints = first_joint_constraints(-10_f64.to_radians(), 10_f64.to_radians());
    for (start, goal) in [(355_f64, 365_f64), (-365_f64, -355_f64)] {
        assert_valid_path(
            constraints,
            first_joint(start.to_radians()),
            first_joint(goal.to_radians()),
        );
    }
}

#[test]
fn accepted_boundary_endpoints_are_preserved() {
    let constraints = first_joint_constraints(-180_f64.to_radians(), -49_f64.to_radians());
    for (start, goal) in [(-49_f64, -100_f64), (-100_f64, -49_f64)] {
        assert_valid_path(
            constraints,
            first_joint(start.to_radians()),
            first_joint(goal.to_radians()),
        );
    }
}

#[test]
fn stationary_requests_without_constraints_remain_supported() {
    let mut robot = robot_with_constraints(Constraints::default());
    robot.kinematics = Arc::new(OPWKinematics::new(Parameters::new()));
    let joints = [0.1; 6];
    for smooth in [0, 8] {
        let planner = RRTPlanner {
            max_try: 0,
            smooth,
            ..RRTPlanner::default()
        };
        assert_eq!(
            planner.plan_rrt(&joints, &joints, &robot, &AtomicBool::new(false)),
            Ok(vec![joints])
        );
    }
}

#[test]
fn unrestricted_joints_allow_multiple_turns() {
    for (from, to) in [(0.0, 0.0), (9.0, 9.0), (-PI, PI), (-TAU, TAU)] {
        assert_valid_path(
            Constraints::new([from; 6], [to; 6], BY_PREV),
            first_joint(0.0),
            first_joint(2.0 * TAU),
        );
    }
}

#[test]
fn a_fixed_joint_can_be_sampled_while_another_joint_moves() {
    let constraints = first_joint_constraints(TAU, 0.0);
    assert_eq!(constraints.tolerances[0], 0.0);
    for fixed_angle in [0.0, TAU] {
        let start = first_joint(fixed_angle);
        let mut goal = start;
        goal[1] = 0.1;
        assert_valid_path(constraints, start, goal);
    }
}

#[test]
fn non_finite_endpoints_are_rejected_for_unrestricted_joints() {
    let robot = robot_with_constraints(Constraints::new([0.0; 6], [0.0; 6], BY_PREV));
    for angle in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let invalid = first_joint(angle);
        for (start, goal) in [(invalid, [0.0; 6]), ([0.0; 6], invalid), (invalid, invalid)] {
            for smooth in [0, 8] {
                let planner = RRTPlanner {
                    max_try: 1,
                    smooth,
                    ..RRTPlanner::default()
                };
                assert!(
                    planner
                        .plan_rrt(&start, &goal, &robot, &AtomicBool::new(false))
                        .is_err()
                );
            }
        }
    }
}

#[test]
fn cancellation_precedes_endpoint_validation() {
    let robot = robot_with_constraints(Constraints::new([-0.25; 6], [0.25; 6], BY_PREV));
    for (start, goal) in [
        ([0.0; 6], [0.1; 6]),
        (first_joint(1.0), [0.0; 6]),
        (first_joint(f64::NAN), first_joint(f64::INFINITY)),
    ] {
        for smooth in [0, 8] {
            let planner = RRTPlanner {
                max_try: 1,
                smooth,
                ..RRTPlanner::default()
            };
            assert_eq!(
                planner.plan_rrt(&start, &goal, &robot, &AtomicBool::new(true)),
                Err("Cancelled".to_string())
            );
        }
    }
}

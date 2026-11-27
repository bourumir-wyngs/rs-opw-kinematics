use crate::constraints::{BY_PREV, Constraints};
use crate::kinematic_traits::{J1, J4, J6, Kinematics};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;

// At these angles the analytic J1 root is a few ulps above the -19 degree stop.
const J1_STOP_POSE_DEGREES: [f64; 6] = [
    -19.0,
    1.0 / 3.0 - 7.0,
    1.0 / 11.0 + 15.0,
    1.0 / 7.0 - 42.0,
    1.0 / 6.0 + 25.0,
    0.25,
];

#[test]
fn inverse_selects_the_nearest_legal_turn_for_each_absolute_joint() {
    let limits = Constraints::from_degrees(
        [
            370.0..=850.0,
            17.0..=19.0,
            24.0..=26.0,
            29.0..=31.0,
            39.0..=41.0,
            -540.0..=540.0,
        ],
        BY_PREV,
    );
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), limits);
    let target = robot.forward(&[12.0, 18.0, 25.0, 30.0, 40.0, -170.0].map(f64::to_radians));
    let previous = [375.0, 18.0, 25.0, 30.0, 40.0, 535.0].map(f64::to_radians);

    for (solutions, expected_j1, expected_j6) in [
        (robot.inverse(&target), 732.0, -170.0),
        (robot.inverse_continuing(&target, &previous), 372.0, 190.0),
    ] {
        assert_eq!(solutions.len(), 1);
        let joints = solutions[0];
        assert!((joints[J1].to_degrees() - expected_j1).abs() < 1e-8);
        assert!((joints[J6].to_degrees() - expected_j6).abs() < 1e-8);
        assert!(limits.compliant(&joints));
        let pose = robot.forward(&joints);
        assert!(pose.angular_distance(target) < 1e-8);
        assert!(pose.translation.distance(target.translation) < 1e-8);
    }
}

#[test]
fn continuing_at_a_travel_stop_does_not_jump_one_turn_due_to_ik_roundoff() {
    let mut ranges = std::array::from_fn(|_| 0.0..=0.0);
    ranges[J1] = -559.0..=-19.0;
    let limits = Constraints::from_degrees(ranges, BY_PREV);
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), limits);
    let mut joints = J1_STOP_POSE_DEGREES.map(f64::to_radians);
    let mut previous = joints;
    previous[J1] -= 0.01;

    for displacement in [0.0, 1e-7] {
        joints[J1] = limits.to[J1] + displacement;
        let target = robot.forward(&joints);
        let solutions = robot.inverse_continuing(&target, &previous);
        let same_branch = solutions
            .iter()
            .find(|solution| (1..6).all(|joint| (solution[joint] - joints[joint]).abs() < 1e-8))
            .expect("the original arm and wrist branch");
        let expected = if displacement == 0.0 {
            joints[J1]
        } else {
            joints[J1] - std::f64::consts::TAU
        };
        assert!((same_branch[J1] - expected).abs() < 1e-9);
        assert!(limits.compliant(same_branch));
    }
}

#[test]
#[cfg(feature = "allow_filesystem")]
fn inverse_recovers_a_short_absolute_joint_at_its_inclusive_stop() {
    let mut from = [0.0; 6];
    let mut to = [0.0; 6];
    from[J1] = -20.0_f64.to_radians();
    to[J1] = -19.0_f64.to_radians();
    let limits = Constraints::from_urdf_bounds(from, to, BY_PREV);
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), limits);
    let joints = J1_STOP_POSE_DEGREES.map(f64::to_radians);
    let target = robot.forward(&joints);
    let solutions = robot.inverse_continuing(&target, &joints);
    assert!(solutions.iter().any(
        |q| limits.compliant(q) && (0..6).all(|joint| (q[joint] - joints[joint]).abs() < 1e-8)
    ));

    let mut outside = joints;
    outside[J1] += 1e-7;
    assert!(robot.inverse(&robot.forward(&outside)).is_empty());
}

#[test]
fn five_axis_inverse_keeps_fixed_j6_and_applies_its_literal_stops() {
    let limits = Constraints::from_degrees(
        [
            370.0..=850.0,
            17.0..=19.0,
            24.0..=26.0,
            29.0..=31.0,
            39.0..=41.0,
            -540.0..=540.0,
        ],
        BY_PREV,
    );
    let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), limits);
    let mut previous = [375.0, 18.0, 25.0, 30.0, 40.0, 540.0].map(f64::to_radians);
    let target = robot.forward(&[12.0, 18.0, 25.0, 30.0, 40.0, -170.0].map(f64::to_radians));
    for (solutions, j1) in [
        (robot.inverse_5dof(&target, previous[J6]), 732.0),
        (robot.inverse_continuing_5dof(&target, &previous), 372.0),
    ] {
        assert!(!solutions.is_empty());
        for joints in solutions {
            assert_eq!(joints[J6], previous[J6]);
            assert!((joints[J1].to_degrees() - j1).abs() < 1e-8);
            assert!(limits.compliant(&joints));
        }
    }
    previous[J6] = 550.0_f64.to_radians();
    assert!(robot.inverse_5dof(&target, previous[J6]).is_empty());
    assert!(robot.inverse_continuing_5dof(&target, &previous).is_empty());
}

#[test]
fn wrist_poles_share_the_phase_correction_within_physical_stops() {
    for (j5, sign) in [(0.0, 1.0), (180.0, -1.0)] {
        let limits = Constraints::from_degrees(
            [
                11.0..=13.0,
                17.0..=19.0,
                24.0..=26.0,
                0.0..=540.0,
                0.0..=0.0,
                -80.0..=80.0,
            ],
            BY_PREV,
        );
        let robot = OPWKinematics::new_with_constraints(Parameters::irb2400_10(), limits);
        let previous = [12.0, 18.0, 25.0, 535.0, j5, 0.0].map(f64::to_radians);
        let witness = [12.0, 18.0, 25.0, 540.0, j5, sign * 35.0].map(f64::to_radians);
        let target = robot.forward(&witness);
        let solutions = robot.inverse_continuing(&target, &previous);
        assert!(!solutions.is_empty());
        let best = solutions[0];
        let travel = (best[J4] - previous[J4]).abs() + (best[J6] - previous[J6]).abs();
        assert!((travel.to_degrees() - 40.0).abs() < 1e-7, "{best:?}");
        assert!(solutions.iter().all(|joints| limits.compliant(joints)));
        let actual = robot.forward(&best);
        assert!(actual.angular_distance(target) < 1e-8);
        assert!(actual.translation.distance(target.translation) < 1e-8);
    }
}

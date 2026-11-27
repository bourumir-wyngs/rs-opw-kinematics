//! Joint limit support

use crate::kinematic_traits::{JOINTS_AT_ZERO, Joints};
use crate::kinematics_impl::normalize_near;
use crate::utils::deg;
use rand::{Rng, RngExt};
use std::f64::consts::PI;
use std::ops::RangeInclusive;

/// Joint limits with inclusive comparisons and no floating-point tolerance.
///
/// An ordered interval wider than one full revolution defines absolute travel
/// in the robot's joint coordinates. An angle must lie between the two supplied
/// endpoints, including its whole turns. Shorter intervals repeat every turn.
/// With [`Constraints::new`], exactly one full revolution or two equal endpoints
/// permit every turn. The URDF chain reader preserves an explicitly bounded
/// one-revolution interval as absolute travel.
/// Reversed endpoints describe a periodic arc through zero.
///
/// Supply `0.0` directly for a stop at zero. For example, an ordered range
/// from zero to two full turns has an absolute lower stop of exactly zero.
/// With [`Constraints::new`], raising the upper endpoint of `[0.0, TAU]`
/// to `TAU.next_up()` selects absolute travel and allows one representable
/// value beyond a full turn.
///
/// ```
/// use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
/// use std::f64::consts::TAU;
///
/// let limits = Constraints::new([0.0; 6], [TAU.next_up(); 6], BY_PREV);
/// assert!(limits.compliant(&[0.0; 6]));
/// assert!(!limits.compliant(&[-0.01; 6]));
/// assert!(limits.compliant(&[TAU.next_up(); 6]));
/// ```
///
/// Exact boundary values cannot be checked reliably after floating-point
/// calculations: degree/radian conversions or forward/inverse kinematics can
/// round an angle just outside its limit, causing an otherwise compliant pose
/// to be rejected. Allow a small margin when reliable acceptance is required.
#[derive(Clone, Debug, Copy)]
pub struct Constraints {
    /// Lower endpoint in radians. Reversed endpoints specify a periodic arc.
    pub from: [f64; 6],

    /// Upper endpoint in radians. Ordered ranges wider than one turn bound absolute travel.
    pub to: [f64; 6],

    // Constraint centers. Used in distance from constraints based sorting.
    pub centers: [f64; 6],

    // How far from the center the value could be
    pub tolerances: [f64; 6],

    /// Used when sorting the solutions by both middle values of the constraints and the previous
    /// values of the joints. 1.0 gives the absolute priority to constraints, 0.0. gives
    /// absolute priority for joints.
    pub sorting_weight: f64,
}

/// When sorting solutions, give absolute priority to constraints (the closer to the midrange
/// of constraints, the better)
pub const BY_CONSTRAINS: f64 = 1.0;
/// When sorting solutions, give absolute priority to previous joints values (the closer to the
/// previous value, the better).
pub const BY_PREV: f64 = 0.0;

const TWO_PI: f64 = 2.0 * PI;

impl Constraints {
    /// Creates joint limits from `from` and `to` angles in radians.
    /// For example, `0.1..=0.2` describes a short periodic arc, and reversing
    /// its endpoints selects the arc through zero. The ordered interval from
    /// `-3π` to `3π` defines absolute travel stops at those two angles.
    /// The sorting_weight parameter influences sorting of the
    /// results: 0.0 (or BY_PREV) gives absolute priority to the previous values of the joints,
    /// 1.0 (or BY_CONSTRAINTS) gives absolute priority to the middle values of constraints.
    /// Intermediate values like 0.5 provide the weighted compromise.
    pub fn new(from: Joints, to: Joints, sorting_weight: f64) -> Self {
        let (centers, tolerances) = Self::compute_centers(from, to);

        Constraints {
            from,
            to,
            centers,
            tolerances,
            sorting_weight,
        }
    }

    /// Treat an explicitly bounded one-turn interval as physical travel stops.
    /// Equal endpoints still denote unrestricted rotation for unbounded joints.
    #[cfg(feature = "allow_filesystem")]
    pub(crate) fn with_bounded_full_turns(mut self) -> Self {
        for half_width in &mut self.tolerances {
            if *half_width == PI {
                // The public Constraints layout uses a cached half-width > π to
                // select absolute travel. Only mark the cache: all absolute
                // comparisons and IK turn selection use the exact from/to.
                *half_width = PI.next_up();
            }
        }
        self
    }

    /// Initializes `Constraints` from an array of 6 ranges (`RangeInclusive<f64>`),
    /// where each range specifies the `from` (start) and `to` (end) values for each joint.
    /// This is convenience method, where VALUES MUST BE GIVEN IN DEGREES.
    ///
    /// # Parameters
    /// - `ranges`: An array of 6 `RangeInclusive<f64>` values, each representing a range for one joint.
    ///   - The start of each range is taken as the `from` bound.
    ///   - The end of each range is taken as the `to` bound.
    /// - `sorting_weight`: A `f64` value representing the sorting weight for the constraint.
    ///
    /// # Returns
    /// A new instance of `Constraints` with `from`, `to`, `centers`, and `tolerances` calculated
    /// based on the specified ranges.
    ///
    /// # Example
    /// ```
    /// use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
    /// let constraints = Constraints::from_degrees(
    ///   [0.0..=90.0, 45.0..=135.0, -90.0..=90.0, 0.0..=180.0, -45.0..=45.0, -180.0..=180.0],
    ///   BY_PREV);
    /// ```
    pub fn from_degrees(ranges: [RangeInclusive<f64>; 6], sorting_weight: f64) -> Self {
        let from = ranges.each_ref().map(|range| range.start().to_radians());
        let to = ranges.each_ref().map(|range| range.end().to_radians());
        let mut constraints = Self::new(from, to, sorting_weight);
        for (joint, range) in ranges.iter().enumerate() {
            let (lower, upper) = (*range.start(), *range.end());
            if lower.is_finite() && upper.is_finite() && lower < upper {
                // Converting each endpoint separately can move an exactly
                // 360-degree interval to either side of the one-turn boundary.
                let half_width = upper.midpoint(-lower);
                if half_width >= 180.0 || constraints.tolerances[joint] >= PI {
                    constraints.tolerances[joint] = half_width.to_radians();
                }
            }
        }
        constraints
    }

    fn compute_centers(from: Joints, to: Joints) -> (Joints, Joints) {
        let mut centers: Joints = JOINTS_AT_ZERO;
        let mut tolerances: Joints = JOINTS_AT_ZERO;

        for j_idx in 0..6 {
            let a = from[j_idx];
            let b = to[j_idx];
            if !a.is_finite() || !b.is_finite() {
                // `Constraints::new` predates fallible construction. Keep invalid direct input
                // from hanging and make it reject every angle instead.
                centers[j_idx] = f64::NAN;
                tolerances[j_idx] = f64::NAN;
            } else if a == b {
                tolerances[j_idx] = f64::INFINITY; // No constraint, not checked
            } else if a < b {
                centers[j_idx] = a.midpoint(b);
                tolerances[j_idx] = b.midpoint(-a);
            } else {
                // Values wrap around. Compute the forward arc in constant time: repeatedly
                // adding TWO_PI can never finish for infinities or sufficiently large values.
                let span = (b.rem_euclid(TWO_PI) - a.rem_euclid(TWO_PI)).rem_euclid(TWO_PI);
                centers[j_idx] = a + span / 2.0;
                tolerances[j_idx] = span / 2.0;
            }
        }
        (centers, tolerances)
    }

    pub fn update_range(&mut self, from: Joints, to: Joints) {
        let (centers, tolerances) = Self::compute_centers(from, to);

        self.from = from;
        self.to = to;
        self.centers = centers;
        self.tolerances = tolerances;
        // This method does not change the sorting weight.
    }

    fn inside_bounds(angle1: f64, angle2: f64, tolerance: f64) -> bool {
        if tolerance.is_infinite() {
            return true;
        }
        let mut difference = (angle1 - angle2).abs();
        difference %= TWO_PI;
        if difference > PI {
            difference = TWO_PI - difference;
        }
        difference <= tolerance
    }

    /// Whether this joint has two absolute travel stops, including explicitly bounded full turns.
    pub(crate) fn is_absolute(&self, joint: usize) -> bool {
        let half_width = self.tolerances[joint];
        half_width.is_finite() && half_width > PI
    }

    /// Checks one joint in the robot's joint coordinates, including its whole turns.
    pub(crate) fn joint_compliant(&self, joint: usize, angle: f64) -> bool {
        if self.is_absolute(joint) {
            self.from[joint] <= angle && angle <= self.to[joint]
        } else {
            Self::inside_bounds(angle, self.centers[joint], self.tolerances[joint])
        }
    }

    /// Selects the equivalent of `angle` nearest `reference` allowed on `joint`.
    /// Both angles are in robot coordinates and in radians. A forbidden periodic
    /// orientation or non-finite input has no valid turn.
    pub(crate) fn nearest_valid_turn(
        &self,
        joint: usize,
        angle: f64,
        reference: f64,
    ) -> Option<f64> {
        if !angle.is_finite() || !reference.is_finite() {
            return None;
        }
        if !self.is_absolute(joint) {
            let mut candidate = angle;
            normalize_near(&mut candidate, reference);
            return (candidate.is_finite() && self.joint_compliant(joint, candidate))
                .then_some(candidate);
        }

        let lower = self.from[joint];
        let upper = self.to[joint];
        let target = reference.max(lower).min(upper);
        if (angle - target).abs() < PI && self.joint_compliant(joint, angle) {
            return Some(angle);
        }
        // Find the nearest turn to the bounded reference. If it crosses a stop,
        // the adjacent turn fits because the interval spans at least a full turn.
        let phase = angle % TWO_PI;
        // At a half-turn tie, prefer the lower angle, as normalize_near does.
        let turns = (((target - phase) / TWO_PI) - 0.5).ceil();
        let mut candidate = turns.mul_add(TWO_PI, phase);
        if candidate < lower {
            candidate = (turns + 1.0).mul_add(TWO_PI, phase);
        } else if candidate > upper {
            candidate = (turns - 1.0).mul_add(TWO_PI, phase);
        }
        self.joint_compliant(joint, candidate).then_some(candidate)
    }

    /// Checks whether all six joint angles satisfy the configured limits.
    pub fn compliant(&self, angles: &[f64; 6]) -> bool {
        angles
            .iter()
            .enumerate()
            .all(|(i, &angle)| self.joint_compliant(i, angle))
    }

    /// Return new vector of angle arrays, removing all that have members not satisfying these
    /// constraints.
    pub fn filter(&self, angles: &[[f64; 6]]) -> Vec<[f64; 6]> {
        angles
            .iter()
            .filter(|angle_array| self.compliant(angle_array))
            .cloned()
            .collect()
    }

    pub fn to_yaml(&self) -> String {
        format!(
            "constraints:\n  \
               from: [{}]\n  \
               to: [{}]\n",
            self.from.iter().map(deg).collect::<Vec<_>>().join(", "),
            self.to.iter().map(deg).collect::<Vec<_>>().join(", ")
        )
    }

    /// Generate a random valid angle within the defined constraints for each joint.
    pub fn random_angles(&self) -> Joints {
        let mut rng = rand::rng();

        fn random_angle(rng: &mut impl Rng, from: f64, to: f64) -> f64 {
            if from < to {
                if (to - from).is_infinite() {
                    // Uniform sampling uses the span, which can exceed f64::MAX.
                    return 2.0 * rng.random_range(from / 2.0..to / 2.0);
                }
                rng.random_range(from..to)
            } else {
                let range_length = TWO_PI - (from - to);
                let segment = rng.random_range(0.0..range_length);
                if segment < (TWO_PI - from) {
                    from + segment
                } else {
                    segment - (TWO_PI - from)
                }
            }
        }

        [
            random_angle(&mut rng, self.from[0], self.to[0]),
            random_angle(&mut rng, self.from[1], self.to[1]),
            random_angle(&mut rng, self.from[2], self.to[2]),
            random_angle(&mut rng, self.from[3], self.to[3]),
            random_angle(&mut rng, self.from[4], self.to[4]),
            random_angle(&mut rng, self.from[5], self.to[5]),
        ]
    }
}

impl Default for Constraints {
    fn default() -> Self {
        Constraints {
            from: [0.0; 6],
            to: [TWO_PI; 6],
            centers: [PI; 6],
            tolerances: [PI; 6],
            sorting_weight: BY_PREV,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematic_traits::Solutions;
    use crate::utils::as_radians;
    use std::sync::mpsc;
    use std::time::Duration;

    #[test]
    fn test_invalid_or_huge_wrapping_bounds_do_not_hang() {
        let (sender, receiver) = mpsc::channel();

        let worker = std::thread::spawn(move || {
            let mut non_finite_to = [1.0; 6];
            non_finite_to[0] = f64::NEG_INFINITY;
            let non_finite = Constraints::new([0.0; 6], non_finite_to, BY_CONSTRAINS);

            let mut huge_from = [0.0; 6];
            huge_from[0] = 1.0e20;
            let huge = Constraints::new(huge_from, [0.0; 6], BY_CONSTRAINS);

            sender
                .send((
                    non_finite.compliant(&[0.0; 6]),
                    huge.centers[0].is_finite(),
                    huge.tolerances[0].is_finite(),
                ))
                .expect("test receiver should still be available");
        });

        let (non_finite_is_compliant, huge_center_is_finite, huge_tolerance_is_finite) = receiver
            .recv_timeout(Duration::from_secs(2))
            .expect("constraint construction must not hang on invalid or huge bounds");
        worker.join().expect("constraint worker should not panic");

        assert!(
            !non_finite_is_compliant,
            "non-finite bounds must fail closed"
        );
        assert!(huge_center_is_finite);
        assert!(huge_tolerance_is_finite);
    }

    #[test]
    fn test_historical_failure_1() {
        let from = as_radians([9, 18, 28, 38, -5, 55]);
        let angles = as_radians([10, 20, 30, 40, 0, 60]);
        let to = as_radians([11, 22, 33, 44, 5, 65]);

        let limits = Constraints::new(from, to, BY_CONSTRAINS);

        let sols: Solutions = vec![angles];
        assert_eq!(limits.filter(&sols).len(), 1);

        assert!(limits.compliant(&angles));
    }

    #[test]
    fn test_no_wrap_around() {
        let angles = [0.1 * PI, 0.2 * PI, 0.3 * PI, 0.4 * PI, 0.5 * PI, 0.6 * PI];
        let from = [0.0, 0.15 * PI, 0.25 * PI, 0.35 * PI, 0.45 * PI, 0.55 * PI];
        let to = [0.2 * PI, 0.3 * PI, 0.4 * PI, 0.5 * PI, 0.6 * PI, 0.7 * PI];
        let limits = Constraints::new(from, to, BY_CONSTRAINS);
        assert!(limits.compliant(&angles));
    }

    #[test]
    fn test_with_wrap_around() {
        let angles = [
            0.9 * PI,
            1.9 * PI,
            0.05 * PI,
            1.05 * PI,
            1.95 * PI,
            0.95 * PI,
        ];
        let from = [0.8 * PI, 1.8 * PI, 0.0, 1.0 * PI, 1.9 * PI, 0.9 * PI];
        let to = [0.1 * PI, 1.1 * PI, 0.2 * PI, 1.2 * PI, 0.0, 1.0 * PI];
        let limits = Constraints::new(from, to, BY_CONSTRAINS);
        assert!(limits.compliant(&angles));
    }

    #[test]
    fn test_full_circle() {
        let angles = [0.0, 1.0 * PI, 0.5 * PI, 1.5 * PI, 0.25 * PI, 0.75 * PI];
        let from = [0.0; 6];
        let to = [2.0 * PI; 6];
        let limits = Constraints::new(from, to, BY_CONSTRAINS);
        assert!(limits.compliant(&angles));
    }

    #[test]
    fn absolute_stops_check_the_unwrapped_joint_and_select_the_nearest_legal_turn() {
        let limits = Constraints::from_degrees(
            [
                0.0..=0.0,
                0.0..=0.0,
                0.0..=0.0,
                0.0..=0.0,
                0.0..=0.0,
                -540.0..=540.0,
            ],
            BY_PREV,
        );
        let angle = |degrees: f64| [0.0, 0.0, 0.0, 0.0, 0.0, degrees.to_radians()];
        assert!(limits.compliant(&angle(-540.0)));
        assert!(limits.compliant(&angle(540.0)));
        assert!(!limits.compliant(&angle(550.0)));
        assert!(!limits.compliant(&angle(-550.0)));
        for (q, reference, expected) in [
            (-170.0_f64, 535.0_f64, 190.0_f64),
            (170.0, -535.0, -190.0),
            (-180.0, 535.0, 540.0),
            (180.0, -535.0, -540.0),
            (10.0, 90_000.0, 370.0),
            (10.0, -90_000.0, -350.0),
            (-180.0, 0.0, -180.0),
            (180.0, 0.0, -180.0),
        ] {
            let actual = limits
                .nearest_valid_turn(5, q.to_radians(), reference.to_radians())
                .unwrap();
            assert!((actual.to_degrees() - expected).abs() < 1e-10);
        }
    }

    #[test]
    fn periodic_and_absolute_limits_can_coexist_and_change_mode_on_update() {
        let mut limits = Constraints::from_degrees(
            [
                720.0..=1110.0,
                350.0..=370.0,
                170.0..=-170.0,
                10.0..=10.0,
                60.0..=420.0,
                90_000.0..=90_360.0,
            ],
            BY_PREV,
        );
        assert!((1..6).all(|joint| !limits.is_absolute(joint)));
        assert!(limits.is_absolute(0));
        assert!(limits.compliant(&[725.0, 5.0, 540.0, 800.0, -800.0, -800.0].map(f64::to_radians)));
        assert!(!limits.joint_compliant(0, 5.0_f64.to_radians()));
        let lifted = limits
            .nearest_valid_turn(0, 5.0_f64.to_radians(), 0.0)
            .unwrap();
        assert!((lifted.to_degrees() - 725.0).abs() < 1e-10);
        assert!(
            limits
                .nearest_valid_turn(1, 150.0_f64.to_radians(), 0.0)
                .is_none()
        );
        assert!(limits.nearest_valid_turn(2, 0.0, 0.0).is_none());

        limits.update_range([0.0; 6], [TWO_PI; 6]);
        assert!(!limits.is_absolute(0));
        assert!(limits.compliant(&[500.0; 6]));
        limits.update_range([0.0; 6], [TWO_PI.next_up(); 6]);
        assert!(limits.is_absolute(0));
        assert!(!limits.compliant(&[500.0; 6]));

        let ranges = std::array::from_fn(|_| 60.0..=420.0_f64.next_up());
        let limits = Constraints::from_degrees(ranges, BY_PREV);
        assert!(limits.is_absolute(0));
        assert!(!limits.compliant(&[0.0; 6]));
    }

    #[test]
    fn large_finite_ranges_retain_their_absolute_bounds() {
        for (lower, upper) in [(-f64::MAX, f64::MAX), (1.0e308, 1.7e308)] {
            let limits = Constraints::new([lower; 6], [upper; 6], BY_PREV);
            assert!(limits.is_absolute(0));
            assert!(limits.centers[0].is_finite());
            assert!(limits.compliant(&[lower; 6]));
            assert!(limits.compliant(&[upper; 6]));
            assert!(!limits.compliant(&[f64::INFINITY; 6]));
            assert!(limits.compliant(&limits.random_angles()));
        }
    }

    #[test]
    fn test_invalid_angles_no_wrap_around() {
        let angles = [
            0.15 * PI,
            0.25 * PI,
            0.55 * PI,
            0.65 * PI,
            0.75 * PI,
            0.85 * PI,
        ];
        let from = [0.2 * PI, 0.3 * PI, 0.6 * PI, 0.7 * PI, 0.8 * PI, 0.9 * PI];
        let to = [0.1 * PI, 0.2 * PI, 0.5 * PI, 0.6 * PI, 0.7 * PI, 0.8 * PI];
        let limits = Constraints::new(from, to, BY_CONSTRAINS);
        assert!(!limits.compliant(&angles));
    }

    #[test]
    fn test_invalid_angles_with_wrap_around() {
        let angles = [0.8 * PI, 1.8 * PI, 1.0 * PI, 0.0, 2.1 * PI, 1.1 * PI];
        let from = [0.9 * PI, 2.0 * PI, 0.1 * PI, 0.2 * PI, 2.2 * PI, 1.2 * PI];
        let to = [0.0, 1.0 * PI, 0.05 * PI, 0.1 * PI, 2.0 * PI, 1.0 * PI];
        let limits = Constraints::new(from, to, BY_CONSTRAINS);
        assert!(!limits.compliant(&angles));
    }

    #[test]
    fn test_filter_angles() {
        let from = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let to = [PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0];
        let angles = vec![
            [PI / 3.0, PI / 4.0, PI / 6.0, PI / 3.0, PI / 4.0, PI / 6.0], // Should be retained
            [PI, 2.0 * PI, PI, PI, PI, PI],                               // Should be removed
        ];

        let limits = Constraints::new(from, to, BY_CONSTRAINS);
        let filtered_angles = limits.filter(&angles);
        assert_eq!(filtered_angles.len(), 1);
        assert_eq!(
            filtered_angles[0],
            [PI / 3.0, PI / 4.0, PI / 6.0, PI / 3.0, PI / 4.0, PI / 6.0]
        );
    }

    #[test]
    fn test_random_angles_compliance_non_wrapping() {
        // Define non-wrapping constraints
        let from = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let to = [PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0];
        let constraints = Constraints::new(from, to, BY_CONSTRAINS);

        let total_samples = 360;
        for _ in 0..total_samples {
            let random_angles = constraints.random_angles();
            assert!(constraints.compliant(&random_angles));
        }
    }

    #[test]
    fn test_random_angles_compliance_wrapping() {
        // Define wrapping constraints
        let from = [PI / 2.0, 0.0, -PI / 2.0, 0.0, -PI, -PI];
        let to = [0.0, PI / 2.0, PI / 2.0, PI, PI / 2.0, 0.0];
        let constraints = Constraints::new(from, to, BY_CONSTRAINS);

        let total_samples = 360;
        for _ in 0..total_samples {
            let random_angles = constraints.random_angles();
            assert!(constraints.compliant(&random_angles));
        }
    }
}

//! Joint limit support

use std::f64::consts::PI;
use std::f64::INFINITY;
use std::ops::RangeInclusive;
use rand::Rng;
use crate::kinematic_traits::{Joints, JOINTS_AT_ZERO};
use crate::utils::deg;

#[derive(Clone, Debug, Copy)]
pub struct Constraints {
    /// Normalized lower limit. If more than upper limit, the range wraps-around through 0
    pub from: [f64; 6],

    /// Normalized upper limit. If less than lower limit, the range wraps-around through 0
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
    /// Create constraints that restrict the joint rotations between 'from' to 'to' values.
    /// Wrapping arround is supported so order is important. For instance,
    /// from = 0.1 and to = 0.2 (radians) means the joint
    /// is allowed to rotate to 0.11, 0.12 ... 1.99, 0.2. 
    /// from = 0.2 ant to = 0.1 is also valid but means the joint is allowed to rotate
    /// to 0.21, 0.22, 0.99, 2 * PI or 0.0 (wrapping around), then to 0.09 and finally 0.1,
    /// so the other side of the circle. The sorting_weight parameter influences sorting of the
    /// results: 0.0 (or BY_PREV) gives absolute priority to the previous values of the joints,
    /// 1.0 (or BY_CONSTRAINTS) gives absolute priority to the middle values of constraints.
    /// Intermediate values like 0.5 provide the weighted compromise.
    pub fn new(from: Joints, to: Joints, sorting_weight: f64) -> Self {
        let (centers, tolerances) = Self::compute_centers(from, to);

        Constraints {
            from: from,
            to: to,
            centers: centers,
            tolerances: tolerances,
            sorting_weight: sorting_weight,
        }
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
        let from: Joints = [
            ranges[0].start().to_radians(),
            ranges[1].start().to_radians(),
            ranges[2].start().to_radians(),
            ranges[3].start().to_radians(),
            ranges[4].start().to_radians(),
            ranges[5].start().to_radians(),
        ];

        let to: Joints = [
            ranges[0].end().to_radians(),
            ranges[1].end().to_radians(),
            ranges[2].end().to_radians(),
            ranges[3].end().to_radians(),
            ranges[4].end().to_radians(),
            ranges[5].end().to_radians(),
        ];

        let (centers, tolerances) = Self::compute_centers(from, to);

        Constraints {
            from,
            to,
            centers,
            tolerances,
            sorting_weight,
        }
    }

    fn compute_centers(from: Joints, to: Joints) -> (Joints, Joints) {
        let mut centers: Joints = JOINTS_AT_ZERO;
        let mut tolerances: Joints = JOINTS_AT_ZERO;

        for j_idx in 0..6 {
            let a = from[j_idx];
            let mut b = to[j_idx];
            if a == b {
                tolerances[j_idx] = INFINITY; // No constraint, not checked
            } else if a < b {
                // Values do not wrap arround
                centers[j_idx] = (a + b) / 2.0;
                tolerances[j_idx] = (b - a) / 2.0;
            } else {
                // Values wrap arround. Move b forward by period till it gets ahead.
                while b < a {
                    b = b + TWO_PI;
                }
                centers[j_idx] = (a + b) / 2.0;
                tolerances[j_idx] = (b - a) / 2.0;
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
            return false;
        }
        let mut difference = (angle1 - angle2).abs();
        difference = difference % TWO_PI;
        if difference > PI {
            difference = TWO_PI - difference;
        }
        difference <= tolerance
    }

    /// Checks if all values in the given vector or angles satisfy these constraints.
    #[must_use = "Ignoring compliance check result may cause constraint violations."]
    pub fn compliant(&self, angles: &[f64; 6]) -> bool {
        let ok = angles.iter().enumerate().all(|(i, &angle)| {
            // '!' is used to negate the condition from 'out_of_bounds' directly in the 'all' call.
            Self::inside_bounds(angle, self.centers[i], self.tolerances[i])
        });
        ok
    }

    /// Return new vector of angle arrays, removing all that have members not satisfying these
    /// constraints.
    #[must_use = "Ignoring filtered results means no constraints are actually applied."]
    pub fn filter(&self, angles: &Vec<[f64; 6]>) -> Vec<[f64; 6]> {
        angles.into_iter()
            .filter(|angle_array| self.compliant(&angle_array))
            .cloned()
            .collect()
    }


    pub fn to_yaml(&self) -> String {
        format!(
            "constraints:\n  \
               from: [{}]\n  \
               to: [{}]\n",
            self.from.iter().map(|x| deg(x))
                .collect::<Vec<_>>().join(", "),
            self.to.iter().map(|x| deg(x))
                .collect::<Vec<_>>().join(", ")
        )
    }


    /// Generate a random valid angle within the defined constraints for each joint.
    pub fn random_angles(&self) -> Joints {
        let mut rng = rand::thread_rng();

        fn random_angle(rng: &mut impl Rng, from: f64, to: f64) -> f64 {
            if from < to {
                rng.gen_range(from..to)
            } else {
                let range_length = TWO_PI - (from - to);
                let segment = rng.gen_range(0.0..range_length);
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
    use crate::kinematic_traits::Solutions;
    use crate::utils::{as_radians};
    use super::*;

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
        let angles = [0.9 * PI, 1.9 * PI, 0.05 * PI, 1.05 * PI, 1.95 * PI, 0.95 * PI];
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
    fn test_invalid_angles_no_wrap_around() {
        let angles = [0.15 * PI, 0.25 * PI, 0.55 * PI, 0.65 * PI, 0.75 * PI, 0.85 * PI];
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
            [PI, 2.0 * PI, PI, PI, PI, PI], // Should be removed
        ];

        let limits = Constraints::new(from, to, BY_CONSTRAINS);
        let filtered_angles = limits.filter(&angles);
        assert_eq!(filtered_angles.len(), 1);
        assert_eq!(filtered_angles[0], [PI / 3.0, PI / 4.0, PI / 6.0, PI / 3.0, PI / 4.0, PI / 6.0]);
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


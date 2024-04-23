use std::f64::consts::PI;

#[derive(Clone)]
pub struct Constraints {
    /// Normalized lower limit. If more than upper limit, the range wraps-around through 0
    pub from: [f64; 6],

    /// Normalized upper limit. If less than lower limit, the range wraps-around through 0
    pub to: [f64; 6],

    // Constraint centers. Used in distance from constraints based sorting.
    pub centers: [f64; 6],

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
    pub fn new(from: [f64; 6], to: [f64; 6], sorting_weight: f64) -> Self {
        let two_pi = 2.0 * PI;
        let from_normalized = from.map(|f| ((f % two_pi) + two_pi) % two_pi);
        let to_normalized = to.map(|t| ((t % two_pi) + two_pi) % two_pi);
        let centers =
            std::array::from_fn(|i| (from_normalized[i] + to_normalized[i]) / 2.0)
                .map(|t| ((t % two_pi) + two_pi) % two_pi);

        Constraints {
            from: from_normalized,
            to: to_normalized,
            centers: centers,
            sorting_weight: sorting_weight,
        }
    }

    /// Checks if all values in the given vector or angles satisfy these constraints.
    pub fn compliant(&self, angles: &[f64; 6]) -> bool {
        let two_pi = 2.0 * PI;
        for i in 0..6 {
            if self.from[i] == self.to[i] {
                continue; // Joint without constraints, from == to
            }
            let angle = ((angles[i] % two_pi) + two_pi) % two_pi;
            if self.from[i] <= self.to[i] {
                if !(angle >= self.from[i] && angle <= self.to[i]) {
                    return false;
                }
            } else {
                if !(angle >= self.from[i] || angle <= self.to[i]) {
                    return false;
                }
            }
        }
        true
    }

    /// Return new vector of angle arrays, removing all that have members not satisfying these
    /// constraints.
    pub fn filter(&self, angles: &Vec<[f64; 6]>) -> Vec<[f64; 6]> {
        angles.into_iter()
            .filter(|angle_array| self.compliant(&angle_array))
            .cloned()
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}


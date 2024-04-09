pub(crate) mod opw_kinematics {
    use crate::kinematic_traits::kinematics_traits::{Joints};

    /// Checks if all elements in the array are finite
    pub fn is_valid(qs: &Joints) -> bool {
        qs.iter().all(|&q| q.is_finite())
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use super::opw_kinematics::*;

    #[test]
    fn test_is_valid_with_all_finite() {
        let qs = [0.0, 1.0, -1.0, 0.5, -0.5, PI];
        assert!(is_valid(&qs));
    }

    #[test]
    fn test_is_valid_with_nan() {
        let qs = [0.0, f64::NAN, 1.0, -1.0, 0.5, -0.5];
        assert!(!is_valid(&qs));
    }

    #[test]
    fn test_is_valid_with_infinity() {
        let qs = [0.0, f64::INFINITY, 1.0, -1.0, 0.5, -0.5];
        assert!(!is_valid(&qs));
    }
}
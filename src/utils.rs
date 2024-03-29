mod opw_kinematics {
    use std::f64::consts::PI;

    /// Checks if all elements in the array are finite
    pub fn is_valid(qs: &[f64; 6]) -> bool {
        qs.iter().all(|&q| q.is_finite())
    }

    /// Adjusts the elements of the array to be within the range [-π, π]
    pub fn harmonize_toward_zero(qs: &mut [f64; 6]) {
        for q in qs.iter_mut() {
            while *q > PI {
                *q -= 2.0 * PI;
            }
            while *q < -PI {
                *q += 2.0 * PI;
            }
        }
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

    #[test]
    fn test_harmonize_toward_zero_within_range() {
        let mut qs = [0.0, -PI / 2.0, PI / 2.0, -0.1, 0.1, PI - 0.1];
        harmonize_toward_zero(&mut qs);
        let expected = [0.0, -PI / 2.0, PI / 2.0, -0.1, 0.1, PI - 0.1];
        assert_eq!(qs, expected);
    }

    #[test]
    fn test_harmonize_toward_zero_above_pi() {
        let mut qs = [2.0 * PI, 3.0 * PI, -3.0 * PI, 4.0 * PI, -4.0 * PI, 5.0 * PI];
        harmonize_toward_zero(&mut qs);
        let expected = [0.0, PI, -PI, 0.0, 0.0, PI];
        assert_eq!(qs, expected);
    }

    #[test]
    fn test_harmonize_toward_zero_below_minus_pi() {
        let mut qs = [-2.0 * PI, -3.0 * PI, 3.0 * PI, -4.0 * PI, 4.0 * PI, -5.0 * PI];
        harmonize_toward_zero(&mut qs);
        let expected = [0.0, -PI, PI, 0.0, 0.0, -PI];
        assert_eq!(qs, expected);
    }
}
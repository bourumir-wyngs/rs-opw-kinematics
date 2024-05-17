//! Helper functions

use crate::kinematic_traits::{Joints, Solutions};

/// Checks the solution for validity. This is only internally needed as all returned
/// solutions are already checked.
pub(crate) mod opw_kinematics {
    use crate::kinematic_traits::{Joints};

    /// Checks if all elements in the array are finite
    pub fn is_valid(qs: &Joints) -> bool {
        qs.iter().all(|&q| q.is_finite())
    }
}

/// Print joint values for all solutions, converting radianst to degrees.
#[allow(dead_code)]
pub fn dump_solutions(solutions: &Solutions) {
    if solutions.is_empty() {
        println!("No solutions");
    }
    for sol_idx in 0..solutions.len() {
        let mut row_str = String::new();
        for joint_idx in 0..6 {
            let computed = solutions[sol_idx][joint_idx];
            row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
        }
        println!("[{}]", row_str.trim_end());
    }
}

#[allow(dead_code)]
pub fn dump_solutions_degrees(solutions: &Solutions) {
    if solutions.is_empty() {
        println!("No solutions");
    }
    for sol_idx in 0..solutions.len() {
        let mut row_str = String::new();
        for joint_idx in 0..6 {
            let computed = solutions[sol_idx][joint_idx];
            row_str.push_str(&format!("{:5.2} ", computed));
        }
        println!("[{}]", row_str.trim_end());
    }
}

/// Print joint values, converting radianst to degrees.
#[allow(dead_code)]
pub fn dump_joints(joints: &Joints) {
    let mut row_str = String::new();
    for joint_idx in 0..6 {
        let computed = joints[joint_idx];
        row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
    }
    println!("[{}]", row_str.trim_end());
}

/// Allows to specify joint values in degrees (converts to radians)
#[allow(dead_code)]
pub fn as_radians(degrees: [i32; 6]) -> Joints {
    std::array::from_fn(|i| (degrees[i] as f64).to_radians())
}

/// formatting for YAML output
pub(crate) fn deg(x: &f64) -> String {
    if *x == 0.0 {
        return "0".to_string();
    }
    format!("deg({:.4})", x.to_degrees())
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
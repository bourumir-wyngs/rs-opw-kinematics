//! Helper functions

use nalgebra::Vector6;
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

/// Convert array of f32's in degrees to Joints 
/// that are array of f64's in radians
pub fn joints(angles: &[f32; 6]) -> Joints {
    [
        (angles[0] as f64).to_radians(),
        (angles[1] as f64).to_radians(),
        (angles[2] as f64).to_radians(),
        (angles[3] as f64).to_radians(),
        (angles[4] as f64).to_radians(),
        (angles[5] as f64).to_radians()
    ]
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

/// Converts ```nalgebra::Vector6<f64>``` to Joints ([f64; 6])
pub fn vector6_to_joints(v: Vector6<f64>) -> Joints {
    [v[0], v[1], v[2], v[3], v[4], v[5]]
}

/// Converts ```Joints ([f64; 6])``` to a ```Vector6<f64>```
pub fn joints_to_vector6(j: Joints) -> nalgebra::Vector6<f64> {
    Vector6::new(j[0], j[1], j[2], j[3], j[4], j[5])
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
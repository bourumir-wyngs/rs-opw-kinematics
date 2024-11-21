//! Helper functions

use crate::kinematic_traits::{Joints, Solutions};
use nalgebra::{Isometry3, UnitQuaternion, Vector6};

/// Checks the solution for validity. This is only internally needed as all returned
/// solutions are already checked.
pub(crate) mod opw_kinematics {
    use crate::kinematic_traits::Joints;

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
        (angles[5] as f64).to_radians(),
    ]
}

/// Convert joints that are array of f64's in radians to
/// array of f32's in degrees
pub fn to_degrees(angles: &Joints) -> [f32; 6] {
    [
        angles[0].to_degrees() as f32,
        angles[1].to_degrees() as f32,
        angles[2].to_degrees() as f32,
        angles[3].to_degrees() as f32,
        angles[4].to_degrees() as f32,
        angles[5].to_degrees() as f32,
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

pub fn dump_pose(isometry: &Isometry3<f64>) {
    // Extract translation components
    let translation = isometry.translation.vector;

    // Extract rotation components and convert to Euler angles in radians
    let rotation: UnitQuaternion<f64> = isometry.rotation;

    // Print translation and rotation
    println!(
        "x: {:.5}, y: {:.5}, z: {:.5},  quat: {:.5},{:.5},{:.5},{:.5}",
        translation.x, translation.y, translation.z, rotation.i, rotation.j, rotation.k, rotation.w
    );
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

/// Calculates the transition cost between two sets of joint positions,
/// weighted by given coefficients (rotating heavy base joints is
/// more expensive). This function returns maximum rotation
pub fn transition_costs(from: &Joints, to: &Joints, coefficients: &Joints) -> f64 {
    [(from[0] - to[0]).abs() * coefficients[0]
        + (from[1] - to[1]).abs() * coefficients[1]
        + (from[2] - to[2]).abs() * coefficients[2]
        + (from[3] - to[3]).abs() * coefficients[3]
        + (from[4] - to[4]).abs() * coefficients[4]
        + (from[5] - to[5]).abs() * coefficients[5]]
    .iter()
    .fold(f64::NEG_INFINITY, |a, &b| a.max(b))
}

pub fn assert_pose_eq(ta: &Isometry3<f64>, tb: &Isometry3<f64>,
                 distance_tolerance: f64, angular_tolerance: f64) -> bool {
    fn bad(ta: &Isometry3<f64>, tb: &Isometry3<f64>) {
        dump_pose(ta);
        dump_pose(tb);
    }
    
    let translation_distance = (ta.translation.vector - tb.translation.vector).norm();
    let angular_distance = ta.rotation.angle_to(&tb.rotation);

    if translation_distance.abs() > distance_tolerance {
        bad(ta, tb);        
        panic!("Poses have too different translations");        
    }

    if angular_distance.abs() > angular_tolerance {
        bad(ta, tb);
        panic!("Poses have too different angles");
    }
    true
}

#[cfg(test)]
mod tests {
    use super::opw_kinematics::*;
    use std::f64::consts::PI;

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

//!
//!  This package provides support for the Jacobian matrix.
//!
//!  The Jacobian matrix, as described here, represents the relationship between the joint velocities
//!  and the end-effector velocities:
//! ```text
//!  | ∂vx/∂θ₁  ∂vx/∂θ₂  ∂vx/∂θ₃  ∂vx/∂θ₄  ∂vx/∂θ₅  ∂vx/∂θ₆ |
//!  | ∂vy/∂θ₁  ∂vy/∂θ₂  ∂vy/∂θ₃  ∂vy/∂θ₄  ∂vy/∂θ₅  ∂vy/∂θ₆ |
//!  | ∂vz/∂θ₁  ∂vz/∂θ₂  ∂vz/∂θ₃  ∂vz/∂θ₄  ∂vz/∂θ₅  ∂vz/∂θ₆ |
//!  | ∂ωx/∂θ₁  ∂ωx/∂θ₂  ∂ωx/∂θ₃  ∂ωx/∂θ₄  ∂ωx/∂θ₅  ∂ωx/∂θ₆ |
//!  | ∂ωy/∂θ₁  ∂ωy/∂θ₂  ∂ωy/∂θ₃  ∂ωy/∂θ₄  ∂ωy/∂θ₅  ∂ωy/∂θ₆ |
//!  | ∂ωz/∂θ₁  ∂ωz/∂θ₂  ∂ωz/∂θ₃  ∂ωz/∂θ₄  ∂ωz/∂θ₅  ∂ωz/∂θ₆ |
//! ```
//!  The first three rows correspond to the linear velocities: vx, vy, vz.
//!  The last three rows correspond to the angular velocities: roll (ωx), pitch (ωy), and yaw (ωz).
//!  θ₁, θ₂, θ₃, θ₄, θ₅, θ₆ are the joint angles.
//!  ∂ denotes a partial derivative.

use crate::kinematic_traits::{Joints, Kinematics};
use crate::pose::{Twist, Wrench};
use std::ops::{Index, IndexMut};

const SIZE: usize = 6;
const SINGULAR_EPS: f64 = 1.0e-15;

/// Fixed 6x6 Jacobian matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix6 {
    rows: [[f64; SIZE]; SIZE],
}

impl Matrix6 {
    /// Creates a zero matrix.
    pub const fn zeros() -> Self {
        Self {
            rows: [[0.0; SIZE]; SIZE],
        }
    }

    /// Creates a matrix from row-major data.
    pub const fn from_rows(rows: [[f64; SIZE]; SIZE]) -> Self {
        Self { rows }
    }

    /// Returns the row-major matrix data.
    pub const fn rows(&self) -> &[[f64; SIZE]; SIZE] {
        &self.rows
    }

    fn transpose(self) -> Self {
        let mut rows = [[0.0; SIZE]; SIZE];
        let mut r = 0;
        while r < SIZE {
            let mut c = 0;
            while c < SIZE {
                rows[r][c] = self.rows[c][r];
                c += 1;
            }
            r += 1;
        }
        Self { rows }
    }

    fn mul_vector(self, vector: &Joints) -> Joints {
        let mut result = [0.0; SIZE];
        for (out, row) in result.iter_mut().zip(self.rows.iter()) {
            *out = row.iter().zip(vector.iter()).map(|(left, right)| left * right).sum();
        }
        result
    }

    fn transpose_mul_vector(self, vector: &Joints) -> Joints {
        self.transpose().mul_vector(vector)
    }

    fn mul_self_transpose(self) -> Self {
        let mut rows = [[0.0; SIZE]; SIZE];
        for (r, row) in rows.iter_mut().enumerate() {
            for (c, value) in row.iter_mut().enumerate() {
                *value = self.rows[r]
                    .iter()
                    .zip(self.rows[c].iter())
                    .map(|(left, right)| left * right)
                    .sum();
            }
        }
        Self { rows }
    }

    fn try_solve(self, rhs: &Joints) -> Option<Joints> {
        solve_linear_system(self.rows, *rhs)
    }

    fn damped_least_squares_solve(self, rhs: &Joints, damping: f64) -> Option<Joints> {
        let damping = damping.abs().max(1.0e-6);
        let mut normal = self.mul_self_transpose();
        for i in 0..SIZE {
            normal[(i, i)] += damping * damping;
        }
        let y = normal.try_solve(rhs)?;
        Some(self.transpose_mul_vector(&y))
    }
}

impl Index<(usize, usize)> for Matrix6 {
    type Output = f64;

    fn index(&self, (row, col): (usize, usize)) -> &Self::Output {
        &self.rows[row][col]
    }
}

impl IndexMut<(usize, usize)> for Matrix6 {
    fn index_mut(&mut self, (row, col): (usize, usize)) -> &mut Self::Output {
        &mut self.rows[row][col]
    }
}

fn solve_linear_system(mut matrix: [[f64; SIZE]; SIZE], mut rhs: Joints) -> Option<Joints> {
    for (row, rhs_value) in matrix.iter().zip(rhs.iter()) {
        if row.iter().any(|value| !value.is_finite()) || !rhs_value.is_finite() {
            return None;
        }
    }

    for pivot_col in 0..SIZE {
        let mut pivot_row = pivot_col;
        let mut pivot_abs = matrix[pivot_col][pivot_col].abs();
        for (row, matrix_row) in matrix.iter().enumerate().skip(pivot_col + 1) {
            let value_abs = matrix_row[pivot_col].abs();
            if value_abs > pivot_abs {
                pivot_abs = value_abs;
                pivot_row = row;
            }
        }

        if pivot_abs <= SINGULAR_EPS {
            return None;
        }

        if pivot_row != pivot_col {
            matrix.swap(pivot_col, pivot_row);
            rhs.swap(pivot_col, pivot_row);
        }

        let pivot_row_values = matrix[pivot_col];
        let pivot_value = pivot_row_values[pivot_col];
        let pivot_rhs = rhs[pivot_col];
        for (row, matrix_row) in matrix.iter_mut().enumerate().skip(pivot_col + 1) {
            let factor = matrix_row[pivot_col] / pivot_value;
            matrix_row[pivot_col] = 0.0;
            for (col, value) in matrix_row.iter_mut().enumerate().skip(pivot_col + 1) {
                *value -= factor * pivot_row_values[col];
            }
            rhs[row] -= factor * pivot_rhs;
        }
    }

    let mut solution = [0.0; SIZE];
    for row in (0..SIZE).rev() {
        let mut value = rhs[row];
        for col in (row + 1)..SIZE {
            value -= matrix[row][col] * solution[col];
        }

        let diagonal = matrix[row][row];
        if diagonal.abs() <= SINGULAR_EPS {
            return None;
        }
        solution[row] = value / diagonal;
    }

    Some(solution)
}

/// This structure holds Jacobian matrix and provides methods to
/// extract velocity and torgue information from it.
///
///
///  This package provides support for the Jacobian matrix.
///
///  The Jacobian matrix, as described here, represents the relationship between the joint velocities
///  and the end-effector velocities:
/// ```text
///  | ∂vx/∂θ₁  ∂vx/∂θ₂  ∂vx/∂θ₃  ∂vx/∂θ₄  ∂vx/∂θ₅  ∂vx/∂θ₆ |
///  | ∂vy/∂θ₁  ∂vy/∂θ₂  ∂vy/∂θ₃  ∂vy/∂θ₄  ∂vy/∂θ₅  ∂vy/∂θ₆ |
///  | ∂vz/∂θ₁  ∂vz/∂θ₂  ∂vz/∂θ₃  ∂vz/∂θ₄  ∂vz/∂θ₅  ∂vz/∂θ₆ |
///  | ∂ωx/∂θ₁  ∂ωx/∂θ₂  ∂ωx/∂θ₃  ∂ωx/∂θ₄  ∂ωx/∂θ₅  ∂ωx/∂θ₆ |
///  | ∂ωy/∂θ₁  ∂ωy/∂θ₂  ∂ωy/∂θ₃  ∂ωy/∂θ₄  ∂ωy/∂θ₅  ∂ωy/∂θ₆ |
///  | ∂ωz/∂θ₁  ∂ωz/∂θ₂  ∂ωz/∂θ₃  ∂ωz/∂θ₄  ∂ωz/∂θ₅  ∂ωz/∂θ₆ |
/// ```
///  The first three rows correspond to the linear velocities: vx, vy, vz.
///  The last three rows correspond to the angular velocities: roll (ωx), pitch (ωy), and yaw (ωz).
///  θ₁, θ₂, θ₃, θ₄, θ₅, θ₆ are the joint angles.
///  ∂ denotes a partial derivative.
pub struct Jacobian {
    /// A 6x6 matrix representing the Jacobian
    ///
    /// The Jacobian matrix maps the joint velocities to the end-effector velocities.
    /// Each column corresponds to a joint, and each row corresponds to a degree of freedom
    /// of the end-effector (linear and angular velocities).    
    matrix: Matrix6,

    /// The disturbance value used for computing the Jacobian
    epsilon: f64,
}

impl Jacobian {
    /// Constructs a new Jacobian struct by computing the Jacobian matrix for the given robot and joint configuration
    ///
    /// # Arguments
    ///
    /// * `robot` - A reference to the robot implementing the Kinematics trait
    /// * `qs` - A reference to the joint configuration
    /// * `epsilon` - A small value used for numerical differentiation
    ///
    /// # Returns
    ///
    /// A new instance of `Jacobian`
    pub fn new(robot: &impl Kinematics, qs: &Joints, epsilon: f64) -> Self {
        let matrix = compute_jacobian(robot, qs, epsilon);
        Self { matrix, epsilon }
    }

    /// Returns the computed Jacobian matrix.
    pub fn matrix(&self) -> &Matrix6 {
        &self.matrix
    }

    /// Computes the joint velocities required to achieve a desired end-effector velocity:
    ///
    /// Q' = J⁻¹ x'
    ///
    /// where Q' are joint velocities, J⁻¹ is the inverted Jacobian matrix and x' is the vector
    /// of velocities of the tool center point. First 3 components are velocities along x,y and z
    /// axis, the other 3 are angular rotation velocities around x (roll), y (pitch) and z (yaw) axis
    ///
    /// # Arguments
    ///
    /// * `desired_end_effector_velocity` - Linear and angular velocity of the end-effector.
    ///
    /// # Returns
    ///
    /// `Result<Joints, &'static str>` - Joint positions, with values representing joint velocities rather than angles,
    /// or an error message if the computation fails.
    ///
    /// This method combines linear and angular velocity into a 6D vector. It then computes the
    /// joint velocities required to achieve the desired end-effector velocity using the
    /// `velocities_from_vector` method.
    pub fn velocities(
        &self,
        desired_end_effector_velocity: &Twist,
    ) -> Result<Joints, &'static str> {
        let linear_velocity = desired_end_effector_velocity.linear;
        let angular_velocity = desired_end_effector_velocity.angular;

        // Combine into a single 6D vector
        let desired_velocity = [
            linear_velocity.x,
            linear_velocity.y,
            linear_velocity.z,
            angular_velocity.x,
            angular_velocity.y,
            angular_velocity.z,
        ];

        // Compute the joint velocities from the 6D vector
        self.velocities_from_vector(&desired_velocity)
    }

    /// Computes the joint velocities required to achieve a desired end-effector velocity:
    ///
    /// Q' = J⁻¹ x'
    ///
    /// where Q' are joint velocities, J⁻¹ is the inverted Jacobian matrix and x' is the vector
    /// of velocities of the tool center point. First 3 components are velocities along x,y and z
    /// axis. The remaining 3 are angular rotation velocities are assumed to be zero.
    ///
    /// # Arguments
    ///
    /// * `vx, vy, vz` - x, y and z components of end effector velocity (linear).
    ///
    /// # Returns
    ///
    /// `Result<Joints, &'static str>` - Joint positions, with values representing
    /// joint velocities rather than angles or an error message if the computation fails.
    pub fn velocities_fixed(&self, vx: f64, vy: f64, vz: f64) -> Result<Joints, &'static str> {
        // Combine into a single 6D vector with 0 rotational part
        let desired_velocity = [vx, vy, vz, 0.0, 0.0, 0.0];

        // Compute the joint velocities from the 6D vector
        self.velocities_from_vector(&desired_velocity)
    }

    /// Computes the joint velocities required to achieve a desired end-effector velocity:
    ///
    /// Q' = J⁻¹ X'
    ///
    /// where Q' are joint velocities, J⁻¹ is the inverted Jacobian matrix and x' is the vector
    /// of velocities of the tool center point. First 3 components are velocities along x,y and z
    /// axis, the other 3 are angular rotation velocities around x (roll), y (pitch) and z (yaw) axis
    ///
    /// # Arguments
    ///
    /// * `X'` - A 6D vector representing the desired linear and angular velocity of the
    ///   end-effector as defined above. The first 3 elements are linear velocity and the
    ///   last 3 elements are angular velocity.
    ///
    /// # Returns
    ///
    /// `Result<Joints, &'static str>` - Joint positions, with values representing joint velocities rather than angles,
    /// or an error message if the computation fails.
    ///
    /// This method first tries to solve the Jacobian system directly. If the Jacobian is singular,
    /// it falls back to damped least squares.
    #[allow(non_snake_case)] // Standard Math notation calls for single uppercase name
    pub fn velocities_from_vector(&self, X: &Joints) -> Result<Joints, &'static str> {
        if let Some(joint_velocities) = self.matrix.try_solve(X) {
            return Ok(joint_velocities);
        }

        self.matrix
            .damped_least_squares_solve(X, self.epsilon)
            .ok_or("Unable to solve Jacobian system")
    }

    /// Computes the joint torques required to achieve a desired end-effector force/torque
    /// This function computes
    ///
    /// t = JᵀF
    ///
    /// where Jᵀ is transposed Jacobian as defined above and f is the desired force vector.
    ///
    /// # Arguments
    ///
    /// * `desired_force_torque` - force in Newtons and torque in Newton-meters.
    ///
    /// # Returns
    ///
    /// Joint positions, with values representing joint torques,
    /// or an error message if the computation fails.
    pub fn torques(&self, desired_force_torque: &Wrench) -> Joints {
        let linear_force = desired_force_torque.force;
        let angular_torque = desired_force_torque.torque;

        // Combine into a single 6D vector
        let desired_force_torque_vector = [
            linear_force.x,
            linear_force.y,
            linear_force.z,
            angular_torque.x,
            angular_torque.y,
            angular_torque.z,
        ];

        self.matrix
            .transpose_mul_vector(&desired_force_torque_vector)
    }

    /// Computes the joint torques required to achieve a desired end-effector force/torque
    /// This function computes
    ///
    /// t = JᵀF
    ///
    /// where Jᵀ is transposed Jacobian as defined above and f is the desired force and torgue
    /// vector. The first 3 components are forces along x, y and z in Newtons, the other 3
    /// components are rotations around x (roll), y (pitch) and z (yaw) axis in Newton meters.
    ///
    /// # Arguments
    ///
    /// * `F` - A 6D vector representing the desired force and torque at the end-effector
    ///   as explained above.
    ///
    /// # Returns
    ///
    /// Joint positions, with values representing joint torques,
    /// or an error message if the computation fails.
    #[allow(non_snake_case)] // Standard Math notation calls for single uppercase name
    pub fn torques_from_vector(&self, F: &Joints) -> Joints {
        self.matrix.transpose_mul_vector(F)
    }
}

/// Function to compute the Jacobian matrix for a given robot and joint configuration
///
/// # Arguments
///
/// * `robot` - A reference to the robot implementing the Kinematics trait
/// * `qs` - A reference to the joint configuration
/// * `epsilon` - A small value used for numerical differentiation
///
/// # Returns
///
/// A 6x6 matrix representing the Jacobian
///
/// The Jacobian matrix maps the joint velocities to the end-effector velocities.
/// Each column corresponds to a joint, and each row corresponds to a degree of freedom
/// of the end-effector (linear and angular velocities).
pub(crate) fn compute_jacobian(robot: &impl Kinematics, joints: &Joints, epsilon: f64) -> Matrix6 {
    let mut jacobian = Matrix6::zeros();
    let current_pose = robot.forward(joints);
    let current_position = current_pose.translation;
    let current_orientation = current_pose.rotation;

    // Parallelize the loop using rayon
    let jacobian_columns: Vec<_> = (0..6)
        .map(|i| {
            let mut perturbed_qs = *joints;
            perturbed_qs[i] += epsilon;
            let perturbed_pose = robot.forward(&perturbed_qs);
            let perturbed_position = perturbed_pose.translation;
            let perturbed_orientation = perturbed_pose.rotation;

            let delta_position = (perturbed_position - current_position) / epsilon;
            let delta_orientation =
                (perturbed_orientation * current_orientation.inverse()).to_scaled_axis() / epsilon;

            (delta_position, delta_orientation)
        })
        .collect();

    for (i, (delta_position, delta_orientation)) in jacobian_columns.into_iter().enumerate() {
        jacobian[(0, i)] = delta_position.x;
        jacobian[(1, i)] = delta_position.y;
        jacobian[(2, i)] = delta_position.z;
        jacobian[(3, i)] = delta_orientation.x;
        jacobian[(4, i)] = delta_orientation.y;
        jacobian[(5, i)] = delta_orientation.z;
    }

    jacobian
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraints::Constraints;
    use crate::kinematic_traits::{Pose, Singularity, Solutions};
    use glam::{DQuat, DVec3};

    const EPSILON: f64 = 1e-6;

    /// Example implementation of the Kinematics trait for a single rotary joint robot
    /// When the first joint rotates, it affects the Y-position and the Z-orientation of the end-effector.
    /// The derivative of the Y-position with respect to the first joint should be 1.
    /// The derivative of the Z-orientation with respect to the first joint should be 1.
    /// No other joint affects the end-effector in this simple robot model.
    pub struct SingleRotaryJointRobot;

    impl Kinematics for SingleRotaryJointRobot {
        fn inverse(&self, _pose: &Pose) -> Solutions {
            panic!() // Not used in this test
        }

        fn inverse_5dof(&self, _pose: &Pose, _j6: f64) -> Solutions {
            panic!() // Not used in this test
        }

        fn inverse_continuing_5dof(&self, _pose: &Pose, _prev: &Joints) -> Solutions {
            panic!() // Not used in this test
        }

        fn forward_with_joint_poses(&self, _joints: &Joints) -> [Pose; 6] {
            panic!() // Not used in this test
        }

        /// Simple inverse kinematics for a single rotary joint of the length 1.
        fn inverse_continuing(&self, pose: &Pose, _previous: &Joints) -> Solutions {
            let angle = pose.translation.y.atan2(pose.translation.x);
            vec![[angle, 0.0, 0.0, 0.0, 0.0, 0.0]]
        }

        fn forward(&self, qs: &Joints) -> Pose {
            // Forward kinematics for a single rotary joint robot
            let angle = qs[0];
            Pose::from_parts(
                DVec3::new(angle.cos(), angle.sin(), 0.0),
                DQuat::from_rotation_z(angle),
            )
        }

        fn kinematic_singularity(&self, _qs: &Joints) -> Option<Singularity> {
            None
        }

        fn constraints(&self) -> &Option<Constraints> {
            &None
        }
    }

    fn assert_matrix_approx_eq(left: &Matrix6, right: &Matrix6, epsilon: f64) {
        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (left[(i, j)] - right[(i, j)]).abs() < epsilon,
                    "left[{0},{1}] = {2} is not approximately equal to right[{0},{1}] = {3}",
                    i,
                    j,
                    left[(i, j)],
                    right[(i, j)]
                );
            }
        }
    }

    #[test]
    fn test_forward_kinematics() {
        let robot = SingleRotaryJointRobot;
        let joints: Joints = [std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0];
        let pose = robot.forward(&joints);
        assert!((pose.translation.x - 0.0).abs() < EPSILON);
        assert!((pose.translation.y - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_inverse_kinematics() {
        let robot = SingleRotaryJointRobot;
        let pose = Pose::from_translation(DVec3::new(0.0, 1.0, 0.0));
        let previous: Joints = [0.0; 6];
        let solutions = robot.inverse_continuing(&pose, &previous);
        assert_eq!(solutions.len(), 1);
        assert!((solutions[0][0] - std::f64::consts::FRAC_PI_2).abs() < EPSILON);
    }

    #[test]
    fn test_compute_jacobian() {
        let robot = SingleRotaryJointRobot;

        // This loop was used to profile rayon performance. No improvement was found so not used.
        for _e in 0..2 {
            let joints: Joints = [0.0; 6];
            let jacobian = compute_jacobian(&robot, &joints, EPSILON);
            let mut expected_jacobian = Matrix6::zeros();

            expected_jacobian[(0, 0)] = 0.0; // No effect on X position
            expected_jacobian[(1, 0)] = 1.0; // Y position is affected by the first joint
            expected_jacobian[(2, 0)] = 0.0; // No effect on Z position

            expected_jacobian[(3, 0)] = 0.0; // No effect on X orientation
            expected_jacobian[(4, 0)] = 0.0; // No effect on Y orientation
            expected_jacobian[(5, 0)] = 1.0; // Z orientation is affected by the first joint

            assert_matrix_approx_eq(&jacobian, &expected_jacobian, EPSILON);
        }
    }

    #[test]
    fn test_velocities_from_twist() {
        let robot = SingleRotaryJointRobot;
        let initial_qs = [0.0; 6];
        let jacobian = Jacobian::new(&robot, &initial_qs, EPSILON);

        // Given an end effector located 1 meter away from the axis of rotation,
        // with the joint rotating at a speed of 1 radian per second, the tip velocity is
        // one meter per second. Given we start from the angle 0, it all goes to the y component.
        let desired_velocity = Twist::new(DVec3::new(0.0, 1.0, 0.0), DVec3::new(0.0, 0.0, 1.0));
        let result = jacobian.velocities(&desired_velocity);

        assert!(result.is_ok());
        let joint_velocities = result.unwrap();
        println!("Computed joint velocities: {:?}", joint_velocities);

        // Add assertions to verify the expected values
        assert!((joint_velocities[0] - 1.0).abs() < EPSILON);
        assert_eq!(joint_velocities[1], 0.0);
        assert_eq!(joint_velocities[2], 0.0);
        assert_eq!(joint_velocities[3], 0.0);
        assert_eq!(joint_velocities[4], 0.0);
        assert_eq!(joint_velocities[5], 0.0);
    }

    #[test]
    fn test_compute_joint_torques() {
        let robot = SingleRotaryJointRobot;
        let initial_qs = [0.0; 6];
        let jacobian = Jacobian::new(&robot, &initial_qs, EPSILON);

        // For a single joint robot, that we want on the torgue is what we need to put
        let desired_force_torque =
            Wrench::new(DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.0, 0.0, 1.234));

        let joint_torques = jacobian.torques(&desired_force_torque);
        println!("Computed joint torques: {:?}", joint_torques);

        assert_eq!(joint_torques[0], 1.234);
        assert_eq!(joint_torques[1], 0.0);
        assert_eq!(joint_torques[2], 0.0);
        assert_eq!(joint_torques[3], 0.0);
        assert_eq!(joint_torques[4], 0.0);
        assert_eq!(joint_torques[5], 0.0);
    }
}

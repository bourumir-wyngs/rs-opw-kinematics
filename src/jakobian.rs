extern crate nalgebra as na;
use na::{Matrix6, Vector6, Isometry3, Translation3, UnitQuaternion, Vector3};
use na::linalg::SVD;
use rayon::prelude::*;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};
use crate::utils::vector6_to_joints;

/// Struct representing the Jacobian matrix
pub struct Jacobian {
    /// A 6x6 matrix representing the Jacobian
    ///
    /// The Jacobian matrix maps the joint velocities to the end-effector velocities.
    /// Each column corresponds to a joint, and each row corresponds to a degree of freedom
    /// of the end-effector (linear and angular velocities).    
    matrix: Matrix6<f64>,
    
    /// The disturbance value used for computing the Jakobian
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

    /// Computes the joint velocities required to achieve a desired end-effector velocity
    ///
    /// # Arguments
    ///
    /// * `desired_end_effector_velocity` - An Isometry3 representing the desired linear and angular velocity of the end-effector
    ///
    /// # Returns
    ///
    /// `Result<Joints, &'static str>` - Joint positions, with values representing joint velocities rather than angles,
    /// or an error message if the computation fails.
    ///
    /// This method extracts the linear and angular velocities from the provided Isometry3
    /// and combines them into a single 6D vector. It then computes the joint velocities required
    /// to achieve the desired end-effector velocity using the `velocities_from_vector` method.
    pub fn velocities(&self, desired_end_effector_velocity: &Isometry3<f64>) -> Result<Joints, &'static str> {
        // Extract the linear velocity (translation) and angular velocity (rotation)
        let linear_velocity = desired_end_effector_velocity.translation.vector;
        let angular_velocity = desired_end_effector_velocity.rotation.scaled_axis();

        // Combine into a single 6D vector
        let desired_velocity = Vector6::new(
            linear_velocity.x, linear_velocity.y, linear_velocity.z,
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
        );

        // Compute the joint velocities from the 6D vector
        self.velocities_from_vector(&desired_velocity)
    }

    /// Computes the joint velocities required to achieve a desired end-effector velocity
    ///
    /// # Arguments
    ///
    /// * `desired_end_effector_velocity` - A 6D vector representing the desired linear and angular velocity of the end-effector
    ///
    /// # Returns
    ///
    /// `Result<Joints, &'static str>` - Joint positions, with values representing joint velocities rather than angles,
    /// or an error message if the computation fails.
    ///
    /// This method tries to compute the joint velocities using the inverse of the Jacobian matrix.
    /// If the Jacobian matrix is not invertible, it falls back to using the pseudoinverse.
    pub fn velocities_from_vector(&self, desired_end_effector_velocity: &Vector6<f64>) -> Result<Joints, &'static str> {
        // Try to calculate the joint velocities using the inverse of the Jacobian matrix
        let joint_velocities: Vector6<f64>;
        if let Some(jacobian_inverse) = self.matrix.try_inverse() {
            joint_velocities = jacobian_inverse * desired_end_effector_velocity;
        } else {
            // If the inverse does not exist, use the pseudoinverse
            let svd = SVD::new(self.matrix.clone(), true, true);
            match svd.pseudo_inverse(self.epsilon) {
                Ok(jacobian_pseudoinverse) => {
                    joint_velocities = jacobian_pseudoinverse * desired_end_effector_velocity;
                }
                Err(_) => {
                    return Err("Unable to compute the pseudoinverse of the Jacobian matrix");
                }
            }
        }
        // Convert the resulting Vector6 to Joints
        Ok(vector6_to_joints(joint_velocities))
    }

    /// Computes the joint torques required to achieve a desired end-effector force/torque
    ///
    /// # Arguments
    ///
    /// * `desired_force_torque` - isometry structure representing forces and torgues
    ///                            rather than dimensions and angles.
    ///
    /// # Returns
    ///
    /// Joint positions, with values representing joint torques,
    /// or an error message if the computation fails.
    pub fn torques(&self, desired_force_torque: &Isometry3<f64>) -> Joints {

        // Extract the linear velocity (translation) and angular velocity (rotation)
        let linear_force = desired_force_torque.translation.vector;
        let angular_torgue = desired_force_torque.rotation.scaled_axis();

        // Combine into a single 6D vector
        let desired_force_torgue_vector = Vector6::new(
            linear_force.x, linear_force.y, linear_force.z,
            angular_torgue.x, angular_torgue.y, angular_torgue.z,
        );        
        
        let joint_torques = self.matrix.transpose() * desired_force_torgue_vector;
        vector6_to_joints(joint_torques)
    }

    /// Computes the joint torques required to achieve a desired end-effector force/torque
    ///
    /// # Arguments
    ///
    /// * `desired_force_torque` - A 6D vector representing the desired force and torque at the end-effector
    ///
    /// # Returns
    ///
    /// Joint positions, with values representing joint torques,
    /// or an error message if the computation fails.
    pub fn torques_from_vector(&self, desired_force_torque: &Vector6<f64>) -> Joints {
        let joint_torques = self.matrix.transpose() * desired_force_torque;
        vector6_to_joints(joint_torques)
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
pub fn compute_jacobian(robot: &impl Kinematics, joints: &Joints, epsilon: f64) -> Matrix6<f64> {
    let mut jacobian = Matrix6::zeros();
    let current_pose = robot.forward(joints);
    let current_position = current_pose.translation.vector;
    let current_orientation = current_pose.rotation;

    // Parallelize the loop using rayon
    let jacobian_columns: Vec<_> = (0..6).into_par_iter().map(|i| {
        let mut perturbed_qs = *joints;
        perturbed_qs[i] += epsilon;
        let perturbed_pose = robot.forward(&perturbed_qs);
        let perturbed_position = perturbed_pose.translation.vector;
        let perturbed_orientation = perturbed_pose.rotation;

        let delta_position = (perturbed_position - current_position) / epsilon;
        let delta_orientation = (perturbed_orientation * current_orientation.inverse()).scaled_axis() / epsilon;

        (delta_position, delta_orientation)
    }).collect();

    for (i, (delta_position, delta_orientation)) in jacobian_columns.into_iter().enumerate() {
        jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&delta_position);
        jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&delta_orientation);
    }

    jacobian
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    /// Example implementation of the Kinematics trait for a single rotary joint robot
    /// When the first joint rotates, it affects the Y-position and the Z-orientation of the end-effector.
    /// The derivative of the Y-position with respect to the first joint should be 1.
    /// The derivative of the Z-orientation with respect to the first joint should be 1.
    /// No other joint affects the end-effector in this simple robot model.
    pub struct SingleRotaryJointRobot;

    impl Kinematics for SingleRotaryJointRobot {
        fn inverse(&self, _pose: &Pose) -> Solutions {
            panic!() // Should not be used
        }

        /// Simple inverse kinematics for a single rotary joint of the length 1.
        fn inverse_continuing(&self, pose: &Pose, _previous: &Joints) -> Solutions {
            let angle = pose.translation.vector[1].atan2(pose.translation.vector[0]);
            vec![[angle, 0.0, 0.0, 0.0, 0.0, 0.0]]
        }

        fn forward(&self, qs: &Joints) -> Pose {
            // Forward kinematics for a single rotary joint robot
            let angle = qs[0];
            let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, angle);
            let translation = Translation3::new(angle.cos(), angle.sin(), 0.0);
            Isometry3::from_parts(translation, rotation)
        }

        fn kinematic_singularity(&self, _qs: &Joints) -> Option<Singularity> {
            None
        }
    }


    fn assert_matrix_approx_eq(left: &Matrix6<f64>, right: &Matrix6<f64>, epsilon: f64) {
        for i in 0..6 {
            for j in 0..6 {
                assert!((left[(i, j)] - right[(i, j)]).abs() < epsilon, "left[{0},{1}] = {2} is not approximately equal to right[{0},{1}] = {3}", i, j, left[(i, j)], right[(i, j)]);
            }
        }
    }

    #[test]
    fn test_forward_kinematics() {
        let robot = SingleRotaryJointRobot;
        let joints: Joints = [std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0];
        let pose = robot.forward(&joints);
        assert!((pose.translation.vector[0] - 0.0).abs() < EPSILON);
        assert!((pose.translation.vector[1] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_inverse_kinematics() {
        let robot = SingleRotaryJointRobot;
        let pose = Isometry3::new(Vector3::new(0.0, 1.0, 0.0), na::zero());
        let previous: Joints = [0.0; 6];
        let solutions = robot.inverse_continuing(&pose, &previous);
        assert_eq!(solutions.len(), 1);
        assert!((solutions[0][0] - std::f64::consts::FRAC_PI_2).abs() < EPSILON);
    }

    #[test]
    fn test_compute_jacobian() {
        let robot = SingleRotaryJointRobot;
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

    #[test]
    fn test_velocities_from_iso() {
        let robot = SingleRotaryJointRobot;
        let initial_qs = [0.0; 6];
        let jacobian = Jacobian::new(&robot, &initial_qs, EPSILON);

        // Given an end effector located 1 meter away from the axis of rotation, 
        // with the joint rotating at a speed of 1 radian per second, the tip velocity is
        // one meter per second. Given we start from the angle 0, it all goes to the y component.
        let desired_velocity_isometry =
            Isometry3::new(Vector3::new(0.0, 1.0, 0.0),
                           Vector3::new(0.0, 0.0, 1.0));
        let result = jacobian.velocities(&desired_velocity_isometry);

        assert!(result.is_ok());
        let joint_velocities = result.unwrap();
        println!("Computed joint velocities: {:?}", joint_velocities);

        // Add assertions to verify the expected values
        assert!( (joint_velocities[0] - 1.0).abs() < EPSILON);
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
            Isometry3::new(Vector3::new(0.0, 0.0, 0.0),
                           Vector3::new(0.0, 0.0, 1.234));            

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




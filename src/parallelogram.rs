use std::sync::Arc;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};

/// Parallelogram Mechanism:
/// The parallelogram mechanism alters how the end-effector's position depends on the joint rotations.
///
/// This mechanism maintains the orientation of the end-effector as the robot arm moves.
/// It introduces a geometric dependency between specific joints, typically joints 2 and 3,
/// where the movement of joint 2 influences the position of joint 3 to ensure that the
/// end-effector remains level. This is useful in tasks that require a constant tool
/// orientation, such as welding or handling objects.
///
/// In forward kinematics, joint 3 is adjusted based on joint 2 to account for the parallelogram linkage:
/// `joint_3' = joint_3 - joint_2`. This adjustment ensures the correct alignment of the end-effector during motion.
///
/// In inverse kinematics, the dependency is reversed, adding the influence of joint 2 back to joint 3:
/// `joint_3' = joint_3 + joint_2`. This ensures accurate calculation of joint angles to achieve the desired pose and orientation.
///
/// These adjustments maintain the correct relationship between joints and guarantee that the end-effector
/// follows the intended path while maintaining its orientation.

/// The Parallelogram automatically adjusts joint 3 based on joint 2
/// to account for the parallelogram mechanism. It has no data.
#[derive(Clone)]
pub struct Parallelogram {
    pub robot: Arc<dyn Kinematics>, // Underlying robot's kinematics
}

impl Kinematics for Parallelogram {
    fn inverse(&self, tcp: &Pose) -> Solutions {
        let mut solutions = self.robot.inverse(tcp);
        // Adjust for the parallelogram mechanism
        solutions.iter_mut().for_each(|x| x[2] += x[1]); // Reversing the influence of joint 2 in inverse kinematics
        solutions
    }

    fn inverse_5dof(&self, tcp: &Pose, j6: f64) -> Solutions {
        let mut solutions = self.robot.inverse_5dof(tcp, j6);
        // Adjust for the parallelogram mechanism
        solutions.iter_mut().for_each(|x| x[2] += x[1]); // Reversing the influence of joint 2 in inverse kinematics
        solutions
    }

    fn inverse_continuing_5dof(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        let mut solutions = self.robot.inverse_continuing_5dof(tcp, previous);
        // Adjust for the parallelogram mechanism
        solutions.iter_mut().for_each(|x| x[2] += x[1]); // Reversing the influence of joint 2 in inverse kinematics
        solutions
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        let mut solutions = self.robot.inverse_continuing(tcp, previous);
        // Adjust for the parallelogram mechanism
        solutions.iter_mut().for_each(|x| x[2] += x[1]); // Reversing the influence of joint 2 in inverse kinematics
        solutions
    }

    fn forward(&self, qs: &Joints) -> Pose {
        let mut joints = *qs; // Create a mutable copy of the joints
        // Adjust for the parallelogram mechanism in forward kinematics
        joints[2] -= joints[1]; // Adjusting joint 3 based on joint 2 in forward kinematics
        self.robot.forward(&joints)
    }

    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        let mut joints = *joints; // Create a mutable copy of the joints
        // Adjust for the parallelogram mechanism in forward kinematics
        joints[2] -= joints[1]; // Adjusting joint 3 based on joint 2 in forward kinematics
        self.robot.forward_with_joint_poses(&joints)
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.robot.kinematic_singularity(qs)
    }
}

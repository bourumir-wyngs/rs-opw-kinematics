use std::sync::Arc;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};

/// Parallelogram Mechanism:
/// The parallelogram mechanism introduces a geometric dependency between two specific joints,
/// typically to maintain the orientation of the end-effector as the robot arm moves.
/// This is especially useful in tasks that require a constant tool orientation, such as welding
/// or handling objects, ensuring that the tool or end-effector remains level.
///
/// The mechanism links two joints, referred to as `joints[driven]` and `joints[coupled]`. The movement
/// of the driven joint influences the coupled joint, maintaining the orientation of the end-effector
/// during motion. The scaling factor determines the proportional influence of the driven joint on the
/// coupled joint.
///
/// - **Forward Kinematics**: The coupled joint (`joints[coupled]`) is adjusted based on the driven joint:
///   `joints[coupled]' = joints[coupled] - scaling * joints[driven]`. This adjustment maintains the correct
///   alignment of the end-effector.
/// - **Inverse Kinematics**: The dependency is reversed, adding the influence of the driven joint to the
///   coupled joint: `joints[coupled]' = joints[coupled] + scaling * joints[driven]`. This ensures accurate
///   calculation of joint angles to achieve the desired pose and orientation.
///
/// The `Parallelogram` structure automatically adjusts `joints[coupled]` based on `joints[driven]` using
/// a scaling factor to account for the parallelogram mechanism.
///
/// # Fields:
/// - `robot`: The underlying robot's kinematics model used to compute forward and inverse kinematics.
/// - `scaling`: The factor that determines how much influence `joints[driven]` has on `joints[coupled]`.
/// - `driven`: The index of the driven joint in the parallelogram mechanism (typically the primary joint).
/// - `coupled`: The index of the coupled joint in the parallelogram mechanism (the secondary joint influenced by the driven joint).
///
/// # Example:
/// ```rust
/// use std::sync::Arc;
/// // As J1 = 0, J2 = 1 and J3 = 2, so it is more clear with J-constants:
/// use rs_opw_kinematics::kinematic_traits::{J2, J3};
///
/// use rs_opw_kinematics::kinematics_impl::OPWKinematics;
/// use rs_opw_kinematics::parallelogram::Parallelogram;
/// use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
///
/// // Assuming a robot that implements the Kinematics trait
/// let robot_kinematics = Arc::new(OPWKinematics::new(Parameters::irb2400_10()));
///
/// // Create the Parallelogram structure with a scaling factor of 0.5,
/// // where joints[1] is the driven joint and joints[2] is the coupled joint.
/// let parallelogram = Parallelogram {
///     robot: robot_kinematics,
///     scaling: 1.0, // typically there is 1-to-1 influence between driven and coupled joints
///     driven: J2,   // Joint 2 is most often the driven joint. 
///     coupled: J3,  // Joint 3 is most often the coupled joint
/// };
/// ```
/// As Parallelogram accepts and itself implements Kinematics, it is possible to chain multiple 
/// parallelograms if the robot has more than one.
/// 
#[derive(Clone)]
pub struct Parallelogram {
    /// The underlying robot's kinematics used for forward and inverse kinematics calculations.
    pub robot: Arc<dyn Kinematics>,

    /// The scaling factor that determines the proportional influence of `joints[driven]` on `joints[coupled]`.
    pub scaling: f64,

    /// The index of the driven joint in the parallelogram mechanism (`joints[driven]`).
    pub driven: usize,

    /// The index of the coupled joint in the parallelogram mechanism (`joints[coupled]`).
    pub coupled: usize,
}
impl Kinematics for Parallelogram {
    fn inverse(&self, tcp: &Pose) -> Solutions {
        let mut solutions = self.robot.inverse(tcp);

        // Reversing the influence of driven joint in inverse kinematics
        solutions.iter_mut().for_each(|x| x[self.coupled] += 
            self.scaling * x[self.driven]); 
        solutions
    }

    fn inverse_5dof(&self, tcp: &Pose, j6: f64) -> Solutions {
        let mut solutions = self.robot.inverse_5dof(tcp, j6);

        // Reversing the influence of driven joint in inverse kinematics        
        solutions.iter_mut().for_each(|x| x[self.coupled] += 
            self.scaling * x[self.driven]); 
        solutions
    }

    fn inverse_continuing_5dof(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        let mut solutions = self.robot.inverse_continuing_5dof(tcp, previous);
        
        // Reversing the influence of driven joint in inverse kinematics
        solutions.iter_mut().for_each(|x| x[self.coupled] += 
            self.scaling * x[self.driven]); 
        solutions
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        let mut solutions = self.robot.inverse_continuing(tcp, previous);
        
        // Reversing the influence of driven joint in inverse kinematics
        solutions.iter_mut().for_each(|x| x[self.coupled] += 
            self.scaling * x[self.driven]); 
        solutions
    }

    fn forward(&self, qs: &Joints) -> Pose {
        let mut joints = *qs;
        // Adjusting coupled joint based on driven joint in forward kinematics
        joints[self.coupled] -= self.scaling * joints[self.driven]; 
        self.robot.forward(&joints)
    }

    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        let mut joints = *joints; 
        // Adjusting coupled joint based on driven joint in forward kinematics
        joints[self.coupled] -= self.scaling * joints[self.driven]; 
        self.robot.forward_with_joint_poses(&joints)
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.robot.kinematic_singularity(qs)
    }
}

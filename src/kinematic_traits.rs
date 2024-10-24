//! Defines traits for direct and inverse kinematics.
 
extern crate nalgebra as na;

use std::f64::NAN;
use na::{Isometry3};


/// Pose is used a pose of the robot tcp. It contains both Cartesian position and rotation quaternion
/// ```
/// extern crate nalgebra as na;
/// use na::{Isometry3, Translation3, UnitQuaternion, Vector3};
///
/// type Pose = Isometry3<f64>;
///
/// let translation = Translation3::new(1.0, 0.0, 0.0);
/// // The quaternion should be normalized to represent a valid rotation.
/// let rotation = UnitQuaternion::from_quaternion(na::Quaternion::new(1.0, 0.0, 0.0, 1.0).normalize());
/// let transform = Pose::from_parts(translation, rotation);
/// ```
pub type Pose = Isometry3<f64>;

/// Defines kinematic singularity. A is a singularity when J5 = 0 (this is possible with
/// any robot). The structure is reserved for other possible singularies but these require
/// b = 0 and a1 = a2 so not possible with most of the robots. 
/// Joints are counted from 1 to 6 in this comment.
#[derive(PartialEq, Debug)]
pub enum Singularity {
    /// Represents singularity when J5 = 0, possible with any robot.
    A,
}

/// Six rotary joints of the robot with angles in radians. 
pub type Joints = [f64; 6];

// Define indices for easier reading (numbering in array starts from 0 and this one-off is
// contra - intuitive)
#[allow(dead_code)]
pub const J1: usize = 0;
#[allow(dead_code)]
pub const J2: usize = 1;
#[allow(dead_code)]
pub const J3: usize = 2;
#[allow(dead_code)]
pub const J4: usize = 3;
#[allow(dead_code)]
pub const J5: usize = 4;
#[allow(dead_code)]
pub const J6: usize = 5;

#[allow(dead_code)]
/// Tool attached to J5, used in collision detection reporting
pub const J_TOOL: usize = 6;

/// For providing singularity - proof solution when the previous value is not known.
/// Joints that take arbitrary angles will take angles as close to 0 as possible:
/// let solutions = kinematics.inverse_continuing(&pose, &JOINTS_AT_ZERO);
#[allow(dead_code)]
pub const JOINTS_AT_ZERO: Joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

/// Special value that can be used with inverse_continuing, indicating that there
/// are not previous joint value, but returned joints must be sorted to be as 
/// close as possible to the centers of the constraints. If no constraitns are set,
/// zeroes are assumed.
pub const CONSTRAINT_CENTERED: Joints = [NAN, 0.0, 0.0, 0.0, 0.0, 0.0];

/// For providing solutions. As invalid solutions are discarded, 
/// this is a variable length vector (may be empty if robot cannot reach the 
/// given point).
pub type Solutions = Vec<Joints>;

/// Defines agreed functionality of direct and inverse kinematics and singularity detection.
pub trait Kinematics: Send + Sync {
    /// Find inverse kinematics (joint position) for this pose
    /// This function is faster but does not handle the singularity J5 = 0 well.
    /// All returned solutions are cross-checked with forward kinematics and
    /// valid. 
    fn inverse(&self, pose: &Pose) -> Solutions;

    /// Find inverse kinematics (joint position) for this pose
    /// This function handles the singularity J5 = 0 by keeping the previous values
    /// the values J4 and J6 from the previous solution
    /// Use CONSTRAINT_CENTERED as previous if there is no previous position but we prefer
    /// to be as close to the center of constraints (or zeroes if not set) as
    /// possible.
    fn inverse_continuing(&self, pose: &Pose, previous: &Joints) -> Solutions;

    /// Find forward kinematics (pose from joint positions).
    /// For 5 DOF robot, the rotation of the joint 6 should normally be 0.0
    /// but some other value can be given, meaning the tool is mounted with
    /// fixed rotation offset.
    fn forward(&self, qs: &Joints) -> Pose;

    /// Calculates the inverse kinematics for a robot while ignoring the rotation
    /// around joint 6. The position of the tool center point remains precise,
    /// but the rotation is approximate (rotation around the tool axis is ignored).
    /// The return value for joint 6 is set according to the provided parameter.
    /// This method is significantly faster
    fn inverse_5dof(&self, pose: &Pose, j6: f64) -> Solutions;


    /// Calculates the inverse kinematics for a robot while ignoring the rotation
    /// around joint 6. The position of the tool center point remains precise,
    /// but the rotation is approximate (rotation around the tool axis is ignored).
    /// The return value for joint 6 is set based on the previous joint values.
    /// This method is significantly faster
    fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions;
    
    /// Detect the singularity. Returns either A type singlularity or None if
    /// no singularity detected.
    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity>;

    /// Computes the forward kinematics for a 6-DOF robotic arm and returns an array of poses
    /// representing the position and orientation of each joint, including the final end-effector.
    ///
    /// The function calculates the transformation for each joint in the robotic arm using the joint
    /// angles (in radians) and the kinematic parameters of the robot (link lengths and offsets).
    /// It returns an array of 6 poses: one for each joint and the end-effector.
    ///
    /// # Parameters
    /// - `self`: A reference to the kinematics model containing the geometric and joint-specific parameters.
    /// - `joints`: A reference to an array of joint angles (in radians) for the 6 joints of the robot.
    ///
    /// # Returns
    /// - An array of 6 `Pose` structures:
    ///   - Pose 1: The position and orientation of the base link.
    ///   - Pose 2: The position and orientation of link 1 (first joint).
    ///   - Pose 3: The position and orientation of link 2 (second joint).
    ///   - Pose 4: The position and orientation of link 3 (third joint).
    ///   - Pose 5: The position and orientation of link 4 (fourth joint), before applying any wrist offset.
    ///   - Pose 6: The position and orientation of the end-effector, including the wrist offset (`c4`).
    ///
    /// # Example
    /// ```
    /// use rs_opw_kinematics::kinematic_traits::Kinematics;
    /// use rs_opw_kinematics::kinematics_impl::OPWKinematics;
    /// use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
    /// let parameters = Parameters {
    ///     a1: 0.150,
    ///     a2: 0.000,
    ///     b: 0.000,
    ///     c1: 0.550,
    ///     c2: 0.550,
    ///     c3: 0.600,
    ///     c4: 0.110,
    ///     offsets: [0.0; 6],  // No joint offsets
    ///     ..Parameters::new()
    /// };
    ///
    /// let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];  // All joints straight up
    /// let kinematics = OPWKinematics::new(parameters);
    ///
    /// let poses = kinematics.forward_with_joint_poses(&joints);
    ///
    /// assert_eq!(poses.len(), 6);  // Should return 6 poses
    /// ```
    ///
    /// # Notes
    /// - The function applies the geometric parameters (like link lengths and joint offsets) and computes
    ///   each joint's position and orientation relative to the base frame.
    /// - The final pose (Pose 6) includes the `c4` offset, which accounts for the wrist length.    
    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6];
}


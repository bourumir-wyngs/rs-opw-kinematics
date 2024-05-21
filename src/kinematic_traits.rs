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
pub trait Kinematics {
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
    fn forward(&self, qs: &Joints) -> Pose;

    /// Detect the singularity. Returns either A type singlularity or None if
    /// no singularity detected.
    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity>;
}


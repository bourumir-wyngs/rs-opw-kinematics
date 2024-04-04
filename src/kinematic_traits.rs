extern crate nalgebra as na;

pub mod kinematics_traits {
    use super::*;
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
    pub(crate) type Pose = Isometry3<f64>;

    /// Defines kinematic singularity. A is a singularity when J5 = 0 (this is possible with
    /// any robot and B is a more rare singularity when J2 = 0 (this is possible for robots
    /// with b = 0. Joints are counted from 1 to 6 in this comment.
    #[derive(PartialEq, Debug)]
    pub(crate) enum Singularity {
        /// Represents singularity when J5 = 0, possible with any robot.
        A,
        /// Represents singularity when J2 = 0, possible for any robot with parameter b == 0.
        B,
        /// Represents case when both A and B apply
        AB
    }

    /// This library may return up to 8 solutions, each defining the rotations of the 6 joints.
    /// Use isValid in utils.rs to check if the solution is valid.
    pub(crate) type Solutions = na::Matrix<f64, na::U6, na::U8, na::ArrayStorage<f64, 6, 8>>;

    pub trait Kinematics {
        fn inverse(&self, pose: &Pose) -> Solutions;
        fn forward(&self, qs: &[f64; 6]) -> Pose;

        /// Detect the singularity. Returns either A, B type singlularity or None if
        /// no singularity detected.
        fn kinematic_singularity(&self, qs: &[f64; 6]) -> Option<Singularity>;
    }
}

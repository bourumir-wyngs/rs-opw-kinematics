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

    /// This library may return up to 8 solutions, each defining the rotations of the 6 joints.
    /// Use isValid in utils.rs to check if the solution is valid.
    pub(crate) type Solutions = na::Matrix<f64, na::U6, na::U8, na::ArrayStorage<f64, 6, 8>>;

    pub trait Kinematics {
        fn inverse(&self, pose: &Pose) -> Solutions;
        fn forward(&self, qs: &[f64; 6]) -> Pose;
    }
}

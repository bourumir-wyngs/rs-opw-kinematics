//! Supports concept of the frame that is common in robotics.
//! Transform frame allows to compute or prepare by hand joint positions of the robot once,
//! for some "standard" location. If the target the robot needs to work with moves somewhere
//! else, it is possible to define a "Frame", specifying this target displacement
//! (most often specifying 3 points both before and after displacement). The common example
//! is robot picking boxes from the transportation pallets: the same box picking program
//! can be used for picking from different pallets by specifying a new transport frame
//! each time.
//!
//! The main functionality of the Frame is to use the "standard" joint positions for finding
//! the new joint position, as they would be required after applying the frame. The Frame in
//! this package also implements the Kinematics trait if such would be required.

use nalgebra::{Isometry3, Point3, Rotation3, Translation3, UnitQuaternion};
use std::error::Error;
use std::fmt;
use std::sync::Arc;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};


/// Defines the frame that transforms the robot working area, moving and rotating it
/// (not stretching). The frame can be created from 3 pairs of points, one defining
/// the points before transforming and another after, or alternatively 1 pair is enough
/// if we assume there is no rotation.
pub struct Frame {
    pub robot: Arc<dyn Kinematics>,  // The robot

    /// The frame transform, normally computed with either Frame::translation or Frame::isometry
    pub frame: Isometry3<f64>,
}

impl Frame {
    /// Compute the frame transform that may include only shift (but not rotation)
    pub fn translation(p: Point3<f64>, q: Point3<f64>) -> Isometry3<f64> {
        let translation = q - p;
        Isometry3::from_parts(Translation3::from(translation), nalgebra::UnitQuaternion::identity())
    }

    /// Compute the frame transform that may include shift and rotation (but not stretching)
    pub fn frame(
        p1: Point3<f64>,
        p2: Point3<f64>,
        p3: Point3<f64>,
        q1: Point3<f64>,
        q2: Point3<f64>,
        q3: Point3<f64>,
    ) -> Result<Isometry3<f64>, Box<dyn Error>> {
        // Vectors between points
        let v1 = p2 - p1;
        let v2 = p3 - p1;

        if v1.cross(&v2).norm() == 0.0 {
            return Err(Box::new(ColinearPoints::new(p1, p2, p3, true)));
        }

        // Destination vectors
        let w1 = q2 - q1;
        let w2 = q3 - q1;

        if w1.cross(&w2).norm() == 0.0 {
            return Err(Box::new(ColinearPoints::new(q1, q2, q3, false)));
        }

        // Create orthonormal bases
        let b1 = v1.normalize();
        let b2 = v1.cross(&v2).normalize();
        let b3 = b1.cross(&b2);

        let d1 = w1.normalize();
        let d2 = w1.cross(&w2).normalize();
        let d3 = d1.cross(&d2);

        let rotation_matrix = nalgebra::Matrix3::from_columns(&[
            d1, d2, d3,
        ]) * nalgebra::Matrix3::from_columns(&[
            b1, b2, b3,
        ]).transpose();

        let rotation_matrix = Rotation3::from_matrix_unchecked(rotation_matrix);
        let rotation = UnitQuaternion::from_rotation_matrix(&rotation_matrix);
        let translation = q1 - rotation.transform_point(&p1);

        Ok(Isometry3::from_parts(translation.into(), rotation))
    }

    /// This function calculates the required joint values for a robot after applying a transformation
    /// to the tool center point (TCP) location, as defined by a specified frame. It uses the provided
    /// joint positions (qs) and the previous joint positions to compute the new positions. Depending on
    /// how the frame is defined, there can be no solutions or multiple solutions; hence, the function
    /// returns a `Solutions` structure to account for this variability. Additionally, this method
    /// returns the transformed tool center pose.
    ///
    /// # Arguments
    ///
    /// * `qs` - A reference to the `Joints` structure representing the initial joint positions
    ///   before the transformation.
    /// * `previous` - A reference to the `Joints` structure representing the previous joint positions.
    ///
    /// # Returns
    ///
    /// A tuple containing:
    /// * `Solutions` - A structure containing the possible joint positions after transformation.
    /// * `Pose` - The transformed tool center pose.
    ///
    /// # Example
    ///
    /// ```
    /// use std::sync::Arc;
    /// use nalgebra::Point3;
    /// use rs_opw_kinematics::frame::Frame;
    /// use rs_opw_kinematics::kinematics_impl::OPWKinematics;
    /// use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
    ///
    /// let robot = OPWKinematics::new(Parameters::irb2400_10());
    /// let frame_transform = Frame::translation(
    ///   Point3::new(0.0, 0.0, 0.0), Point3::new(0.01, 0.00, 0.0));
    /// let framed = rs_opw_kinematics::frame::Frame {
    ///   robot: Arc::new(robot),
    ///   frame: frame_transform,
    /// };
    /// let joints_no_frame: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5]; // without frame
    /// let (solutions, _transformed_pose) = framed.forward_transformed(
    ///   &joints_no_frame, &joints_no_frame /* prioritize closest to this value */);
    /// // Use solutions and transformed_pose as needed
    /// ```
    pub fn forward_transformed(&self, qs: &Joints, previous: &Joints) -> (Solutions, Pose) {
        // Calculate the pose of the tip joint using the robot's kinematics
        let tip_joint = self.robot.forward(qs);

        // Apply the frame transformation to the tool center point (TCP)
        let tcp = tip_joint * self.frame;

        // Compute the transformed joint values based on the transformed TCP pose and 
        // previous joint positions
        let transformed_joints = self.robot.inverse_continuing(&tcp, previous);

        // Return the possible joint solutions and the transformed TCP pose
        (transformed_joints, tcp)
    }
}

impl Kinematics for Frame {
    fn inverse(&self, tcp: &Pose) -> Solutions {
        self.robot.inverse(&(tcp * self.frame.inverse()))
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        self.robot.inverse_continuing(&(tcp * self.frame.inverse()), previous)
    }

    fn forward(&self, qs: &Joints) -> Pose {
        // Calculate the pose of the tip joint using the robot's kinematics
        let tip_joint = self.robot.forward(qs);
        let tcp = tip_joint * self.frame;
        tcp
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.robot.kinematic_singularity(qs)
    }
}


/// Defines error when points specified as source or target for creating the frame are colinear (on the same line).
/// Such points cannot be used to create the frame. The exact values of the points in question
/// are included in the error structure and printed in the error message.
#[derive(Debug)]
pub struct ColinearPoints {
    pub p1: Point3<f64>,
    pub p2: Point3<f64>,
    pub p3: Point3<f64>,
    pub source: bool,
}

impl ColinearPoints {
    pub fn new(p1: Point3<f64>, p2: Point3<f64>, p3: Point3<f64>, source: bool) -> Self {
        Self { p1, p2, p3, source }
    }
}

impl fmt::Display for ColinearPoints {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f, "Cannot create a frame from colinear {} points: p1 = {:?}, p2 = {:?}, p3 = {:?}",
            if self.source { "source" } else { "target" }, self.p1, self.p2, self.p3
        )
    }
}

impl Error for ColinearPoints {}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, Vector3};

    #[test]
    fn test_find_isometry3_rotation_translation() {
        // Define points before transformation
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(1.0, 0.0, 0.0);
        let p3 = Point3::new(0.0, 1.0, 0.0);

        // Define points after 90-degree rotation around Z axis and translation by (1, 2, 0)
        let q1 = Point3::new(1.0, 2.0, 0.0);
        let q2 = Point3::new(1.0, 3.0, 0.0);
        let q3 = Point3::new(0.0, 2.0, 0.0);

        // Define additional points before transformation
        let p4 = Point3::new(1.0, 1.0, 0.0);
        let p5 = Point3::new(2.0, 1.0, 0.0);
        let p6 = Point3::new(1.0, 2.0, 0.0);

        // Define expected points after transformation
        let q4 = Point3::new(0.0, 3.0, 0.0); // p4 rotated 90 degrees and translated
        let q5 = Point3::new(0.0, 4.0, 0.0); // p5 rotated 90 degrees and translated
        let q6 = Point3::new(-1.0, 3.0, 0.0); // p6 rotated 90 degrees and translated

        // Find the isometry using our function
        let result = Frame::frame(p1, p2, p3, q1, q2, q3)
            .expect("These points are not colinear and must be ok");

        // Expected translation
        let expected_translation = Translation3::new(1.0, 2.0, 0.0);

        // Expected rotation (90 degrees around the Z axis)
        let expected_rotation = 
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);

        // Compare the result with the expected translation and rotation
        assert!((result.translation.vector - expected_translation.vector).norm() < 1e-6);

        // Convert Unit<Vector3<f64>> to Vector3<f64> for comparison
        let result_axis = result.rotation.axis().unwrap().into_inner();
        let expected_axis = expected_rotation.axis().unwrap().into_inner();
        assert!((result_axis - expected_axis).norm() < 1e-6);
        assert!((result.rotation.angle() - expected_rotation.angle()).abs() < 1e-6);

        // Check if the additional points are transformed correctly
        assert!((result.transform_point(&p4) - q4).norm() < 1e-6);
        assert!((result.transform_point(&p5) - q5).norm() < 1e-6);
        assert!((result.transform_point(&p6) - q6).norm() < 1e-6);
    }

    #[test]
    fn test_find_translation() {
        // Define points
        let p = Point3::new(1.0, 2.0, 3.0);
        let q = Point3::new(4.0, 5.0, 6.0);

        // Expected translation vector
        let expected_translation = Translation3::new(3.0, 3.0, 3.0);

        // Get the isometry
        let isometry = Frame::translation(p, q);

        // Check if the translation part is correct
        assert!((isometry.translation.vector - expected_translation.vector).norm() < 1e-6);

        // Check if the rotation part is identity
        let identity_rotation = UnitQuaternion::identity();
        assert!((isometry.rotation.quaternion() - identity_rotation.quaternion()).norm() < 1e-6);
    }
}


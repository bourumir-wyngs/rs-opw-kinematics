//! Supports concept of the frame that is common in robotics.
//! Transform frame allows to compute or prepare by hand joint positions of the robot once,
//! for some "standard" location. If the target the robot needs to work with moves somewhere
//! else, it is possible to define a "Frame", specifying this target displacement
//! (most often specifying 3 tie points both before and after displacement). The common example
//! is robot picking boxes from the transportation pallets: the same box picking program
//! can be used for picking from different pallets by specifying a new transport frame
//! each time. Tie point frames may include uniform scale as well as rotation and translation.
//!
//! The main functionality of the Frame is to use the "standard" joint positions for finding
//! the new joint position, as they would be required after applying the frame. The Frame in
//! this package also implements the Kinematics trait if such would be required.

use crate::constraints::Constraints;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};
use glam::{DMat3, DQuat, DVec3};
use std::error::Error;
use std::fmt;
use std::sync::Arc;

/// Similarity transform used by [`Frame`].
///
/// Unlike general robot poses, a frame transform may include uniform scale. This is useful
/// when the same canonical trajectory must be retargeted to a workpiece, pallet, or drawing
/// whose measured tie points are a scaled copy of the original tie points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameTransform {
    /// Translation applied after scaling and rotation.
    pub translation: DVec3,
    /// Rotation applied after scaling.
    pub rotation: DQuat,
    /// Uniform scale. Must be positive.
    pub scale: f64,
}

impl FrameTransform {
    /// Identity frame transform.
    pub const IDENTITY: Self = Self {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
        scale: 1.0,
    };

    /// Creates a frame transform from parts.
    pub fn from_parts(translation: DVec3, rotation: DQuat, scale: f64) -> Self {
        assert!(
            scale.is_finite() && scale > 0.0,
            "frame scale must be finite and positive"
        );
        let pose = Pose::from_parts(translation, rotation);
        Self {
            translation: pose.translation,
            rotation: pose.rotation,
            scale,
        }
    }

    /// Creates a rigid frame transform from a pose.
    pub fn from_pose(pose: Pose) -> Self {
        Self {
            translation: pose.translation,
            rotation: pose.rotation,
            scale: 1.0,
        }
    }

    /// Returns this frame as a rigid pose when scale is 1.
    pub fn rigid_pose(&self) -> Option<Pose> {
        if (self.scale - 1.0).abs() <= f64::EPSILON {
            Some(Pose::from_parts(self.translation, self.rotation))
        } else {
            None
        }
    }

    /// Transforms a point by scale, rotation, and translation.
    pub fn transform_point(&self, point: DVec3) -> DVec3 {
        self.translation + self.rotation * (point * self.scale)
    }

    /// Applies the inverse frame transform to a point.
    pub fn inverse_transform_point(&self, point: DVec3) -> DVec3 {
        (self.rotation.inverse() * (point - self.translation)) / self.scale
    }

    /// Applies this frame to a pose.
    ///
    /// Scale affects the pose translation. The pose orientation is rotated but not scaled.
    pub fn transform_pose(&self, pose: Pose) -> Pose {
        Pose::from_parts(
            self.transform_point(pose.translation),
            self.rotation * pose.rotation,
        )
    }

    /// Applies the inverse of this frame to a pose.
    pub fn inverse_transform_pose(&self, pose: Pose) -> Pose {
        Pose::from_parts(
            self.inverse_transform_point(pose.translation),
            self.rotation.inverse() * pose.rotation,
        )
    }
}

impl From<Pose> for FrameTransform {
    fn from(pose: Pose) -> Self {
        Self::from_pose(pose)
    }
}

/// Defines the frame that transforms the robot working area by translating,
/// rotating, and optionally uniformly scaling it.
///
/// The frame can be created from 3 pairs of tie points, one triplet defining the
/// original trajectory points and another triplet defining the required target points.
/// Alternatively, 1 pair is enough if only translation is involved.
pub struct Frame {
    pub robot: Arc<dyn Kinematics>, // The robot

    /// The frame transform, normally computed with [`Frame::translation`],
    /// [`Frame::frame`], or [`Frame::from_tie`].
    pub frame: FrameTransform,
}

impl Frame {
    /// Compute the frame transform that may include only shift (but not rotation)
    pub fn translation(p: DVec3, q: DVec3) -> FrameTransform {
        FrameTransform::from_parts(q - p, DQuat::IDENTITY, 1.0)
    }

    /// Compute the frame transform that may include shift and rotation (but not stretching)
    pub fn frame(
        p1: DVec3,
        p2: DVec3,
        p3: DVec3,
        q1: DVec3,
        q2: DVec3,
        q3: DVec3,
    ) -> Result<FrameTransform, Box<dyn Error>> {
        const NON_ISOMETRY_TOLERANCE: f64 = 0.005; // Tolerance how much the isometry can be actually not
        // an isometry (distance between points differs). 5 mm looks like a reasonable check.
        if !is_valid_isometry(&p1, &p2, &p3, &q1, &q2, &q3, NON_ISOMETRY_TOLERANCE) {
            return Err(Box::new(NotIsometry::new(p1, p2, p3, q1, q2, q3)));
        }
        similarity_from_tie_points([p1, p2, p3], [q1, q2, q3], 1.0)
    }

    /// Compute a frame transform from three original and three target tie points.
    ///
    /// Unlike [`Frame::frame`], this constructor supports uniform scaling. The three
    /// target tie points must be a translated, rotated, and uniformly scaled copy of
    /// the three original tie points. Non-uniform scale or shear is rejected.
    pub fn from_tie(
        original: [DVec3; 3],
        target: [DVec3; 3],
    ) -> Result<FrameTransform, Box<dyn Error>> {
        const NON_SIMILARITY_TOLERANCE: f64 = 0.005;
        let scale = uniform_scale(&original, &target)?;
        let transform = similarity_from_tie_points(original, target, scale)?;

        if !is_valid_similarity(
            &original,
            &target,
            transform.scale,
            NON_SIMILARITY_TOLERANCE,
        ) {
            return Err(Box::new(NotSimilarity::new(
                original[0],
                original[1],
                original[2],
                target[0],
                target[1],
                target[2],
            )));
        }

        Ok(transform)
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
    /// use rs_opw_kinematics::glam::DVec3;
    /// use rs_opw_kinematics::frame::Frame;
    /// use rs_opw_kinematics::kinematics_impl::OPWKinematics;
    /// use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
    ///
    /// let robot = OPWKinematics::new(Parameters::irb2400_10());
    /// let frame_transform = Frame::translation(
    ///   DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.01, 0.00, 0.0));
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
        let tcp_no_frame = self.robot.forward(qs);

        // Apply the frame transformation to the tool center point (TCP)
        let tcp_frame = self.frame.transform_pose(tcp_no_frame);

        // Compute the transformed joint values based on the transformed TCP pose and
        // previous joint positions
        let transformed_joints = self.robot.inverse_continuing(&tcp_frame, previous);

        // Return the possible joint solutions and the transformed TCP pose
        (transformed_joints, tcp_frame)
    }
}

impl Kinematics for Frame {
    fn inverse(&self, tcp: &Pose) -> Solutions {
        self.robot.inverse(&self.frame.inverse_transform_pose(*tcp))
    }

    fn inverse_5dof(&self, tcp: &Pose, j6: f64) -> Solutions {
        self.robot
            .inverse_5dof(&self.frame.inverse_transform_pose(*tcp), j6)
    }

    fn inverse_continuing_5dof(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        self.robot
            .inverse_continuing_5dof(&self.frame.inverse_transform_pose(*tcp), previous)
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        self.robot
            .inverse_continuing(&self.frame.inverse_transform_pose(*tcp), previous)
    }

    fn forward(&self, qs: &Joints) -> Pose {
        // Calculate the pose of the tip joint using the robot's kinematics
        let tip_joint = self.robot.forward(qs);

        self.frame.transform_pose(tip_joint)
    }

    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        // Compute the forward kinematics for the robot itself
        let mut poses = self.robot.forward_with_joint_poses(joints);

        // Apply the frame transformation only to the last pose (TCP pose)
        poses[5] = self.frame.transform_pose(poses[5]);

        poses
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.robot.kinematic_singularity(qs)
    }

    fn constraints(&self) -> &Option<Constraints> {
        self.robot.constraints()
    }
}

/// Defines error when points specified as source or target for creating the frame are colinear (on the same line).
/// Such points cannot be used to create the frame. The exact values of the points in question
/// are included in the error structure and printed in the error message.
#[derive(Debug)]
pub struct ColinearPoints {
    pub p1: DVec3,
    pub p2: DVec3,
    pub p3: DVec3,
    pub source: bool,
}

/// Struct to hold six points that still do not represent a valid isometry.
/// It implements Error, containing at the same time six 3D points
/// from whom the isometry cannot be constructed.
#[derive(Debug)]
pub struct NotIsometry {
    pub a1: DVec3,
    pub a2: DVec3,
    pub a3: DVec3,
    pub b1: DVec3,
    pub b2: DVec3,
    pub b3: DVec3,
}

/// Struct to hold six points that do not represent a valid similarity transform.
///
/// A frame created from tie points may rotate, translate, and uniformly scale the
/// original points. It must not shear or scale axes differently.
#[derive(Debug)]
pub struct NotSimilarity {
    pub a1: DVec3,
    pub a2: DVec3,
    pub a3: DVec3,
    pub b1: DVec3,
    pub b2: DVec3,
    pub b3: DVec3,
}

impl NotSimilarity {
    /// Creates a new NotSimilarity instance, containing 6 points that do not
    /// represent a valid similarity transform.
    pub fn new(a1: DVec3, a2: DVec3, a3: DVec3, b1: DVec3, b2: DVec3, b3: DVec3) -> Self {
        NotSimilarity {
            a1,
            a2,
            a3,
            b1,
            b2,
            b3,
        }
    }
}

impl NotIsometry {
    /// Creates a new NotIsometry instance, containing 6 points that do not
    /// represent a valid isometry.
    pub fn new(a1: DVec3, a2: DVec3, a3: DVec3, b1: DVec3, b2: DVec3, b3: DVec3) -> Self {
        NotIsometry {
            a1,
            a2,
            a3,
            b1,
            b2,
            b3,
        }
    }
}

impl fmt::Display for NotIsometry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Not isometry: (  a1: {:?},  a2: {:?},  a3: {:?},  b1: {:?},  b2: {:?},  b3: {:?})",
            self.a1, self.a2, self.a3, self.b1, self.b2, self.b3
        )
    }
}

impl fmt::Display for NotSimilarity {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Not similarity: (  a1: {:?},  a2: {:?},  a3: {:?},  b1: {:?},  b2: {:?},  b3: {:?})",
            self.a1, self.a2, self.a3, self.b1, self.b2, self.b3
        )
    }
}

impl ColinearPoints {
    pub fn new(p1: DVec3, p2: DVec3, p3: DVec3, source: bool) -> Self {
        Self { p1, p2, p3, source }
    }
}

impl fmt::Display for ColinearPoints {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Cannot create a frame from colinear {} points: p1 = {:?}, p2 = {:?}, p3 = {:?}",
            if self.source { "source" } else { "target" },
            self.p1,
            self.p2,
            self.p3
        )
    }
}

impl Error for ColinearPoints {}

impl Error for NotIsometry {}

impl Error for NotSimilarity {}

const TIE_POINT_EPSILON: f64 = 1.0e-12;

fn side_lengths(points: &[DVec3; 3]) -> [f64; 3] {
    [
        (points[0] - points[1]).length(),
        (points[0] - points[2]).length(),
        (points[1] - points[2]).length(),
    ]
}

fn uniform_scale(original: &[DVec3; 3], target: &[DVec3; 3]) -> Result<f64, Box<dyn Error>> {
    let source = side_lengths(original);
    let destination = side_lengths(target);

    if source.iter().any(|length| *length <= TIE_POINT_EPSILON) {
        return Err(Box::new(ColinearPoints::new(
            original[0],
            original[1],
            original[2],
            true,
        )));
    }

    let numerator = source
        .iter()
        .zip(destination.iter())
        .map(|(source, destination)| source * destination)
        .sum::<f64>();
    let denominator = source.iter().map(|source| source * source).sum::<f64>();
    let scale = numerator / denominator;

    if scale.is_finite() && scale > TIE_POINT_EPSILON {
        Ok(scale)
    } else {
        Err(Box::new(NotSimilarity::new(
            original[0],
            original[1],
            original[2],
            target[0],
            target[1],
            target[2],
        )))
    }
}

fn basis_from_tie_points(points: [DVec3; 3], source: bool) -> Result<DMat3, Box<dyn Error>> {
    let [p1, p2, p3] = points;
    let v1 = p2 - p1;
    let v2 = p3 - p1;
    let v1_length = v1.length();
    let normal = v1.cross(v2);
    let normal_length = normal.length();

    if v1_length <= TIE_POINT_EPSILON || normal_length <= TIE_POINT_EPSILON {
        return Err(Box::new(ColinearPoints::new(p1, p2, p3, source)));
    }

    let e1 = v1 / v1_length;
    let e3 = normal / normal_length;
    let e2 = e3.cross(e1);
    Ok(DMat3::from_cols(e1, e2, e3))
}

fn similarity_from_tie_points(
    original: [DVec3; 3],
    target: [DVec3; 3],
    scale: f64,
) -> Result<FrameTransform, Box<dyn Error>> {
    let source_basis = basis_from_tie_points(original, true)?;
    let target_basis = basis_from_tie_points(target, false)?;
    let rotation_matrix = target_basis * source_basis.transpose();
    let rotation = DQuat::from_mat3(&rotation_matrix);
    let translation = target[0] - rotation * (original[0] * scale);

    Ok(FrameTransform::from_parts(translation, rotation, scale))
}

fn distances_match(
    a1: &DVec3,
    a2: &DVec3,
    a3: &DVec3,
    b1: &DVec3,
    b2: &DVec3,
    b3: &DVec3,
    tolerance: f64,
) -> bool {
    let dist_a1_a2 = (*a1 - *a2).length();
    let dist_a1_a3 = (*a1 - *a3).length();
    let dist_a2_a3 = (*a2 - *a3).length();

    let dist_b1_b2 = (*b1 - *b2).length();
    let dist_b1_b3 = (*b1 - *b3).length();
    let dist_b2_b3 = (*b2 - *b3).length();

    (dist_a1_a2 - dist_b1_b2).abs() < tolerance
        && (dist_a1_a3 - dist_b1_b3).abs() < tolerance
        && (dist_a2_a3 - dist_b2_b3).abs() < tolerance
}

fn distances_match_scaled(
    original: &[DVec3; 3],
    target: &[DVec3; 3],
    scale: f64,
    tolerance: f64,
) -> bool {
    let source = side_lengths(original);
    let destination = side_lengths(target);

    source
        .iter()
        .zip(destination.iter())
        .all(|(source, destination)| (source * scale - destination).abs() < tolerance)
}

/// Function to check if 3 pairs of points define a valid isometry.
pub fn is_valid_isometry(
    a1: &DVec3,
    a2: &DVec3,
    a3: &DVec3,
    b1: &DVec3,
    b2: &DVec3,
    b3: &DVec3,
    tolerance: f64,
) -> bool {
    distances_match(a1, a2, a3, b1, b2, b3, tolerance)
}

/// Function to check if 3 pairs of points define a valid similarity transform.
pub fn is_valid_similarity(
    original: &[DVec3; 3],
    target: &[DVec3; 3],
    scale: f64,
    tolerance: f64,
) -> bool {
    distances_match_scaled(original, target, scale, tolerance)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;
    use crate::utils::{dump_joints, dump_solutions};

    #[test]
    fn test_find_isometry3_rotation_translation() {
        // Define points before transformation
        let p1 = DVec3::new(0.0, 0.0, 0.0);
        let p2 = DVec3::new(1.0, 0.0, 0.0);
        let p3 = DVec3::new(0.0, 1.0, 0.0);

        // Define points after 90-degree rotation around Z axis and translation by (1, 2, 0)
        let q1 = DVec3::new(1.0, 2.0, 0.0);
        let q2 = DVec3::new(1.0, 3.0, 0.0);
        let q3 = DVec3::new(0.0, 2.0, 0.0);

        // Define additional points before transformation
        let p4 = DVec3::new(1.0, 1.0, 0.0);
        let p5 = DVec3::new(2.0, 1.0, 0.0);
        let p6 = DVec3::new(1.0, 2.0, 0.0);

        // Define expected points after transformation
        let q4 = DVec3::new(0.0, 3.0, 0.0); // p4 rotated 90 degrees and translated
        let q5 = DVec3::new(0.0, 4.0, 0.0); // p5 rotated 90 degrees and translated
        let q6 = DVec3::new(-1.0, 3.0, 0.0); // p6 rotated 90 degrees and translated

        // Find the isometry using our function
        let result = Frame::frame(p1, p2, p3, q1, q2, q3)
            .expect("These points are not colinear and must be ok");

        // Expected translation
        let expected_translation = DVec3::new(1.0, 2.0, 0.0);

        // Expected rotation (90 degrees around the Z axis)
        let expected_rotation = DQuat::from_axis_angle(DVec3::Z, std::f64::consts::FRAC_PI_2);

        // Compare the result with the expected translation and rotation
        assert!((result.translation - expected_translation).length() < 1e-6);

        assert!(result.rotation.angle_between(expected_rotation) < 1e-6);

        // Check if the additional points are transformed correctly
        assert!((result.transform_point(p4) - q4).length() < 1e-6);
        assert!((result.transform_point(p5) - q5).length() < 1e-6);
        assert!((result.transform_point(p6) - q6).length() < 1e-6);
    }

    #[test]
    fn test_find_translation() {
        // Define points
        let p = DVec3::new(1.0, 2.0, 3.0);
        let q = DVec3::new(4.0, 5.0, 6.0);

        // Expected translation vector
        let expected_translation = DVec3::new(3.0, 3.0, 3.0);

        // Get the isometry
        let isometry = Frame::translation(p, q);

        // Check if the translation part is correct
        assert!((isometry.translation - expected_translation).length() < 1e-6);

        // Check if the rotation part is identity
        assert!(isometry.rotation.angle_between(DQuat::IDENTITY) < 1e-6);
    }

    #[test]
    fn test_from_tie_supports_uniform_scale() {
        let original = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, 1.0, 0.0),
        ];
        let scale = 1.25;
        let rotation = DQuat::from_axis_angle(DVec3::Z, std::f64::consts::FRAC_PI_2);
        let translation = DVec3::new(1.0, 2.0, 3.0);
        let target = original.map(|point| translation + rotation * (point * scale));

        let result = Frame::from_tie(original, target).expect("scaled tie points must be valid");

        assert!((result.scale - scale).abs() < 1e-12);
        assert!((result.translation - translation).length() < 1e-12);
        assert!(result.rotation.angle_between(rotation) < 1e-12);

        let extra_original = DVec3::new(0.4, 0.6, 0.2);
        let extra_target = translation + rotation * (extra_original * scale);
        assert!((result.transform_point(extra_original) - extra_target).length() < 1e-12);
        assert!((result.inverse_transform_point(extra_target) - extra_original).length() < 1e-12);
    }

    #[test]
    fn test_from_tie_rejects_non_uniform_scale() {
        let original = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, 1.0, 0.0),
        ];
        let target = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(2.0, 0.0, 0.0),
            DVec3::new(0.0, 3.0, 0.0),
        ];

        let error = Frame::from_tie(original, target).expect_err("non-uniform scale must fail");
        assert!(error.to_string().contains("Not similarity"));
    }

    #[test]
    fn test_scaled_frame_forward_and_inverse_transform_pose() {
        let robot = OPWKinematics::new(Parameters::irb2400_10());
        let frame_transform = FrameTransform::from_parts(
            DVec3::new(0.011, 0.022, 0.033),
            DQuat::from_axis_angle(DVec3::Z, 5.0_f64.to_radians()),
            1.02,
        );
        let framed = Frame {
            robot: Arc::new(robot),
            frame: frame_transform,
        };
        let joints = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5];
        let canonical_pose = OPWKinematics::new(Parameters::irb2400_10()).forward(&joints);

        let target_pose = framed.forward(&joints);
        let expected_pose = frame_transform.transform_pose(canonical_pose);

        assert!((target_pose.translation - expected_pose.translation).length() < 1e-10);
        let angular_distance = target_pose.angular_distance(expected_pose);
        assert!(
            angular_distance < 1e-7,
            "angular distance should be near zero, got {angular_distance}"
        );
        assert!(
            frame_transform
                .inverse_transform_pose(target_pose)
                .translation
                .distance(canonical_pose.translation)
                < 1e-10
        );
    }

    #[test]
    fn test_restore_pose() {
        let robot = OPWKinematics::new(Parameters::irb2400_10());

        // The frame is shifted by these offsets. Putting joints as reported
        // by the computed frame should result these values again.
        let dx = 0.011;
        let dy = 0.022;
        let dz = 0.033;

        // Shift not too much to have values close to previous
        let frame_transform = Frame::translation(DVec3::new(0.0, 0.0, 0.0), DVec3::new(dx, dy, dz));

        let framed = Frame {
            robot: Arc::new(robot),
            frame: frame_transform,
        };
        let joints_no_frame: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5]; // without frame

        println!("No frame transform:");
        dump_joints(&joints_no_frame);

        println!("Possible joint values after the frame transform:");
        let (solutions, _transformed_pose) =
            framed.forward_transformed(&joints_no_frame, &joints_no_frame);
        dump_solutions(&solutions);

        let framed = robot.forward(&solutions[0]).translation;
        let unframed = robot.forward(&joints_no_frame).translation;

        println!(
            "Distance between framed and not framed pose {:.3} {:.3} {:.3}",
            framed.x - unframed.x,
            framed.y - unframed.y,
            framed.z - unframed.z
        );

        let actual_dx = framed.x - unframed.x;
        let actual_dy = framed.y - unframed.y;
        let actual_dz = framed.z - unframed.z;

        assert!(
            (actual_dx - dx).abs() < 1e-6,
            "dx should be approximately {:.6}",
            dx
        );
        assert!(
            (actual_dy - dy).abs() < 1e-6,
            "dy should be approximately {:.6}",
            dy
        );
        assert!(
            (actual_dz - dz).abs() < 1e-6,
            "dz should be approximately {:.6}",
            dz
        );
    }

    #[test]
    fn test_restore_pose_isometry_shift() -> Result<(), Box<dyn Error>> {
        let robot = OPWKinematics::new(Parameters::irb2400_10());

        // The frame is shifted by these offsets. Putting joints as reported
        // by the computed frame should result these values again.
        let dx = 0.011;
        let dy = 0.022;
        let dz = 0.033;

        let angle = 0.0_f64.to_radians();
        let quaternion = DQuat::from_axis_angle(DVec3::Z, angle);

        // Shift not too much to have values close to previous. Rotate 30 degrees.
        let frame_transform = Frame::frame(
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(1.0, 1.1, 1.2),
            DVec3::new(2.0, 2.2, 2.3),
            quaternion * DVec3::new(0.0 + dx, 0.0 + dy, 0.0 + dz),
            quaternion * DVec3::new(1.0 + dx, 1.1 + dy, 1.2 + dz),
            quaternion * DVec3::new(2.0 + dx, 2.2 + dy, 2.3 + dz),
        )?;

        let framed = Frame {
            robot: Arc::new(robot),
            frame: frame_transform,
        };
        let joints_no_frame: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5]; // without frame

        println!("No frame transform:");
        dump_joints(&joints_no_frame);

        println!("Possible joint values after the frame transform:");
        let (solutions, _transformed_pose) =
            framed.forward_transformed(&joints_no_frame, &joints_no_frame);
        dump_solutions(&solutions);

        let framed = robot.forward(&solutions[0]).translation;
        let unframed = robot.forward(&joints_no_frame).translation;

        println!(
            "Distance between framed and not framed pose {:.3} {:.3} {:.3}",
            framed.x - unframed.x,
            framed.y - unframed.y,
            framed.z - unframed.z
        );

        let actual_dx = framed.x - unframed.x;
        let actual_dy = framed.y - unframed.y;
        let actual_dz = framed.z - unframed.z;

        assert!(
            (actual_dx - dx).abs() < 1e-6,
            "dx should be approximately {:.6}",
            dx
        );
        assert!(
            (actual_dy - dy).abs() < 1e-6,
            "dy should be approximately {:.6}",
            dy
        );
        assert!(
            (actual_dz - dz).abs() < 1e-6,
            "dz should be approximately {:.6}",
            dz
        );

        Ok(())
    }

    #[test]
    fn test_restore_pose_isometry_rotate() -> Result<(), Box<dyn Error>> {
        let robot = OPWKinematics::new(Parameters::irb2400_10());

        let angle = 90_f64.to_radians(); // We will rotate 90 degrees around z axis.
        let quaternion = DQuat::from_axis_angle(DVec3::Z, angle);

        // Shift not too much to have values close to previous. Rotate 30 degrees.
        let p1 = DVec3::new(0.0, 0.0, 0.0);
        let p2 = DVec3::new(1.0, 1.1, 1.2);
        let p3 = DVec3::new(2.0, 2.2, 2.3);
        let frame_transform = Frame::frame(
            p1,
            p2,
            p3,
            quaternion * p1,
            quaternion * p2,
            quaternion * p3,
        )?;

        let framed = Frame {
            robot: Arc::new(robot),
            frame: frame_transform,
        };
        let joints_no_frame: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5]; // without frame

        println!("No frame transform:");
        dump_joints(&joints_no_frame);

        println!("Possible joint values after the frame transform:");
        let (solutions, _transformed_pose) =
            framed.forward_transformed(&joints_no_frame, &joints_no_frame);
        dump_solutions(&solutions);

        let framed = robot.forward(&solutions[0]).translation;
        let unframed = robot.forward(&joints_no_frame).translation;

        println!(
            "Distance between framed and not framed pose {:.3} {:.3} {:.3} vs {:.3} {:.3} {:.3}",
            framed.x, framed.y, framed.z, unframed.x, unframed.y, unframed.z,
        );

        let actual_dx = framed.x - -unframed.y; // x becomes -y
        let actual_dy = framed.y - unframed.x; // y becomes x
        let actual_dz = framed.z - unframed.z; // z does not change as we rotate around z.

        assert!(
            (actual_dx - 0.0).abs() < 1e-6,
            "dx should be approximately {:.6}",
            0.0
        );
        assert!(
            (actual_dy - 0.0).abs() < 1e-6,
            "dy should be approximately {:.6}",
            0.0
        );
        assert!(
            (actual_dz - 0.0).abs() < 1e-6,
            "dz should be approximately {:.6}",
            0.0
        );

        Ok(())
    }
}

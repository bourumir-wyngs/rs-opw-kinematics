//! Glam-backed rigid pose and kinematic vector types.

use glam::{DQuat, DVec3, Quat, Vec3};
use std::{error::Error, fmt, ops::Mul};

/// Error returned by fallible pose constructors.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PoseError {
    message: String,
}

impl PoseError {
    fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for PoseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.message)
    }
}

impl Error for PoseError {}

/// Rigid f64 pose used by kinematics.
///
/// Constructors normalize `rotation`, and all pose operations assume the rotation remains a
/// finite unit quaternion. The fields are public for ergonomic access; if you mutate `rotation`
/// directly, you must keep it normalized.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Translation component in meters.
    pub translation: DVec3,
    /// Unit quaternion rotation component.
    ///
    /// Direct mutation must preserve a finite, normalized quaternion. Use `Pose::from_parts` to
    /// construct or re-normalize a pose from raw rotation data.
    pub rotation: DQuat,
}

/// Rigid f32 pose used by collision and visualization layers.
///
/// Constructors normalize `rotation`, and all pose operations assume the rotation remains a
/// finite unit quaternion. The fields are public for ergonomic access; if you mutate `rotation`
/// directly, you must keep it normalized.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose32 {
    /// Translation component in meters.
    pub translation: Vec3,
    /// Unit quaternion rotation component.
    ///
    /// Direct mutation must preserve a finite, normalized quaternion. Use `Pose32::from_parts` to
    /// construct or re-normalize a pose from raw rotation data.
    pub rotation: Quat,
}

/// Linear and angular velocity of a rigid body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Twist {
    /// Linear velocity in meters per second.
    pub linear: DVec3,
    /// Angular velocity in radians per second.
    pub angular: DVec3,
}

/// Force and torque applied to a rigid body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wrench {
    /// Linear force in Newtons.
    pub force: DVec3,
    /// Torque in Newton-meters.
    pub torque: DVec3,
}

impl Pose {
    /// Identity pose.
    pub const IDENTITY: Pose = Pose {
        translation: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };

    /// Returns the identity pose.
    pub const fn identity() -> Self {
        Self::IDENTITY
    }

    /// Creates a translation-only pose.
    pub fn from_translation(translation: DVec3) -> Self {
        Self::from_parts(translation, DQuat::IDENTITY)
    }

    /// Tries to create a translation-only pose.
    ///
    /// This is intended for file, Python, and other FFI boundaries where invalid input should be
    /// reported as an error instead of panicking.
    pub fn try_from_translation(translation: DVec3) -> Result<Self, PoseError> {
        Self::try_from_parts(translation, DQuat::IDENTITY)
    }

    /// Creates a pose from translation and rotation.
    ///
    /// The rotation is normalized so callers do not need to pre-normalize
    /// quaternions loaded from files or intermediate calculations. If the public
    /// `rotation` field is later mutated directly, callers must preserve this normalized
    /// quaternion invariant.
    pub fn from_parts(translation: DVec3, rotation: DQuat) -> Self {
        Self::try_from_parts(translation, rotation).expect("pose parts must be valid")
    }

    /// Tries to create a pose from translation and rotation.
    ///
    /// The rotation is normalized on success. Non-finite translation components and non-finite or
    /// zero-length rotations are returned as [`PoseError`] instead of panicking.
    pub fn try_from_parts(translation: DVec3, rotation: DQuat) -> Result<Self, PoseError> {
        validate_finite_dvec3(translation, "pose translation")?;
        Ok(Pose {
            translation,
            rotation: try_normalize_dquat(rotation, "pose rotation")?,
        })
    }

    /// Returns the inverse rigid transform.
    pub fn inverse(self) -> Self {
        let rotation = self.rotation.inverse();
        Self::from_parts(rotation * -self.translation, rotation)
    }

    /// Composes this pose with another pose.
    pub fn compose(self, other: Self) -> Self {
        Self::from_parts(
            self.transform_point(other.translation),
            self.rotation * other.rotation,
        )
    }

    /// Transforms a vector by the rotation component only.
    pub fn transform_vector(self, vector: DVec3) -> DVec3 {
        self.rotation * vector
    }

    /// Transforms a point by rotation and translation.
    pub fn transform_point(self, point: DVec3) -> DVec3 {
        self.transform_vector(point) + self.translation
    }

    /// Converts this f64 pose to the f32 pose used by rendering/collision code.
    pub fn to_f32(self) -> Pose32 {
        Pose32::from_parts(
            Vec3::new(
                self.translation.x as f32,
                self.translation.y as f32,
                self.translation.z as f32,
            ),
            Quat::from_xyzw(
                self.rotation.x as f32,
                self.rotation.y as f32,
                self.rotation.z as f32,
                self.rotation.w as f32,
            ),
        )
    }

    /// Returns the rotational distance to `other` in radians.
    pub fn angular_distance(self, other: Self) -> f64 {
        let dot = self.rotation.dot(other.rotation).abs().min(1.0);
        2.0 * dot.acos()
    }
}

impl Pose32 {
    /// Identity pose.
    pub const IDENTITY: Pose32 = Pose32 {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    };

    /// Returns the identity pose.
    pub const fn identity() -> Self {
        Self::IDENTITY
    }

    /// Creates a translation-only pose.
    pub fn from_translation(translation: Vec3) -> Self {
        Self::from_parts(translation, Quat::IDENTITY)
    }

    /// Tries to create a translation-only pose.
    ///
    /// This is intended for file, Python, and other FFI boundaries where invalid input should be
    /// reported as an error instead of panicking.
    pub fn try_from_translation(translation: Vec3) -> Result<Self, PoseError> {
        Self::try_from_parts(translation, Quat::IDENTITY)
    }

    /// Creates a pose from translation and rotation.
    ///
    /// The rotation is normalized so callers do not need to pre-normalize raw data. If the public
    /// `rotation` field is later mutated directly, callers must preserve this normalized
    /// quaternion invariant.
    pub fn from_parts(translation: Vec3, rotation: Quat) -> Self {
        Self::try_from_parts(translation, rotation).expect("pose32 parts must be valid")
    }

    /// Tries to create a pose from translation and rotation.
    ///
    /// The rotation is normalized on success. Non-finite translation components and non-finite or
    /// zero-length rotations are returned as [`PoseError`] instead of panicking.
    pub fn try_from_parts(translation: Vec3, rotation: Quat) -> Result<Self, PoseError> {
        validate_finite_vec3(translation, "pose32 translation")?;
        Ok(Pose32 {
            translation,
            rotation: try_normalize_quat(rotation, "pose32 rotation")?,
        })
    }

    /// Returns the inverse rigid transform.
    pub fn inverse(self) -> Self {
        let rotation = self.rotation.inverse();
        Self::from_parts(rotation * -self.translation, rotation)
    }

    /// Composes this pose with another pose.
    pub fn compose(self, other: Self) -> Self {
        Self::from_parts(
            self.transform_point(other.translation),
            self.rotation * other.rotation,
        )
    }

    /// Transforms a vector by the rotation component only.
    pub fn transform_vector(self, vector: Vec3) -> Vec3 {
        self.rotation * vector
    }

    /// Transforms a point by rotation and translation.
    pub fn transform_point(self, point: Vec3) -> Vec3 {
        self.transform_vector(point) + self.translation
    }
}

impl Twist {
    /// Zero twist.
    pub const ZERO: Twist = Twist {
        linear: DVec3::ZERO,
        angular: DVec3::ZERO,
    };

    /// Creates a twist from linear and angular velocity.
    pub fn new(linear: DVec3, angular: DVec3) -> Self {
        assert_finite_dvec3(linear, "twist linear velocity");
        assert_finite_dvec3(angular, "twist angular velocity");
        Twist { linear, angular }
    }
}

impl Wrench {
    /// Zero wrench.
    pub const ZERO: Wrench = Wrench {
        force: DVec3::ZERO,
        torque: DVec3::ZERO,
    };

    /// Creates a wrench from force and torque.
    pub fn new(force: DVec3, torque: DVec3) -> Self {
        assert_finite_dvec3(force, "wrench force");
        assert_finite_dvec3(torque, "wrench torque");
        Wrench { force, torque }
    }
}

impl Mul<Pose> for Pose {
    type Output = Pose;

    fn mul(self, rhs: Pose) -> Self::Output {
        self.compose(rhs)
    }
}

impl Mul<Pose32> for Pose32 {
    type Output = Pose32;

    fn mul(self, rhs: Pose32) -> Self::Output {
        self.compose(rhs)
    }
}

fn try_normalize_dquat(rotation: DQuat, name: &str) -> Result<DQuat, PoseError> {
    let length = (rotation.x * rotation.x
        + rotation.y * rotation.y
        + rotation.z * rotation.z
        + rotation.w * rotation.w)
        .sqrt();

    if !length.is_finite() || length <= 0.0 {
        return Err(PoseError::new(format!(
            "{name} must be finite and non-zero"
        )));
    }

    Ok(DQuat::from_xyzw(
        rotation.x / length,
        rotation.y / length,
        rotation.z / length,
        rotation.w / length,
    ))
}

fn try_normalize_quat(rotation: Quat, name: &str) -> Result<Quat, PoseError> {
    let length = (rotation.x * rotation.x
        + rotation.y * rotation.y
        + rotation.z * rotation.z
        + rotation.w * rotation.w)
        .sqrt();

    if !length.is_finite() || length <= 0.0 {
        return Err(PoseError::new(format!(
            "{name} must be finite and non-zero"
        )));
    }

    Ok(Quat::from_xyzw(
        rotation.x / length,
        rotation.y / length,
        rotation.z / length,
        rotation.w / length,
    ))
}

fn assert_finite_dvec3(vector: DVec3, name: &str) {
    validate_finite_dvec3(vector, name).expect("vector components must be valid")
}

fn validate_finite_dvec3(vector: DVec3, name: &str) -> Result<(), PoseError> {
    if vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite() {
        Ok(())
    } else {
        Err(PoseError::new(format!("{name} must be finite")))
    }
}

fn validate_finite_vec3(vector: Vec3, name: &str) -> Result<(), PoseError> {
    if vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite() {
        Ok(())
    } else {
        Err(PoseError::new(format!("{name} must be finite")))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1.0e-12;

    fn assert_dvec3_close(left: DVec3, right: DVec3) {
        assert!(
            (left - right).length() <= EPS,
            "left {left:?} was not close to right {right:?}"
        );
    }

    #[test]
    fn identity_preserves_points_and_vectors() {
        let pose = Pose::identity();
        let point = DVec3::new(1.0, 2.0, 3.0);
        let vector = DVec3::new(-4.0, 5.0, -6.0);

        assert_dvec3_close(pose.transform_point(point), point);
        assert_dvec3_close(pose.transform_vector(vector), vector);
    }

    #[test]
    fn constructors_normalize_rotation() {
        let rotation = DQuat::from_xyzw(0.0, 0.0, 2.0_f64.sqrt(), 2.0_f64.sqrt());
        let pose = Pose::from_parts(DVec3::ZERO, rotation);
        let length = pose.rotation.length();

        assert!((length - 1.0).abs() <= EPS);
    }

    #[test]
    fn try_from_parts_rejects_invalid_pose_values() {
        let bad_translation = Pose::try_from_parts(DVec3::new(0.0, f64::NAN, 0.0), DQuat::IDENTITY)
            .expect_err("non-finite translation must fail");
        assert!(bad_translation.to_string().contains("translation"));

        let zero_rotation = Pose::try_from_parts(DVec3::ZERO, DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0))
            .expect_err("zero rotation quaternion must fail");
        assert!(zero_rotation.to_string().contains("rotation"));

        let nan_rotation =
            Pose::try_from_parts(DVec3::ZERO, DQuat::from_xyzw(0.0, 0.0, f64::NAN, 1.0))
                .expect_err("non-finite rotation quaternion must fail");
        assert!(nan_rotation.to_string().contains("rotation"));
    }

    #[test]
    fn pose32_try_from_parts_rejects_invalid_values() {
        let bad_translation =
            Pose32::try_from_parts(Vec3::new(0.0, f32::INFINITY, 0.0), Quat::IDENTITY)
                .expect_err("non-finite translation must fail");
        assert!(bad_translation.to_string().contains("translation"));

        let zero_rotation = Pose32::try_from_parts(Vec3::ZERO, Quat::from_xyzw(0.0, 0.0, 0.0, 0.0))
            .expect_err("zero rotation quaternion must fail");
        assert!(zero_rotation.to_string().contains("rotation"));
    }

    #[test]
    fn inverse_undoes_pose() {
        let pose = Pose::from_parts(DVec3::new(1.0, 2.0, 3.0), DQuat::from_rotation_z(FRAC_PI_2));
        let point = DVec3::new(4.0, 5.0, 6.0);

        assert_dvec3_close(
            pose.inverse().transform_point(pose.transform_point(point)),
            point,
        );
    }

    #[test]
    fn composition_matches_sequential_transform() {
        let first = Pose::from_parts(DVec3::new(1.0, 0.0, 0.0), DQuat::from_rotation_z(FRAC_PI_2));
        let second = Pose::from_translation(DVec3::new(0.0, 2.0, 0.0));
        let composed = first * second;
        let point = DVec3::new(3.0, 4.0, 5.0);

        assert_dvec3_close(
            composed.transform_point(point),
            first.transform_point(second.transform_point(point)),
        );
    }

    #[test]
    fn converts_to_f32_pose() {
        let pose = Pose::from_parts(
            DVec3::new(1.25, -2.5, 3.75),
            DQuat::from_rotation_z(FRAC_PI_2),
        );
        let pose32 = pose.to_f32();

        assert_eq!(pose32.translation, Vec3::new(1.25, -2.5, 3.75));
        assert!((pose32.rotation.length() - 1.0).abs() <= 1.0e-6);
    }

    #[test]
    fn angular_distance_uses_shortest_quaternion_arc() {
        let first = Pose::identity();
        let second = Pose::from_parts(DVec3::ZERO, DQuat::from_rotation_z(FRAC_PI_2));
        let negated_second = Pose::from_parts(
            DVec3::ZERO,
            DQuat::from_xyzw(
                -second.rotation.x,
                -second.rotation.y,
                -second.rotation.z,
                -second.rotation.w,
            ),
        );

        assert!((first.angular_distance(second) - FRAC_PI_2).abs() <= EPS);
        assert!((second.angular_distance(negated_second)).abs() <= EPS);
    }

    #[test]
    fn twist_and_wrench_store_components() {
        let twist = Twist::new(DVec3::X, DVec3::Y);
        let wrench = Wrench::new(DVec3::Z, DVec3::new(1.0, 2.0, 3.0));

        assert_eq!(twist.linear, DVec3::X);
        assert_eq!(twist.angular, DVec3::Y);
        assert_eq!(wrench.force, DVec3::Z);
        assert_eq!(wrench.torque, DVec3::new(1.0, 2.0, 3.0));
    }
}

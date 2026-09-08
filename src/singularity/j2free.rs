//! Recovers the singularity where J2 is free in the arm's position equations.
//!
//! When the first two arms are the same length, `c2 = sqrt(a2^2 + c3^2)`, they can fold
//! completely at the internal angle `q3 = PI - atan2(a2, c3)`. The wrist center
//! then coincides with the shoulder, so rotating J2 leaves it in place.
//! This solver searches J2 while holding J1 and J3 fixed for the arm branch.
//! The tool orientation still depends on J2, so wrist recovery and joint
//! constraints determine which candidates can satisfy the complete target pose.

use super::continuum::{self, TrigVector};
use crate::kinematic_traits::{Joints, Pose};
use crate::kinematics_impl::{ArmBranch, OPWKinematics};
use glam::DVec3;

/// Samples free J2 at wrist boundaries and between consecutive crossings.
pub(crate) fn search(
    robot: &OPWKinematics,
    pose: &Pose,
    arm: ArmBranch,
    reference: &Joints,
    fixed_j6: Option<f64>,
) -> Vec<ArmBranch> {
    continuum::sample_angles(robot, pose, arm_axes(arm), 1, reference, fixed_j6)
        .into_iter()
        .map(|angle| ArmBranch { q2: angle, ..arm })
        .collect()
}

/// Builds analytic columns of R_arm as functions of J2, avoiding interpolation residue.
fn arm_axes(arm: ArmBranch) -> [TrigVector; 3] {
    let (s1, c1) = arm.q1.sin_cos();
    let (s3, c3) = arm.q3.sin_cos();
    let radial = DVec3::new(c1, s1, 0.0);
    [
        TrigVector {
            cosine: radial * c3 - DVec3::Z * s3,
            sine: -radial * s3 - DVec3::Z * c3,
            constant: DVec3::ZERO,
        },
        TrigVector {
            cosine: DVec3::ZERO,
            sine: DVec3::ZERO,
            constant: DVec3::new(-s1, c1, 0.0),
        },
        TrigVector {
            cosine: radial * s3 + DVec3::Z * c3,
            sine: radial * c3 - DVec3::Z * s3,
            constant: DVec3::ZERO,
        },
    ]
}

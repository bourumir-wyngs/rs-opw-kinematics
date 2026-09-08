//! Recovers the singularity where J1 is free in the arm's position equations.
//!
//! With lateral offset `b = 0` and the wrist center on the base rotation axis,
//! rotating J1 leaves the wrist center in place. J2 and J3 stay fixed for each
//! arm branch while this solver searches J1 for candidate solutions.
//! The tool orientation still depends on J1, so wrist recovery and joint
//! constraints determine which candidates can satisfy the complete target pose.

use super::continuum::{self, TrigVector};
use crate::kinematic_traits::{Joints, Pose};
use crate::kinematics_impl::{ArmBranch, OPWKinematics};
use glam::DVec3;

/// Samples free J1 at wrist boundaries and between consecutive crossings.
pub(crate) fn search(
    robot: &OPWKinematics,
    pose: &Pose,
    arm: ArmBranch,
    reference: &Joints,
    fixed_j6: Option<f64>,
) -> Vec<ArmBranch> {
    continuum::sample_angles(robot, pose, arm_axes(arm), 0, reference, fixed_j6)
        .into_iter()
        .map(|angle| ArmBranch { q1: angle, ..arm })
        .collect()
}

/// Builds analytic columns of R_arm as functions of J1, avoiding interpolation residue.
fn arm_axes(arm: ArmBranch) -> [TrigVector; 3] {
    let (s23, c23) = (arm.q2 + arm.q3).sin_cos();
    [
        TrigVector {
            cosine: DVec3::new(c23, 0.0, 0.0),
            sine: DVec3::new(0.0, c23, 0.0),
            constant: DVec3::new(0.0, 0.0, -s23),
        },
        TrigVector {
            cosine: DVec3::Y,
            sine: -DVec3::X,
            constant: DVec3::ZERO,
        },
        TrigVector {
            cosine: DVec3::new(s23, 0.0, 0.0),
            sine: DVec3::new(0.0, s23, 0.0),
            constant: DVec3::new(0.0, 0.0, c23),
        },
    ]
}

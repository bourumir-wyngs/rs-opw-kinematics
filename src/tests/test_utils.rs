use std::collections::HashMap;
use std::f64::consts::PI;
use std::path::Path;

use anyhow::{Context, Result};
use glam::{DQuat, DVec3};
use once_cell::sync::Lazy;
use serde::{Deserialize, Serialize};

use crate::pose::Pose as KinematicPose;

// ---- Domain types ----

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub(crate) struct Pose {
    /// Translation in meters: [x, y, z]
    pub translation: [f64; 3],
    /// Quaternion in [x, y, z, w] ordering
    pub quaternion: [f64; 4],
}

#[derive(Debug, Clone)]
pub struct Case {
    pub id: i32,
    pub(crate) parameters: String,
    pub(crate) joints: [f64; 6], // assumed degrees from YAML with angle_conversions
    pub(crate) solutions: Vec<[f64; 6]>, // degrees from the reference implementation
    pub(crate) pose: Pose,
}

impl Case {
    /// Returns joints converted from degrees to radians.
    #[inline]
    pub fn joints_in_radians(&self) -> [f64; 6] {
        // Slightly cleaner than a manual loop
        std::array::from_fn(|i| self.joints[i].to_radians())
    }
}

// ---- Pose conversions ----

impl Pose {
    pub fn as_pose(&self) -> KinematicPose {
        KinematicPose::from_parts(
            DVec3::new(
                self.translation[0],
                self.translation[1],
                self.translation[2],
            ),
            DQuat::from_xyzw(
                self.quaternion[0],
                self.quaternion[1],
                self.quaternion[2],
                self.quaternion[3],
            ),
        )
    }

    pub fn from_pose(pose: &KinematicPose) -> Self {
        let translation = pose.translation;
        let quaternion = pose.rotation;

        Pose {
            translation: [translation.x, translation.y, translation.z],
            quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
        }
    }
}

// ---- YAML I/O ----

#[derive(Debug, Deserialize)]
struct PoseYaml {
    translation: [f64; 3],
    quaternion: [f64; 4], // [x, y, z, w]
}

#[derive(Debug, Deserialize)]
struct CaseYaml {
    id: i32,
    parameters: String,
    joints: [f64; 6],
    solutions: Vec<[f64; 6]>,
    pose: PoseYaml,
}

#[derive(Debug, Deserialize)]
struct TestsRoot {
    cases: Vec<CaseYaml>,
}

/// Load test cases from YAML.
/// - `file_path`: path to a YAML with fields matching `TestsRoot`.
pub(crate) fn load_yaml(file_path: impl AsRef<Path>) -> Result<Vec<Case>> {
    let p = file_path.as_ref();
    let contents = std::fs::read_to_string(p)
        .with_context(|| format!("Failed to read YAML file: {}", p.display()))?;

    let opts = serde_saphyr::options! { angle_conversions: true };

    let root: TestsRoot = serde_saphyr::from_str_with_options(&contents, opts)
        .context("Failed to parse YAML with serde_saphyr")?;

    let mut cases = Vec::with_capacity(root.cases.len());
    for c in root.cases {
        cases.push(Case {
            id: c.id,
            parameters: c.parameters,
            joints: c.joints,
            solutions: c.solutions,
            pose: Pose {
                translation: c.pose.translation,
                quaternion: c.pose.quaternion,
            },
        });
    }
    Ok(cases)
}

// ---- Parameter map (static) ----

use crate::parameters::opw_kinematics::Parameters;

static PARAMS: Lazy<HashMap<&'static str, Parameters>> = Lazy::new(|| {
    HashMap::from([
        ("Irb2400_10", Parameters::irb2400_10()),
        ("KukaKR6_R700_sixx", Parameters::kuka_kr6_r700_sixx()),
        ("Fanuc_r2000ib_200r", Parameters::fanuc_r2000ib_200r()),
        ("Staubli_tx40", Parameters::staubli_tx40()),
        ("Irb2600_12_165", Parameters::irb2600_12_165()),
        ("Irb4600_60_205", Parameters::irb4600_60_205()),
        ("Staubli_tx2_140", Parameters::staubli_tx2_140()),
        ("Staubli_tx2_160", Parameters::staubli_tx2_160()),
        ("Staubli_tx2_160l", Parameters::staubli_tx2_160l()),
    ])
});

/// Create a fresh (cloned) map if mutation is needed by caller; otherwise expose a getter.
pub(crate) fn create_parameter_map() -> HashMap<String, Parameters> {
    PARAMS.iter().map(|(k, v)| (String::from(*k), *v)).collect()
}

// ---- Pose comparison ----

const TWO_PI: f64 = 2.0 * PI;

/// Compare two poses with separate tolerances.
/// - `trans_tol_m`: max allowed Euclidean distance in meters
/// - `rot_tol_rad`: max allowed rotation angle difference in radians
pub fn are_poses_close(
    a: &KinematicPose,
    b: &KinematicPose,
    trans_tol_m: f64,
    rot_tol_rad: f64,
) -> bool {
    if !trans_tol_m.is_finite()
        || trans_tol_m < 0.0
        || !rot_tol_rad.is_finite()
        || rot_tol_rad < 0.0
        || !a.translation.is_finite()
        || !b.translation.is_finite()
        || !a.rotation.is_finite()
        || !b.rotation.is_finite()
    {
        return false;
    }
    let tdiff = (a.translation - b.translation).length();
    if !tdiff.is_finite() || tdiff > trans_tol_m {
        return false;
    }
    a.angular_distance(*b) <= rot_tol_rad
}

/// Backwards-compatible wrapper using a single `tolerance` (meters & radians).
#[inline]
pub fn are_poses_approx_equal(a: &KinematicPose, b: &KinematicPose, tolerance: f64) -> bool {
    are_poses_close(a, b, tolerance, tolerance)
}

// ---- Joint solution comparison ----

use crate::kinematic_traits::Joints;

#[inline]
fn normalize_angle_rad(a: f64) -> f64 {
    // Map to (-π, π]
    let x = a.rem_euclid(TWO_PI);
    if x > PI { x - TWO_PI } else { x }
}

/// Check if `expected` exists within `solutions` within `tolerance` (radians),
/// accounting for 2π wrap.
///
/// Parameters:
/// - `solutions`: candidate joint sets to search through
/// - `expected`: the joint vector we want to find
/// - `tolerance`: absolute tolerance in radians for each joint
///
/// Returns:
/// - `Some(index)` of the matching solution, or `None` if not found
pub fn found_joints_approx_equal(
    solutions: &[Joints],
    expected: &Joints,
    tolerance: f64,
) -> Option<i32> {
    if !tolerance.is_finite() || tolerance < 0.0 || expected.iter().any(|joint| !joint.is_finite())
    {
        return None;
    }
    'outer: for (idx, sol) in solutions.iter().enumerate() {
        for j in 0..6 {
            let d = normalize_angle_rad(sol[j] - expected[j]).abs();
            if !d.is_finite() || d > tolerance {
                continue 'outer;
            }
        }
        return Some(idx as i32);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::{KinematicPose, are_poses_close, found_joints_approx_equal};
    use glam::DQuat;
    use std::f64::consts::{PI, TAU};

    #[test]
    fn pose_matching_rejects_nonfinite_components() {
        let valid = KinematicPose::IDENTITY;
        for axis in 0..3 {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut invalid = valid;
                invalid.translation[axis] = value;
                assert!(!are_poses_close(&invalid, &valid, 1e-6, 1e-6));
                assert!(!are_poses_close(&valid, &invalid, 1e-6, 1e-6));
            }
        }
        for component in 0..4 {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut invalid = valid;
                let mut quaternion = [0.0, 0.0, 0.0, 1.0];
                quaternion[component] = value;
                invalid.rotation = DQuat::from_array(quaternion);
                assert!(!are_poses_close(&invalid, &valid, 1e-6, 1e-6));
                assert!(!are_poses_close(&valid, &invalid, 1e-6, 1e-6));
            }
        }
    }

    #[test]
    fn pose_matching_rejects_invalid_tolerances() {
        let valid = KinematicPose::IDENTITY;
        for tolerance in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1e-6] {
            assert!(!are_poses_close(&valid, &valid, tolerance, 1e-6));
            assert!(!are_poses_close(&valid, &valid, 1e-6, tolerance));
        }
    }

    #[test]
    fn joint_matching_rejects_nonfinite_angles() {
        let valid = [0.0; 6];
        for joint in 0..6 {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut invalid = valid;
                invalid[joint] = value;
                assert_eq!(found_joints_approx_equal(&[invalid], &valid, 1e-6), None);
                assert_eq!(found_joints_approx_equal(&[valid], &invalid, 1e-6), None);
                assert_eq!(found_joints_approx_equal(&[invalid], &invalid, 1e-6), None);
                assert_eq!(
                    found_joints_approx_equal(&[invalid, valid], &valid, 1e-6),
                    Some(1)
                );
            }
        }
    }

    #[test]
    fn joint_matching_rejects_invalid_tolerances() {
        for tolerance in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1e-6] {
            assert_eq!(
                found_joints_approx_equal(&[[0.0; 6]], &[0.0; 6], tolerance),
                None
            );
        }
    }

    #[test]
    fn joint_matching_accepts_wrapped_angles_only_within_tolerance() {
        let expected = [0.25, -0.5, PI, -PI, 0.0, 1.0];
        let wrapped = [0.25 + TAU, -0.5 - TAU, -PI, PI, 4.0 * TAU, 1.0 - 3.0 * TAU];
        assert_eq!(
            found_joints_approx_equal(&[wrapped], &expected, 1e-12),
            Some(0)
        );

        for joint in 0..6 {
            let mut different = wrapped;
            different[joint] += 2e-6;
            assert_eq!(
                found_joints_approx_equal(&[different], &expected, 1e-6),
                None
            );
            assert_eq!(
                found_joints_approx_equal(&[different], &expected, 3e-6),
                Some(0)
            );
        }
    }
}

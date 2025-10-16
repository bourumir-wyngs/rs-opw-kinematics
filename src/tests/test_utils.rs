use std::collections::HashMap;
use std::f64::consts::PI;
use std::path::Path;

use anyhow::{Context, Result};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use once_cell::sync::Lazy;
use serde::{Deserialize, Serialize};
use serde_saphyr::Options;

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
    pub(crate) joints: [f64; 6],          // assumed degrees from YAML with angle_conversions
    pub(crate) _solutions: Vec<[f64; 6]>,  // currently not used
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
    pub fn to_isometry(&self) -> Isometry3<f64> {
        let translation = Translation3::new(self.translation[0], self.translation[1], self.translation[2]);

        // Adjusting quaternion creation to match [x, y, z, w] ordering
        let quaternion = UnitQuaternion::from_quaternion(Quaternion::new(
            self.quaternion[3], // w
            self.quaternion[0], // x
            self.quaternion[1], // y
            self.quaternion[2], // z
        ));

        Isometry3::from_parts(translation, quaternion)
    }

    pub fn from_isometry(isometry: &Isometry3<f64>) -> Self {
        let translation = isometry.translation.vector;

        // Extract the quaternion from isometry (rotation part)
        let quaternion = isometry.rotation.quaternion();

        Pose {
            translation: [translation.x, translation.y, translation.z],
            quaternion: [quaternion.i, quaternion.j, quaternion.k, quaternion.w], // [x, y, z, w] ordering
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

    let opts = Options {
        angle_conversions: true,
        ..Default::default()
    };

    let root: TestsRoot = serde_saphyr::from_str_with_options(&contents, opts)
        .context("Failed to parse YAML with serde_saphyr")?;

    let mut cases = Vec::with_capacity(root.cases.len());
    for c in root.cases {
        cases.push(Case {
            id: c.id,
            parameters: c.parameters,
            joints: c.joints,
            _solutions: c.solutions,
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
    PARAMS.iter().map(|(k, v)| (String::from(*k), v.clone())).collect()
}

// ---- Isometry comparison ----

const TWO_PI: f64 = 2.0 * PI;

/// Compare two isometries with separate tolerances.
/// - `trans_tol_m`: max allowed Euclidean distance in meters
/// - `rot_tol_rad`: max allowed rotation angle difference in radians
pub fn are_isometries_close(a: &Isometry3<f64>, b: &Isometry3<f64>, trans_tol_m: f64, rot_tol_rad: f64) -> bool {
    let tdiff = (a.translation.vector - b.translation.vector).norm();
    if tdiff > trans_tol_m {
        return false;
    }
    // Relative rotation a⁻¹ ∘ b ∈ SO(3)
    let rdiff = a.rotation.inverse() * b.rotation;
    let mut angle = rdiff.angle(); // in [0, π]
    // Be tolerant to tiny numerical drift
    if angle.is_nan() {
        angle = 0.0;
    }
    angle <= rot_tol_rad
}

/// Backwards-compatible wrapper using a single `tolerance` (meters & radians).
#[inline]
pub fn are_isometries_approx_equal(a: &Isometry3<f64>, b: &Isometry3<f64>, tolerance: f64) -> bool {
    are_isometries_close(a, b, tolerance, tolerance)
}

// ---- Joint solution comparison ----

use crate::kinematic_traits::{Joints, Solutions};

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
pub fn found_joints_approx_equal(solutions: &Solutions, expected: &Joints, tolerance: f64) -> Option<i32> {
    'outer: for (idx, sol) in solutions.iter().enumerate() {
        for j in 0..6 {
            let d = normalize_angle_rad(sol[j] - expected[j]).abs();
            if d > tolerance {
                continue 'outer;
            }
        }
        return Some(idx as i32);
    }
    None
}

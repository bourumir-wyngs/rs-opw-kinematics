//! Supports extracting OPW parameters from YAML file (optional)

use std::path::Path;

use garde::Validate;
use serde::Deserialize;
use serde_saphyr::Options;

use crate::parameter_error::ParameterError;
use crate::parameters::opw_kinematics::Parameters;

fn default_offsets() -> Vec<f64> { vec![0.0; 6] }
fn default_sign_corrections() -> Vec<i8> { vec![1; 6] }

fn opw_geometry_parameter(v: &f64, _ctx: &()) -> garde::Result {
    if !v.is_finite() {
        return Err(garde::Error::new("must be finite"));
    }
    Ok(())
}

fn joint_offset(v: &f64, _ctx: &()) -> garde::Result {
    if !v.is_finite() {
        return Err(garde::Error::new("must be finite"));
    }
    let limit = 2.0 * std::f64::consts::PI;
    if *v < -limit || *v > limit {
        return Err(garde::Error::new("must be within [-2*PI, 2*PI]"));
    }
    Ok(())
}

fn sign_correction(v: &i8, _ctx: &()) -> garde::Result {
    if *v != -1 && *v != 1 {
        return Err(garde::Error::new("must be -1 or 1"));
    }
    Ok(())
}

#[derive(Deserialize, Validate)]
struct GeometricParameters {
    #[garde(custom(opw_geometry_parameter))]
    pub a1: f64,
    #[garde(custom(opw_geometry_parameter))]
    pub a2: f64,
    #[garde(custom(opw_geometry_parameter))]
    pub b: f64,
    #[garde(custom(opw_geometry_parameter))]
    pub c1: f64,
    #[garde(custom(opw_geometry_parameter))]
    pub c2: f64,
    #[garde(custom(opw_geometry_parameter))]
    pub c3: f64,
    #[garde(custom(opw_geometry_parameter))]
    pub c4: f64,
    /// Optional here; top-level `dof` overrides if also present
    #[serde(default)]
    #[garde(range(min = 5, max = 6))]
    pub dof: Option<i8>,
}

#[derive(Deserialize, Validate)]
struct Root {
    #[garde(dive)]
    pub opw_kinematics_geometric_parameters: GeometricParameters,
    #[serde(default = "default_offsets")]
    #[garde(length(min = 5, max = 6), inner(custom(joint_offset)))]
    pub opw_kinematics_joint_offsets: Vec<f64>,
    #[serde(default = "default_sign_corrections")]
    #[garde(length(min = 5, max = 6), inner(custom(sign_correction)))]
    pub opw_kinematics_joint_sign_corrections: Vec<i8>,
    /// Optional; overrides opw_kinematics_geometric_parameters.dof if present
    #[serde(default)]
    #[garde(range(min = 5, max = 6))]
    pub dof: Option<i8>,
}

impl Parameters {
    /// Read the robot configuration from YAML file. YAML file like this is supported:
    /// ```yaml
    /// # FANUC m16ib20
    /// opw_kinematics_geometric_parameters:
    ///   a1: 0.15
    ///   a2: -0.10
    ///   b: 0.0
    ///   c1: 0.525
    ///   c2: 0.77
    ///   c3: 0.74
    ///   c4: 0.10
    /// opw_kinematics_joint_offsets: [0.0, 0.0, deg(-90.0), 0.0, 0.0, deg(180.0)]
    /// opw_kinematics_joint_sign_corrections: [1, 1, -1, -1, -1, -1]
    /// dof: 6
    /// ```
    /// Offsets, sign corrections, and DOF are optional.
    ///
    /// YAML extension to parse the deg(angle) function is supported (serde_saphyr).
    ///
    /// See e.g. ROS-Industrial fanuc_m16ib_support opw yaml.
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, ParameterError> {
        let contents = std::fs::read_to_string(path)?;
        let root: Root = serde_saphyr::from_str_with_options_valid(
            &contents,
            Options { angle_conversions: true, ..Default::default() }
        ).map_err(|e| ParameterError::ParseError(format!("{}", e)))?;

        // DOF precedence:
        // - If both present and different -> error.
        // - Else prefer top-level; else gp; else default 6.
        let dof = match (root.dof, root.opw_kinematics_geometric_parameters.dof) {
            (Some(top), Some(inner)) if top != inner => {
                return Err(ParameterError::ParseError(format!(
                    "dof appears at top-level ({}) and under geometric parameters ({}) with conflicting values",
                    top, inner
                )));
            }
            (Some(top), _) => top,
            (None, Some(inner)) => inner,
            (None, None) => 6,
        };

        // Sign corrections: allow 5 (pad with 1) or 6.
        // Value/length validation is handled by garde on `Root`.
        let mut sign_corrections = vec_to_six(root.opw_kinematics_joint_sign_corrections, 1i8)?;

        // Offsets: allow 5 (pad with 0.0) or 6.
        // Value/length validation is handled by garde on `Root`.
        let mut offsets = vec_to_six(root.opw_kinematics_joint_offsets, 0.0f64)?;

        // 5-DOF normalization: lock joint 6 to 0 (both sign correction and offset)
        if dof == 5 {
            if sign_corrections[5] != 0 {
                // Normalize to 0 to match "blocked J6"
                sign_corrections[5] = 0;
            }
            if offsets[5] != 0.0 {
                // Offset on a locked joint is misleading; normalize to 0
                offsets[5] = 0.0;
            }
        }

        let gp = &root.opw_kinematics_geometric_parameters;

        Ok(Parameters {
            a1: gp.a1,
            a2: gp.a2,
            b: gp.b,
            c1: gp.c1,
            c2: gp.c2,
            c3: gp.c3,
            c4: gp.c4,
            dof,
            offsets,
            sign_corrections,
        })
    }
}

/// Convert a vector to a 6-element array:
/// - If length is 5, pad with `pad`.
/// - If length is 6, pass-through.
/// - Otherwise, error.
fn vec_to_six<T: Copy>(
    mut v: Vec<T>,
    pad: T,
) -> Result<[T; 6], ParameterError> {
    if v.len() == 5 {
        v.push(pad);
    }
    v.try_into()
        .map_err(|v: Vec<T>| ParameterError::InvalidLength { expected: 6, found: v.len() })
}

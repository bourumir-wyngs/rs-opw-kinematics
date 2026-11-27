use crate::kinematic_traits::Pose;
use crate::parameter_error::ParameterError;
use crate::parameter_error::ParameterError::ParameterPopulationError;
use crate::parameters::opw_kinematics::Parameters;
use glam::{DMat3, DQuat, DVec3};
use std::f64::consts::FRAC_PI_2;

// Compare directions by chord length on the unit sphere, and positions in metres.
const AXIS_TOLERANCE: f64 = 1e-8;
const POSITION_TOLERANCE: f64 = 1e-8;

/// Factor six home-position joint axes into an OPW arm, a rigid base and a rigid tool.
/// `origins`, `axes` and `tip` are all expressed in the selected URDF base link.
pub(super) fn extract_geometry(
    origins: &[Pose; 6],
    axes: &[DVec3; 6],
    tip: Pose,
) -> Result<(Parameters, Pose, Pose), ParameterError> {
    // At home, Z points from shoulder to elbow and X points along the forearm.
    // Orient both from the geometry so reversing a URDF joint axis only changes
    // its sign correction. A joint origin may lie anywhere on its rotation axis.
    let mut z = axes[0].try_normalize().ok_or_else(|| {
        ParameterPopulationError("J1 must have a finite, nonzero rotation axis".into())
    })?;
    if z.dot(origins[2].translation - origins[1].translation) < 0.0 {
        z = -z;
    }
    let mut x = axes[1]
        .cross(z)
        .try_normalize()
        .ok_or_else(|| ParameterPopulationError("J1 and J2 must have perpendicular axes".into()))?;
    if x.dot(origins[4].translation - origins[2].translation) < 0.0 {
        x = -x;
    }
    let y = z.cross(x);
    let expected_axes = [z, y, y, x, y, x];
    let mut signs = [1; 6];
    for (i, (&axis, expected)) in axes.iter().zip(expected_axes).enumerate() {
        signs[i] = if axis.dot(expected) < 0.0 { -1 } else { 1 };
        let error = (axis - expected * f64::from(signs[i])).length();
        if !error.is_finite() || error > AXIS_TOLERANCE {
            return Err(ParameterPopulationError(format!(
                "J{} axis is incompatible with the supported Z,Y,Y,X,Y,X home layout (direction error {error})",
                i + 1
            )));
        }
    }

    let basis = DMat3::from_cols(x, y, z);
    let base = Pose::try_from_parts(origins[0].translation, DQuat::from_mat3(&basis))
        .map_err(|e| ParameterPopulationError(format!("Invalid OPW base frame: {e}")))?;
    let to_base = basis.transpose();
    let [_, shoulder, elbow, roll, pitch, flange] =
        origins.map(|origin| to_base * (origin.translation - base.translation));

    // In these coordinates the J4 line is (x, roll.y, roll.z) and the J5
    // line is (pitch.x, y, pitch.z). Their intersection is the wrist center.
    for (relation, distance) in [
        (
            "J2 and J3 axes must be vertically aligned at home",
            (elbow.x - shoulder.x).abs(),
        ),
        ("J4 and J5 axes must intersect", (pitch.z - roll.z).abs()),
        (
            "J4 and J6 axes must coincide at home",
            (flange.y - roll.y).hypot(flange.z - roll.z),
        ),
    ] {
        if !distance.is_finite() || distance > POSITION_TOLERANCE {
            return Err(ParameterPopulationError(format!(
                "{relation} (distance {distance} m)"
            )));
        }
    }

    // The home forearm points along X: OPW therefore uses q3 = +π/2 at
    // URDF joint zero. The solver subtracts its offset from the signed angle.
    let parameters = Parameters {
        a1: shoulder.x,
        a2: elbow.z - roll.z,
        b: roll.y,
        c1: shoulder.z,
        c2: elbow.z - shoulder.z,
        c3: pitch.x - elbow.x,
        c4: flange.x - pitch.x,
        offsets: [0.0, 0.0, -FRAC_PI_2, 0.0, 0.0, 0.0],
        sign_corrections: signs,
        dof: 6,
    };
    let upper_arm_squared = parameters.c2 * parameters.c2;
    let forearm_squared = parameters.a2 * parameters.a2 + parameters.c3 * parameters.c3;
    if !(upper_arm_squared > 0.0
        && forearm_squared > 0.0
        && (upper_arm_squared + forearm_squared).is_finite()
        && parameters.c4.is_finite())
    {
        return Err(ParameterPopulationError(
            "OPW arm lengths must be finite and both arm segments must have nonzero length".into(),
        ));
    }

    // At zero the OPW flange is at J6 with orientation Ry(+π/2). Express
    // the selected tip relative to that flange, including any fixed adapters.
    let flange_rotation = base.rotation * DQuat::from_rotation_y(FRAC_PI_2);
    let to_flange = flange_rotation.conjugate();
    let tool = Pose::try_from_parts(
        to_flange * (tip.translation - origins[5].translation),
        to_flange * tip.rotation,
    )
    .map_err(|e| ParameterPopulationError(format!("Invalid OPW tool frame: {e}")))?;
    Ok((parameters, base, tool))
}

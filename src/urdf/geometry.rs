use crate::kinematic_traits::Pose;
use crate::parameter_error::ParameterError::{self, ParameterPopulationError};
use crate::parameters::opw_kinematics::Parameters;
use glam::{DMat3, DQuat, DVec3};

// Axes are unit vectors. Dot and cross product errors below this threshold
// correspond to angular errors of approximately 1e-8 radians.
const AXIS_TOLERANCE: f64 = 1e-8;
const POSITION_TOLERANCE: f64 = 1e-8; // metres

/// Select an axis direction and the sign that maps URDF rotation into it.
fn orient_axis(axis: DVec3, toward: DVec3) -> (DVec3, i8) {
    if axis.dot(toward) < 0.0 {
        (-axis, -1)
    } else {
        (axis, 1)
    }
}

/// Factor six home-position joint axes into an OPW arm, a rigid base and a rigid tool.
/// `origins`, `axes` and `tip` are all expressed in the selected URDF base link.
pub(super) fn extract_geometry(
    origins: &[Pose; 6],
    axes: &[DVec3; 6],
    tip: Pose,
) -> Result<(Parameters, Pose, Pose), ParameterError> {
    let mut axis_residual = 0.0_f64;
    for (relation, error) in [
        (
            "J1 and J2 must be perpendicular",
            axes[0].dot(axes[1]).abs(),
        ),
        (
            "J2 and J3 must be parallel",
            axes[1].cross(axes[2]).length(),
        ),
        (
            "J3 and J4 must be perpendicular",
            axes[2].dot(axes[3]).abs(),
        ),
        (
            "J4 and J5 must be perpendicular",
            axes[3].dot(axes[4]).abs(),
        ),
        (
            "J5 and J6 must be perpendicular",
            axes[4].dot(axes[5]).abs(),
        ),
    ] {
        if !error.is_finite() || error > AXIS_TOLERANCE {
            return Err(ParameterPopulationError(format!(
                "{relation} (direction error {error})"
            )));
        }
        axis_residual = axis_residual.max(error);
    }

    // The wrist center is the point of closest approach on the J4 line to
    // the J5 line. This works even when origins are displaced along their
    // axes and the reported directions are only perpendicular to roundoff.
    let points = origins.map(|origin| origin.translation - origins[0].translation);
    let roll_to_pitch = points[4] - points[3];
    let cos45 = axes[3].dot(axes[4]);
    let distance_on_roll =
        (roll_to_pitch.dot(axes[3]) - cos45 * roll_to_pitch.dot(axes[4])) / (1.0 - cos45 * cos45);
    let center = points[3] + axes[3] * distance_on_roll;
    for joint in [4, 5] {
        let distance = (center - points[joint]).cross(axes[joint]).length();
        if !distance.is_finite() || distance > POSITION_TOLERANCE {
            return Err(ParameterPopulationError(format!(
                "J{} axis misses the spherical wrist center (distance {distance} m)",
                joint + 1
            )));
        }
    }

    // OPW's Y axis is parallel to the two arm hinges. Their origin locations
    // along Y are arbitrary, so only their X and Z coordinates locate them.
    let (z, sign1) = orient_axis(axes[0], points[2] - points[1]);
    let (x, _) = orient_axis(axes[1].cross(z).normalize(), center - points[2]);
    let y = z.cross(x);
    let basis = DMat3::from_cols(x, y, z);
    let base = Pose::try_from_parts(origins[0].translation, DQuat::from_mat3(&basis))
        .map_err(|e| ParameterPopulationError(format!("Invalid OPW base frame: {e}")))?;
    let to_base = basis.transpose();
    let [_, shoulder, elbow, _, _, flange] = points.map(|point| to_base * point);
    let wrist = to_base * center;
    let [_, _, _, axis4, axis5, axis6] = axes.map(|axis| to_base * axis);

    let upper_arm = elbow - shoulder;
    let forearm = wrist - elbow;
    let q2 = upper_arm.x.atan2(upper_arm.z);
    let (positive4, sign4) = orient_axis(axis4, forearm);
    let q23 = positive4.x.atan2(positive4.z);
    let arm_rotation = DQuat::from_rotation_y(q23);
    let forearm_dimensions = arm_rotation.conjugate() * forearm;

    // The OPW wrist is a Z-Y-Z rotation. Read the J4 and J5 home angles
    // from the following hinge axis in each preceding joint frame. J6 home
    // roll is arbitrary and is absorbed into the fixed tool rotation.
    let (pitch_in_arm, sign5) = orient_axis(arm_rotation.conjugate() * axis5, DVec3::Y);
    let q4 = (-pitch_in_arm.x).atan2(pitch_in_arm.y);
    let roll_rotation = arm_rotation * DQuat::from_rotation_z(q4);
    let (flange_in_roll, sign6) = orient_axis(roll_rotation.conjugate() * axis6, DVec3::Z);
    let q5 = flange_in_roll.x.atan2(flange_in_roll.z);
    let flange_rotation = roll_rotation * DQuat::from_rotation_y(q5);

    // The solver distinguishes b == 0 exactly when recovering a free J1.
    // A rotated URDF frame can leak a few ulps of in-plane link length into
    // the lateral projection. Limit the allowance to the measured axis
    // residual and arithmetic roundoff; exact axes preserve even tiny b.
    let c2 = upper_arm.x.hypot(upper_arm.z);
    let in_plane_reach = shoulder.x.hypot(shoulder.z) + c2 + forearm.x.hypot(forearm.z);
    let lateral_roundoff = axis_residual.min(8.0 * f64::EPSILON) * in_plane_reach;
    let (_, sign2) = orient_axis(axes[1], y);
    let (_, sign3) = orient_axis(axes[2], y);
    let parameters = Parameters {
        a1: shoulder.x,
        a2: forearm_dimensions.x,
        b: if wrist.y.abs() <= lateral_roundoff {
            0.0
        } else {
            wrist.y
        },
        c1: shoulder.z,
        c2,
        c3: forearm_dimensions.z,
        c4: (flange - wrist).dot(flange_rotation * DVec3::Z),
        // The solver subtracts this offset from the signed URDF angle.
        offsets: [0.0, -q2, q2 - q23, -q4, -q5, 0.0],
        sign_corrections: [sign1, sign2, sign3, sign4, sign5, sign6],
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

    let to_flange = (base.rotation * flange_rotation).conjugate();
    let tool = Pose::try_from_parts(
        to_flange * (tip.translation - origins[5].translation),
        to_flange * tip.rotation,
    )
    .map_err(|e| ParameterPopulationError(format!("Invalid OPW tool frame: {e}")))?;
    Ok((parameters, base, tool))
}

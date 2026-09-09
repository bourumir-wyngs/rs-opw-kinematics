//! Recovers wrist singularities where the J4 and J6 rotation axes align.
//!
//! In model coordinates (after sign corrections and offsets), J5 = 0 fixes
//! only the phase J4 + J6, while J5 = ±π fixes J4 - J6. One wrist angle remains
//! free, so recovery chooses a coupled J4/J6 pair near the reference and within
//! the joint limits. Near either pole, the caller also tries regular wrist
//! solutions and validates every candidate with forward kinematics.
//! For a five-axis robot, J6 is fixed, and tool roll is ignored, leaving J4 free
//! at either pole; its reference and limits determine the recovered angle.

use crate::constraints::Constraints;
use crate::kinematic_traits::{J4, J6, Joints};
use crate::kinematics_impl::{
    ANGULAR_TOLERANCE, J4J6Near, JOINT_DUPLICATE_THR, RotationMatrix, WristBranch, normalize_near,
    weighted_distance, wrapped_angle, wrist_limit_intervals,
};
use crate::parameters::opw_kinematics::Parameters;
use std::f64::consts::PI;

/// Allows pole candidates throughout the pose tolerance, alongside regular bends.
/// Arm recovery can amplify roundoff beyond the matrix-only floor; FK still validates candidates.
const WRIST_POLE_OVERLAP_THR: f64 = ANGULAR_TOLERANCE;

/// Allows only arithmetic roundoff when a recovered pole phase touches a limit.
pub(crate) const POLE_PHASE_ROUNDOFF_THR: f64 = 64.0 * f64::EPSILON * (1.0 + 2.0 * PI);

/// Recovers the coupled wrist phase near a pole and splits its correction near the reference.
/// The tuples hold sine/cosine of J1 and J2 + J3; `m` is a cosine of J5,
/// and `sin5` contains two estimates of its absolute sine.
pub(crate) fn recover(
    matrix: &RotationMatrix,
    (sin1, cos1): (f64, f64),
    (sin23, cos23): (f64, f64),
    m: f64,
    sin5: [f64; 2],
    near: &J4J6Near,
) -> Option<WristBranch> {
    if sin5[0] <= WRIST_POLE_OVERLAP_THR
        && sin5[1] <= WRIST_POLE_OVERLAP_THR
        && near.j4.is_finite()
        && near.j6.is_finite()
    {
        // Upper-left block of R_arm^T * R_target. Its sum/difference
        // terms remain well-conditioned when the usual atan2 pairs vanish.
        let r00 =
            matrix[(0, 0)] * cos23 * cos1 + matrix[(1, 0)] * cos23 * sin1 - matrix[(2, 0)] * sin23;
        let r01 =
            matrix[(0, 1)] * cos23 * cos1 + matrix[(1, 1)] * cos23 * sin1 - matrix[(2, 1)] * sin23;
        let r10 = matrix[(1, 0)] * cos1 - matrix[(0, 0)] * sin1;
        let r11 = matrix[(1, 1)] * cos1 - matrix[(0, 1)] * sin1;

        // At q5=0 the observable phase is q4+q6; at q5=pi it is q4-q6.
        let sign = if m >= 0.0 { 1.0 } else { -1.0 };
        let phase = (sign * r10 - r01).atan2(r11 + sign * r00);
        // Work with bounded representatives so large references cannot
        // swallow the correction or enter the caller's normalization loops.
        // inverse_continuing restores the turn count near the reference.
        let near4 = near.j4.rem_euclid(2.0 * PI);
        let near6 = near.j6.rem_euclid(2.0 * PI);
        let previous_phase = near4 + sign * near6;
        let correction = (phase - previous_phase + PI).rem_euclid(2.0 * PI) - PI;

        // Equal splitting minimizes squared wrist motion. The existing
        // L1 solution sorter can tie on other splits of the same correction.
        // Offer an exact-pole candidate; the caller still checks it with FK.
        Some(WristBranch {
            q4: near4 + correction / 2.0,
            q5: if m >= 0.0 { 0.0 } else { PI },
            q6: near6 + sign * correction / 2.0,
        })
    } else {
        // Outside the overlap, or without a finite reference, use only regular solutions.
        None
    }
}

/// Adjusts an exact-pole candidate in user coordinates to respect wrist limits and five-axis freedom.
/// `q5` is the model angle; the caller applies any fixed J6 value after adjustment.
pub(crate) fn adjust_joints(
    joints: &mut Joints,
    q5: f64,
    parameters: &Parameters,
    reference: &Joints,
    constraints: Option<&Constraints>,
    fixed_j6: Option<f64>,
) -> Option<()> {
    if q5 == 0.0 || q5.abs() == PI {
        if fixed_j6.is_some() {
            // At a five-axis pole J4 is free: the ignored tool roll must
            // not tie its choice to the provisional six-axis wrist phase.
            joints[J4] = wrapped_angle(reference[J4]);
            if let Some(constraints) = constraints {
                joints[J4] = nearest_feasible_j4(reference[J4], constraints)?;
            }
        } else if let Some(constraints) = constraints {
            let sign = if q5 == 0.0 { 1.0 } else { -1.0 }
                * parameters.sign_corrections[J4] as f64
                * parameters.sign_corrections[J6] as f64;
            let phase = wrapped_angle(joints[J4]) + sign * wrapped_angle(joints[J6]);
            let pair =
                nearest_feasible_pole(phase, sign, [reference[J4], reference[J6]], constraints)?;
            joints[J4] = pair[0];
            joints[J6] = pair[1];
        }
    }
    Some(())
}

/// Choose free five-axis J4 using the public sorter's weighted L1 distance.
fn nearest_feasible_j4(reference: f64, constraints: &Constraints) -> Option<f64> {
    let near = wrapped_angle(reference);
    let center = constraints.centers[J4];
    let margin = JOINT_DUPLICATE_THR * (1.0 + reference.abs());
    let mut best = None;
    let mut best_distance = f64::INFINITY;
    let mut best_motion = f64::INFINITY;
    for (lower, upper) in wrist_limit_intervals(center, constraints.tolerances[J4], near, 1.0) {
        // The score is piecewise linear on each feasible interval. Its only
        // kinks occur at the previous value and the raw constraint center.
        for candidate in [0.0, center - reference, lower, upper] {
            let displacement = candidate.clamp(lower, upper);
            let inward = displacement
                + (lower + (upper - lower) / 2.0 - displacement).clamp(-margin, margin);
            // Also try just inside an endpoint: exact +pi normalizes to -pi,
            // and constraint-boundary rounding can reject the exact endpoint.
            for displacement in [displacement, inward] {
                let angle = wrapped_angle(near + displacement);
                let mut joints = constraints.centers;
                joints[J4] = angle;
                if !angle.is_finite() || !constraints.compliant(&joints) {
                    continue;
                }
                // Score and check the actual representation continuation will
                // return, retaining the caller's original reference and turns.
                normalize_near(&mut joints[J4], reference);
                if !joints[J4].is_finite() || !constraints.compliant(&joints) {
                    continue;
                }
                let distance = weighted_distance(
                    &[joints[J4]],
                    &[reference],
                    &[center],
                    constraints.sorting_weight,
                );
                let motion = (joints[J4] - reference).abs();
                let roundoff =
                    64.0 * f64::EPSILON * (1.0 + distance.abs().max(best_distance.abs()));
                if best.is_none()
                    || distance < best_distance - roundoff
                    || ((distance - best_distance).abs() <= roundoff && motion < best_motion)
                {
                    best = Some(angle);
                    best_distance = distance;
                    best_motion = motion;
                }
            }
        }
    }
    best
}

/// Intersects J4 + sign*J6 = phase (modulo 2*pi) with both circular limits,
/// choosing the feasible pair using the public sorter's weighted L1 distance.
fn nearest_feasible_pole(
    phase: f64,
    sign: f64,
    reference: [f64; 2],
    constraints: &Constraints,
) -> Option<[f64; 2]> {
    let near = reference.map(wrapped_angle);
    let correction = wrapped_angle(phase - near[0] - sign * near[1]);
    let intervals4 = wrist_limit_intervals(
        constraints.centers[J4],
        constraints.tolerances[J4],
        near[0],
        1.0,
    );
    let intervals6 = wrist_limit_intervals(
        constraints.centers[J6],
        constraints.tolerances[J6],
        near[1],
        sign,
    );

    // Constraint comparisons are inclusive but exact. Check the actual angle
    // representations used by inverse and inverse_continuing so conversion
    // roundoff cannot turn a feasible endpoint into a rejected solution.
    let compliant = |pair: [f64; 2]| {
        let mut joints = constraints.centers;
        for (joint, angle) in [J4, J6].into_iter().zip(pair) {
            joints[joint] = angle;
            while joints[joint] > PI {
                joints[joint] -= 2.0 * PI;
            }
            while joints[joint] < -PI {
                joints[joint] += 2.0 * PI;
            }
        }
        if !constraints.compliant(&joints) {
            return None;
        }
        normalize_near(&mut joints[J4], reference[0]);
        normalize_near(&mut joints[J6], reference[1]);
        constraints
            .compliant(&joints)
            .then_some([joints[J4], joints[J6]])
    };

    let mut best = None;
    let mut best_distance = f64::INFINITY;
    let mut best_motion = f64::INFINITY;
    let centers = [constraints.centers[J4], constraints.centers[J6]];
    let inward = |value: f64, lower: f64, upper: f64, margin: f64| {
        value + (lower + (upper - lower) / 2.0 - value).clamp(-margin, margin)
    };
    let margin = JOINT_DUPLICATE_THR * (1.0 + reference[0].abs().max(reference[1].abs()));
    for (lower4, upper4) in intervals4 {
        for (lower6, upper6) in intervals6.clone() {
            // x = J4 - near4 and y = sign*(J6 - near6) lie in [-pi, pi].
            // Their sum can require either neighboring phase winding.
            for winding in [-1.0, 0.0, 1.0] {
                let sum = correction + winding * 2.0 * PI;
                let lower = lower4.max(sum - upper6);
                let upper = upper4.min(sum - lower6);
                // A phase recovered with roundoff can miss a touching corner
                // by a few ulps. Keep that corner eligible without widening
                // either joint limit; check the final phase residual below.
                if lower - upper > POLE_PHASE_ROUNDOFF_THR {
                    continue;
                }
                // Along y=sum-x the weighted L1 score is piecewise linear.
                // Its minimum lies at an endpoint or where a joint equals its
                // reference or center. Keep the balanced split for tied scores.
                // Centers stay in user coordinates, as in the public sorter.
                let candidates = [
                    sum / 2.0,
                    lower,
                    upper,
                    0.0,
                    sum,
                    centers[0] - reference[0],
                    sum - sign * (centers[1] - reference[1]),
                    // At the turn cut, +pi normalizes to -pi. A point just
                    // inside can have a better distance to the raw centers.
                    inward(lower, lower, upper, margin),
                    inward(upper, lower, upper, margin),
                ];
                for candidate in candidates {
                    let mut x = candidate
                        .clamp(lower.min(upper), lower.max(upper))
                        .clamp(lower4, upper4);
                    let mut y = (sum - x).clamp(lower6, upper6);
                    let make_pair = |x, y| [near[0] + x, near[1] + sign * y];
                    let mut pair = make_pair(x, y);
                    if compliant(pair).is_none() && lower < upper {
                        // Move a rejected endpoint slightly inside its feasible
                        // segment, preserving the phase whenever possible.
                        x = inward(x, lower, upper, margin);
                        y = sum - x;
                        pair = make_pair(x, y);
                    }
                    if compliant(pair).is_none() {
                        // At a touching corner, rounding may put both joints just
                        // outside. Nudge each inward by only a bounded roundoff
                        // amount; strict constraints and FK validation still apply.
                        let margin = POLE_PHASE_ROUNDOFF_THR / 4.0;
                        x = inward(x, lower4, upper4, margin);
                        y = inward(y, lower6, upper6, margin);
                        pair = make_pair(x, y);
                    }
                    let Some(normalized) = compliant(pair) else {
                        continue;
                    };
                    if (x + y - sum).abs() > POLE_PHASE_ROUNDOFF_THR {
                        continue;
                    }
                    let distance = weighted_distance(
                        &normalized,
                        &reference,
                        &centers,
                        constraints.sorting_weight,
                    );
                    // Preserve balanced motion only when the requested scores
                    // tie within arithmetic roundoff.
                    let motion = x * x + y * y;
                    let roundoff =
                        64.0 * f64::EPSILON * (1.0 + distance.abs().max(best_distance.abs()));
                    if best.is_none()
                        || distance < best_distance - roundoff
                        || ((distance - best_distance).abs() <= roundoff && motion < best_motion)
                    {
                        best = Some(pair);
                        best_distance = distance;
                        best_motion = motion;
                    }
                }
            }
        }
    }
    best
}

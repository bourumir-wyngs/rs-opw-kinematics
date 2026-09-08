//! Shared angle sampling for the J1-free and J2-free singular arm families.
//!
//! J1 is free when the wrist center lies on the base rotation axis. J2 is free
//! when equal effective arm lengths fold back onto the shoulder. This sampler
//! handles either free angle; the combined J1/J2 solver also uses it through
//! the J2 solver at fixed J1 slices. Wrist orientation and joint limits guide
//! the choice of candidate angles.
//!
//! With either J1 or J2 free, every entry of the relative wrist rotation has
//! the form `a*cos(t) + b*sin(t) + c`. Wrist-limit crossings can therefore be
//! found analytically. Between consecutive crossings, a regular wrist branch
//! cannot enter or leave its allowed ranges. Sampling each such interval
//! avoids the arbitrarily narrow possible intervals that a fixed grid misses.

use crate::kinematic_traits::{Joints, Kinematics, Pose};
use crate::kinematics_impl::{OPWKinematics, wrapped_angle, wrist_limit_intervals};
use glam::{DMat3, DVec3};
use std::f64::consts::PI;

const TWO_PI: f64 = 2.0 * PI;
const ROOT_ROUNDOFF: f64 = 64.0 * f64::EPSILON;

/// A first-harmonic polynomial of the free model angle.
#[derive(Clone, Copy)]
struct Trig {
    /// Coefficient of the free angle's cosine.
    cosine: f64,
    /// Coefficient of the free angle's sine.
    sine: f64,
    /// Angle-independent term.
    constant: f64,
}

impl Trig {
    /// Multiplies every coefficient by the given factor.
    fn scaled(self, factor: f64) -> Self {
        Self {
            cosine: self.cosine * factor,
            sine: self.sine * factor,
            constant: self.constant * factor,
        }
    }

    /// Adds two polynomials coefficient by coefficient.
    fn plus(self, other: Self) -> Self {
        Self {
            cosine: self.cosine + other.cosine,
            sine: self.sine + other.sine,
            constant: self.constant + other.constant,
        }
    }

    /// Appends wrapped angles where the polynomial equals the target.
    fn roots(self, target: f64, roots: &mut Vec<f64>) {
        let constant = self.constant - target;
        let radius = self.cosine.hypot(self.sine);
        let error = ROOT_ROUNDOFF
            * (self.cosine.abs() + self.sine.abs() + self.constant.abs() + target.abs());
        if !radius.is_finite()
            || !constant.is_finite()
            || radius == 0.0
            || constant.abs() > radius + error
        {
            // An identically zero equation places no additional boundary.
            // Its entire interval is sampled by the other boundary equations.
            return;
        }
        let phase = self.sine.atan2(self.cosine);
        let offset = (-constant / radius).clamp(-1.0, 1.0).acos();
        roots.push(wrapped_angle(phase - offset));
        roots.push(wrapped_angle(phase + offset));
    }
}

/// A vector whose components are first-harmonic polynomials of the free angle.
#[derive(Clone, Copy)]
pub(super) struct TrigVector {
    /// Vector coefficient of the free angle's cosine.
    pub(super) cosine: DVec3,
    /// Vector coefficient of the free angle's sine.
    pub(super) sine: DVec3,
    /// Angle-independent vector term.
    pub(super) constant: DVec3,
}

impl TrigVector {
    /// Projects the vector polynomial onto a fixed vector.
    fn dot(self, vector: DVec3) -> Trig {
        Trig {
            cosine: self.cosine.dot(vector),
            sine: self.sine.dot(vector),
            constant: self.constant.dot(vector),
        }
    }
}

/// Appends angles where the vector `(x, y)` lies on an atan2 boundary line.
/// Both directions of an atan2 boundary ray are intentional: the opposite
/// direction belongs to the flipped Euler representation of the same wrist.
fn ray_roots(x: Trig, y: Trig, angle: f64, roots: &mut Vec<f64>) {
    let (sine, cosine) = angle.sin_cos();
    y.scaled(cosine).plus(x.scaled(-sine)).roots(0.0, roots);
}

/// Appends wrist-bend boundary crossings, preserving narrow ranges near poles.
fn bend_roots(cosine: Trig, x: Trig, y: Trig, angle: f64, roots: &mut Vec<f64>) {
    cosine.roots(angle.cos(), roots);
    let sine = angle.sin().abs();
    if sine > 0.0 && sine < 1.0e-4 {
        // Close to a pole, cos(bound) can round to exactly +/-1 although the
        // allowed bend is nonzero. Keep those narrow ranges by solving the
        // two atan2 components' squared norm instead. Compensated polynomial
        // arithmetic preserves the small squared sine during root isolation.
        roots.extend(super::j1j2free::squared_norm_roots(
            [x.constant, x.cosine, x.sine],
            [y.constant, y.cosine, y.sine],
            sine,
        ));
    }
}

/// Samples a free joint's angles at wrist boundaries and between consecutive crossings.
pub(super) fn sample_angles(
    robot: &OPWKinematics,
    pose: &Pose,
    axes: [TrigVector; 3],
    free_joint: usize,
    reference: &Joints,
    fixed_j6: Option<f64>,
) -> Vec<f64> {
    let parameters = &robot.parameters;
    let model_angle = |joint: usize, angle: f64| {
        wrapped_angle(
            wrapped_angle(angle) * parameters.sign_corrections[joint] as f64
                - wrapped_angle(parameters.offsets[joint]),
        )
    };
    let near = model_angle(free_joint, reference[free_joint]);
    if !near.is_finite() {
        return Vec::new();
    }

    let [axis_x, axis_y, axis_z] = axes;
    let matrix = DMat3::from_quat(pose.rotation);
    let q4_x = axis_x.dot(matrix.z_axis);
    let q4_y = axis_y.dot(matrix.z_axis);
    let q6_x = axis_z.dot(matrix.x_axis).scaled(-1.0);
    let q6_y = axis_z.dot(matrix.y_axis);
    let cos5 = axis_z.dot(matrix.z_axis);
    let mut roots = vec![near];

    // Include poles explicitly because the regular angle pairs vanish there.
    cos5.roots(1.0, &mut roots);
    cos5.roots(-1.0, &mut roots);

    let mut limits = [Vec::new(), Vec::new(), Vec::new()];
    for joint in 3..=5 {
        if joint == 5 && fixed_j6.is_some() {
            // Five-axis IK constrains tool direction only. Target roll and the
            // provisional six-axis q6 must not restrict this arm continuum.
            continue;
        }
        let mut angles = vec![model_angle(joint, reference[joint])];
        if let Some(constraints) = robot.constraints() {
            let center = model_angle(joint, constraints.centers[joint]);
            angles.push(center);
            let tolerance = constraints.tolerances[joint];
            if tolerance.is_finite() && (0.0..PI).contains(&tolerance) {
                limits[joint - 3] = vec![center - tolerance, center + tolerance];
                angles.extend_from_slice(&limits[joint - 3]);
            }
        }
        for angle in angles.into_iter().filter(|angle| angle.is_finite()) {
            match joint {
                3 => ray_roots(q4_x, q4_y, angle, &mut roots),
                4 => bend_roots(cos5, q4_x, q4_y, angle, &mut roots),
                5 => ray_roots(q6_x, q6_y, angle, &mut roots),
                _ => unreachable!(),
            }
        }
    }

    if fixed_j6.is_none() {
        // Some free-arm curves stay at a wrist pole throughout an interval.
        // Their feasible boundary is a sum/difference of J4 and J6 endpoints,
        // rather than an individual atan2 limit. If either joint is unlimited,
        // every phase has a feasible split and no constraint boundary is needed.
        for sign in [1.0, -1.0] {
            let phase_y = axis_y
                .dot(matrix.x_axis)
                .scaled(sign)
                .plus(axis_x.dot(matrix.y_axis).scaled(-1.0));
            let phase_x = axis_y
                .dot(matrix.y_axis)
                .plus(axis_x.dot(matrix.x_axis).scaled(sign));
            ray_roots(
                phase_x,
                phase_y,
                model_angle(3, reference[3]) + sign * model_angle(5, reference[5]),
                &mut roots,
            );
            for &limit4 in &limits[0] {
                for &limit6 in &limits[2] {
                    ray_roots(phase_x, phase_y, limit4 + sign * limit6, &mut roots);
                }
            }
        }
    }

    let (center, tolerance) = robot.constraints().map_or((0.0, PI), |constraints| {
        (
            model_angle(free_joint, constraints.centers[free_joint]),
            constraints.tolerances[free_joint],
        )
    });
    roots.push(center);
    let mut result = Vec::new();
    for (lower, upper) in wrist_limit_intervals(center, tolerance, near, 1.0) {
        let lower = near + lower;
        let upper = near + upper;
        let mut cuts = vec![lower, upper];
        for &root in roots.iter().filter(|root| root.is_finite()) {
            for winding in [-1.0, 0.0, 1.0] {
                let angle = root + winding * TWO_PI;
                if lower <= angle && angle <= upper {
                    cuts.push(angle);
                }
            }
        }
        cuts.sort_unstable_by(f64::total_cmp);
        // Do not merge distinct nearby roots: a narrow valid interval can lie
        // between them even when its width is much smaller than a sampling step.
        cuts.dedup();
        let mut samples = cuts.clone();
        for interval in cuts.windows(2) {
            let [lower, upper] = [interval[0], interval[1]];
            let width = upper - lower;
            let inset = (ROOT_ROUNDOFF * (1.0 + lower.abs().max(upper.abs()))).min(width / 4.0);
            // Endpoints cover touching feasible sets; inward samples avoid an
            // inclusive constraint being lost to the last few ulps of recovery.
            samples.extend([lower + inset, lower + width / 2.0, upper - inset]);
        }
        result.extend(samples);
    }
    // Joint conversion, pole fitting, FK, limits, and final ranking are shared
    // with the ordinary arm branches in the caller.
    result
}

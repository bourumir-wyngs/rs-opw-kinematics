//! Combined singularity: a fully folded arm leaves both J1 and J2 free.
//!
//! With equal arm lengths and the shoulder on the base rotation axis
//! (`a1 = 0`, `b = 0`), the wrist center folds onto that axis. Changing J1 or J2
//! then leaves the wrist center unchanged; J3 stays at its folded angle.
//! Wrist orientation and joint limits still restrict the possible solutions.
//!
//! Wrist-limit crossings are curves A(q1) cos(t) + B(q1) sin(t) + C(q1) = 0,
//! where t = q2 + q3 and A/B/C are linear sinusoids. Their intersections and
//! vertical tangencies partition J1 into slices with unchanged feasibility
//! topology. Testing those slices with the J2-free solver avoids a sampling
//! grid that could step over a narrow possible component.

use super::j2free;
use crate::kinematic_traits::{Joints, Kinematics, Pose};
use crate::kinematics_impl::{ArmBranch, OPWKinematics, wrapped_angle};
use glam::DMat3;
use std::f64::consts::PI;

/// Constant, cosine, and sine coefficients of a first harmonic in J1.
type Trig = [f64; 3];
/// Compensated coefficients ordered from the constant term to the highest power.
type Polynomial = Vec<Compensated>;

/// A number represented by a rounded value and its residual rounding error.
///
/// Two-component arithmetic keeps elimination from erasing a small possible
/// interval when nearby wrist-limit curves produce nearly equal products.
/// Coefficients and evaluation stay compensated; angular root intervals use f64.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Compensated {
    /// Rounded main value.
    high: f64,
    /// Residual correction carried through subsequent arithmetic.
    low: f64,
}

impl From<f64> for Compensated {
    /// Wraps an `f64` with an initially zero residual.
    fn from(high: f64) -> Self {
        Self { high, low: 0.0 }
    }
}

impl Compensated {
    /// Adds both components while retaining the residual rounding error.
    fn plus(self, other: Self) -> Self {
        let sum = self.high + other.high;
        let virtual_other = sum - self.high;
        let error = (self.high - (sum - virtual_other))
            + (other.high - virtual_other)
            + self.low
            + other.low;
        let high = sum + error;
        Self {
            high,
            low: error - (high - sum),
        }
    }

    /// Multiplies both components while retaining the residual rounding error.
    pub(crate) fn times(self, other: Self) -> Self {
        let product = self.high * other.high;
        let error = self.high.mul_add(other.high, -product)
            + self.high * other.low
            + self.low * other.high
            + self.low * other.low;
        let high = product + error;
        Self {
            high,
            low: error - (high - product),
        }
    }

    /// Rounds the combined main value and residual to a single `f64`.
    fn value(self) -> f64 {
        self.high + self.low
    }
    /// Checks whether both components are exactly zero.
    fn is_zero(self) -> bool {
        self.high == 0.0 && self.low == 0.0
    }
}

/// A boundary `A(q1) * cos(t) + B(q1) * sin(t) + C(q1) = 0`, with `t = q2 + q3`.
#[derive(Clone, Copy)]
struct Curve {
    /// J1 harmonic multiplying `cos(t)`.
    a: Trig,
    /// J1 harmonic multiplying `sin(t)`.
    b: Trig,
    /// J1 harmonic independent of `t`.
    c: Trig,
}

impl Curve {
    /// Converts the J1 harmonics to polynomial numerators in `u = tan((q1 - origin) / 2)`.
    /// Each numerator shares the denominator `1 + u^2`.
    fn polynomials(self, origin: f64) -> [Polynomial; 3] {
        [self.a, self.b, self.c].map(|[constant, cosine, sine]| {
            let (s, c) = origin.sin_cos();
            let rotated_cosine = cosine * c + sine * s;
            let rotated_sine = -cosine * s + sine * c;
            vec![
                Compensated::from(constant).plus(rotated_cosine.into()),
                (2.0 * rotated_sine).into(),
                Compensated::from(constant).plus((-rotated_cosine).into()),
            ]
        })
    }
}

/// Returns a continuous model-angle interval for a joint, or `None` for invalid limits.
fn limits(robot: &OPWKinematics, joint: usize, reference: &Joints) -> Option<[f64; 2]> {
    let sign = robot.parameters.sign_corrections[joint] as f64;
    let offset = robot.parameters.offsets[joint];
    let (center, tolerance) = if let Some(constraints) = robot.constraints() {
        (constraints.centers[joint], constraints.tolerances[joint])
    } else {
        (reference[joint], PI)
    };
    if tolerance.is_nan() || tolerance < 0.0 || !center.is_finite() {
        return None;
    }
    let center = wrapped_angle(wrapped_angle(center) * sign - offset);
    let half_width = tolerance.min(PI);
    Some([center - half_width, center + half_width])
}

/// Adds corresponding coefficients of two J1 harmonics.
fn plus(a: Trig, b: Trig) -> Trig {
    std::array::from_fn(|i| a[i] + b[i])
}

/// Scales every coefficient of a J1 harmonic.
fn times(a: Trig, factor: f64) -> Trig {
    a.map(|coefficient| coefficient * factor)
}

/// Builds curves for J2 limits, wrist limits, wrist poles, and coupled pole-phase limits.
fn boundaries(
    robot: &OPWKinematics,
    pose: &Pose,
    q3: f64,
    reference: &Joints,
    fixed_j6: Option<f64>,
) -> Vec<Curve> {
    let matrix = DMat3::from_quat(pose.rotation);
    let x = matrix.x_axis;
    let y = matrix.y_axis;
    let z = matrix.z_axis;
    let ax = [0.0, x.x, x.y];
    let ay = [0.0, y.x, y.y];
    let az = [0.0, z.x, z.y];
    let yx = [0.0, x.y, -x.x];
    let yy = [0.0, y.y, -y.x];
    let yz = [0.0, z.y, -z.x];
    let zero = [0.0; 3];
    let mut curves = Vec::new();

    // A feasibility component can enter or leave the allowed J2 interval only
    // through one of its boundaries. Opposite rays add harmless extra cuts.
    if let Some(range) = limits(robot, 1, reference) {
        for angle in range {
            let (s, c) = (angle + q3).sin_cos();
            curves.push(Curve {
                a: [-s, 0.0, 0.0],
                b: [c, 0.0, 0.0],
                c: zero,
            });
        }
    }

    // Poles remain relevant without wrist limits: atan2 changes representation
    // there, and the pole's coupled phase must be handled by the 1D solver.
    for cosine in [-1.0, 1.0] {
        curves.push(Curve {
            a: [z.z, 0.0, 0.0],
            b: az,
            c: [-cosine, 0.0, 0.0],
        });
    }
    for joint in 3..6 {
        if joint == 5 && fixed_j6.is_some() {
            continue;
        }
        let Some(constraints) = robot.constraints() else {
            continue;
        };
        if constraints.tolerances[joint] >= PI {
            continue;
        }
        let Some(range) = limits(robot, joint, reference) else {
            continue;
        };
        for angle in range {
            let (s, c) = angle.sin_cos();
            curves.push(match joint {
                3 => Curve {
                    a: times(az, -s),
                    b: [z.z * s, 0.0, 0.0],
                    c: times(yz, c),
                },
                4 => Curve {
                    a: [z.z, 0.0, 0.0],
                    b: az,
                    c: [-c, 0.0, 0.0],
                },
                5 => Curve {
                    a: [y.z * c + x.z * s, 0.0, 0.0],
                    b: plus(times(ay, c), times(ax, s)),
                    c: zero,
                },
                _ => unreachable!(),
            });
        }
    }

    // With a vertical target axis, a pole persists along a whole J1 interval.
    // Ordinary wrist ray equations then vanish and cannot locate the endpoints
    // of its feasible phase range. Add q4 +/- q6 corner phases explicitly.
    if fixed_j6.is_none()
        && let Some(constraints) = robot.constraints()
        && constraints.tolerances[3] < PI
        && constraints.tolerances[5] < PI
        && let (Some(range4), Some(range6)) =
            (limits(robot, 3, reference), limits(robot, 5, reference))
    {
        for sign in [-1.0, 1.0] {
            for q4 in range4 {
                for q6 in range6 {
                    let (s, c) = (q4 + sign * q6).sin_cos();
                    curves.push(Curve {
                        a: plus(times(ay, -c), times(ax, -sign * s)),
                        b: [y.z * c + sign * x.z * s, 0.0, 0.0],
                        c: plus(times(yx, sign * c), times(yy, -s)),
                    });
                }
            }
        }
    }
    curves
}

/// Computes `a + scale * b` with compensated polynomial coefficients.
pub(crate) fn add(a: &[Compensated], b: &[Compensated], scale: f64) -> Polynomial {
    let mut result = vec![Compensated::default(); a.len().max(b.len())];
    for (i, &value) in a.iter().enumerate() {
        result[i] = result[i].plus(value);
    }
    for (i, &value) in b.iter().enumerate() {
        result[i] = result[i].plus(value.times(scale.into()));
    }
    result
}

/// Multiplies two polynomials by convolving their compensated coefficients.
pub(crate) fn multiply(a: &[Compensated], b: &[Compensated]) -> Polynomial {
    let mut result = vec![Compensated::default(); a.len() + b.len() - 1];
    for (i, &x) in a.iter().enumerate() {
        for (j, &y) in b.iter().enumerate() {
            result[i + j] = result[i + j].plus(x.times(y));
        }
    }
    result
}

/// Computes the polynomial difference `a * b - c * d` used to eliminate curve variables.
fn determinant(
    a: &[Compensated],
    b: &[Compensated],
    c: &[Compensated],
    d: &[Compensated],
) -> Polynomial {
    add(&multiply(a, b), &multiply(c, d), -1.0)
}

/// Evaluates a polynomial with compensated Horner arithmetic, then rounds to `f64`.
fn evaluate(polynomial: &[Compensated], x: f64) -> f64 {
    polynomial
        .iter()
        .rev()
        .fold(Compensated::default(), |result, &coefficient| {
            result.times(x.into()).plus(coefficient)
        })
        .value()
}

/// Checks whether the residual at `x` fits a coefficient-scaled compensated roundoff bound.
fn near_zero(polynomial: &[Compensated], x: f64) -> bool {
    let scale = polynomial.iter().rev().fold(0.0, |result, coefficient| {
        result * x.abs() + coefficient.high.abs() + coefficient.low.abs()
    });
    evaluate(polynomial, x).abs() <= 128.0 * f64::EPSILON * f64::EPSILON * scale
}

/// Isolate real roots on a bounded chart. Derivative roots split a polynomial
/// into monotone intervals; sign changes find simple roots and a roundoff
/// residual at a stationary point retains repeated roots. Only exactly zero
/// leading coefficients are removed, preserving small but resolvable features.
pub(crate) fn roots(mut polynomial: Polynomial, lower: f64, upper: f64) -> Vec<f64> {
    while polynomial
        .last()
        .is_some_and(|coefficient| coefficient.is_zero())
    {
        polynomial.pop();
    }
    if polynomial.len() <= 1
        || polynomial
            .iter()
            .any(|value| !value.high.is_finite() || !value.low.is_finite())
    {
        return Vec::new();
    }
    if polynomial.len() == 2 {
        let root = -polynomial[0].value() / polynomial[1].value();
        return if root >= lower && root <= upper {
            vec![root]
        } else {
            Vec::new()
        };
    }
    let derivative = polynomial
        .iter()
        .enumerate()
        .skip(1)
        .map(|(i, value)| value.times((i as f64).into()))
        .collect();
    let mut partitions = vec![lower];
    partitions.extend(roots(derivative, lower, upper));
    partitions.push(upper);
    partitions.sort_by(f64::total_cmp);
    partitions.dedup();
    let mut result: Vec<f64> = partitions
        .iter()
        .copied()
        .filter(|&point| near_zero(&polynomial, point))
        .collect();
    for pair in partitions.windows(2) {
        let [mut lo, mut hi] = [pair[0], pair[1]];
        let mut flo = evaluate(&polynomial, lo);
        let fhi = evaluate(&polynomial, hi);
        if flo == 0.0 || fhi == 0.0 || flo.is_sign_positive() == fhi.is_sign_positive() {
            continue;
        }
        // The bounded tan-half-angle chart keeps this finite even for roots
        // beside +/-pi. Stop when no representable midpoint remains.
        loop {
            let middle = lo + (hi - lo) / 2.0;
            if middle == lo || middle == hi {
                break;
            }
            let value = evaluate(&polynomial, middle);
            if value == 0.0 {
                lo = middle;
                hi = middle;
                break;
            }
            if value.is_sign_positive() == flo.is_sign_positive() {
                lo = middle;
                flo = value;
            } else {
                hi = middle;
            }
        }
        result.push(lo + (hi - lo) / 2.0);
    }
    result.sort_by(f64::total_cmp);
    result.dedup();
    result
}

/// Roots of x(angle)^2 + y(angle)^2 = norm^2 without subtracting a
/// near-unit cosine from one. Triples are constant/cosine/sine coefficients.
/// This also serves the one-angle solver's tiny nonzero wrist bends.
pub(crate) fn squared_norm_roots(x: [f64; 3], y: [f64; 3], norm: f64) -> Vec<f64> {
    let mut result = Vec::new();
    let denominator = vec![1.0.into(), 0.0.into(), 1.0.into()];
    let norm_squared = Compensated::from(norm).times(norm.into());
    let target: Polynomial = multiply(&denominator, &denominator)
        .into_iter()
        .map(|value| value.times(norm_squared))
        .collect();
    for [lower, upper] in [[-PI, 0.0], [0.0, PI]] {
        let origin = lower + (upper - lower) / 2.0;
        let [x, y, _] = Curve {
            a: x,
            b: y,
            c: [0.0; 3],
        }
        .polynomials(origin);
        let polynomial = add(
            &add(&multiply(&x, &x), &multiply(&y, &y), 1.0),
            &target,
            -1.0,
        );
        result.extend(
            roots(polynomial, -1.0, 1.0)
                .into_iter()
                .map(|root| wrapped_angle(origin + 2.0 * root.atan())),
        );
    }
    result.sort_by(f64::total_cmp);
    result.dedup();
    result
}

/// A near-pole cosine differs from +/-1 by less than matrix roundoff.
/// Recover its critical slices from transverse sine components instead. Each
/// true boundary interaction is either a J4/J5, J6/J5, or J2/J5 intersection;
/// the remaining extrema occur where the transverse J4 component is maximal.
fn bend_slices(
    robot: &OPWKinematics,
    pose: &Pose,
    q3: f64,
    reference: &Joints,
    fixed_j6: Option<f64>,
) -> Vec<f64> {
    let Some(constraints) = robot.constraints() else {
        return Vec::new();
    };
    if constraints.tolerances[4] >= PI {
        return Vec::new();
    }
    let Some(range5) = limits(robot, 4, reference) else {
        return Vec::new();
    };
    let matrix = DMat3::from_quat(pose.rotation);
    let z = matrix.z_axis;
    let norm = z.length();
    let radius = z.x.hypot(z.y);
    let azimuth = z.y.atan2(z.x);
    let mut result = Vec::new();
    for angle5 in range5 {
        let (s5, c5) = angle5.sin_cos();
        let mut transverse = vec![-norm * s5, norm * s5];
        if constraints.tolerances[3] < PI
            && let Some(range4) = limits(robot, 3, reference)
        {
            transverse.extend(range4.map(|q4| norm * s5 * q4.sin()));
        }
        for component in transverse {
            if radius > 0.0 && component.abs() <= radius * (1.0 + 64.0 * f64::EPSILON) {
                let offset = (component / radius).clamp(-1.0, 1.0).asin();
                result.extend([
                    wrapped_angle(azimuth - offset),
                    wrapped_angle(azimuth - PI + offset),
                ]);
            }
        }
        if fixed_j6.is_none()
            && constraints.tolerances[5] < PI
            && let Some(range6) = limits(robot, 5, reference)
        {
            for angle6 in range6 {
                let (s6, c6) = angle6.sin_cos();
                let arm_axis = matrix.x_axis * (-s5 * c6) + matrix.y_axis * (s5 * s6) + z * c5;
                let angle = arm_axis.y.atan2(arm_axis.x);
                result.extend([angle, wrapped_angle(angle + PI)]);
            }
        }
        if let Some(range2) = limits(robot, 1, reference) {
            for angle2 in range2 {
                let (s23, c23) = (angle2 + q3).sin_cos();
                result.extend(squared_norm_roots(
                    [-z.z * s23, z.x * c23, z.y * c23],
                    [0.0, z.y, -z.x],
                    norm * s5.abs(),
                ));
            }
        }
    }
    result
}

/// Finds J1 cuts at interval endpoints, curve intersections, and vertical tangencies.
fn critical_slices(curves: &[Curve], lower: f64, upper: f64) -> Vec<f64> {
    let origin = lower + (upper - lower) / 2.0;
    let min_t = ((lower - origin) / 2.0).tan();
    let max_t = ((upper - origin) / 2.0).tan();
    let polynomials: Vec<_> = curves
        .iter()
        .map(|curve| curve.polynomials(origin))
        .collect();
    let mut cuts = vec![lower, upper];
    let mut append = |polynomial| {
        cuts.extend(
            roots(polynomial, min_t, max_t)
                .into_iter()
                .map(|root| origin + 2.0 * root.atan()),
        );
    };
    for (index, [a, b, c]) in polynomials.iter().enumerate() {
        // A repeated t root is precisely a vertical tangent (including a curve
        // that vanishes for all t at an isolated q1).
        append(add(
            &add(&multiply(a, a), &multiply(b, b), 1.0),
            &multiply(c, c),
            -1.0,
        ));
        for [d, e, f] in polynomials.iter().skip(index + 1) {
            let denominator = determinant(a, e, d, b);
            let cosine_numerator = determinant(b, f, e, c);
            let sine_numerator = determinant(c, d, f, a);
            append(add(
                &add(
                    &multiply(&cosine_numerator, &cosine_numerator),
                    &multiply(&sine_numerator, &sine_numerator),
                    1.0,
                ),
                &multiply(&denominator, &denominator),
                -1.0,
            ));
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup();
    cuts
}

/// Selects the best feasible arm branch per J1 slice by searching its free J2 angle.
pub(crate) fn search(
    robot: &OPWKinematics,
    pose: &Pose,
    q3: f64,
    reference: &Joints,
    fixed_j6: Option<f64>,
) -> Vec<ArmBranch> {
    let Some([lower, upper]) = limits(robot, 0, reference) else {
        return Vec::new();
    };
    let model_reference = wrapped_angle(
        reference[0] * robot.parameters.sign_corrections[0] as f64 - robot.parameters.offsets[0],
    );
    let mut slices = vec![model_reference, lower, upper];
    let curves = boundaries(robot, pose, q3, reference, fixed_j6);
    let bend_slices = bend_slices(robot, pose, q3, reference, fixed_j6);
    // Two overlapping-endpoint charts keep tan(q1/2) in [-1,1], avoiding its
    // pole and poorly conditioned coefficients at very large chart coordinates.
    let middle = lower + (upper - lower) / 2.0;
    for [lo, hi] in [[lower, middle], [middle, upper]] {
        let mut cuts = critical_slices(&curves, lo, hi);
        for &angle in &bend_slices {
            for winding in [-1.0, 0.0, 1.0] {
                let angle = angle + winding * 2.0 * PI;
                if lo <= angle && angle <= hi {
                    cuts.push(angle);
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.dedup();
        slices.extend(cuts.iter().copied());
        slices.extend(
            cuts.windows(2)
                .map(|pair| pair[0] + (pair[1] - pair[0]) / 2.0),
        );
    }
    slices.sort_by(f64::total_cmp);
    slices.dedup();
    let mut result = Vec::new();
    for q1 in slices {
        let arm = ArmBranch { q1, q2: 0.0, q3 };
        let mut candidates = Vec::new();
        for branch in j2free::search(robot, pose, arm, reference, fixed_j6) {
            candidates.extend(robot.arm_wrist_candidates(pose, branch, reference, fixed_j6));
        }
        robot.sort_by_closeness(&mut candidates, reference);
        if let Some(joints) = candidates.first() {
            // One representative per slice bounds the candidate count for the
            // downstream duplicate filter, while retaining its best ranking.
            result.push(ArmBranch {
                q1: joints[0] * robot.parameters.sign_corrections[0] as f64
                    - robot.parameters.offsets[0],
                q2: joints[1] * robot.parameters.sign_corrections[1] as f64
                    - robot.parameters.offsets[1],
                q3,
            });
        }
    }
    result
}

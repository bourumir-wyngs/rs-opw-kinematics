use crate::annotations::AnnotatedJoints;

/// Structure representing the interpolator with the original trajectory steps.
/// Uses cubic Bezier curves.
pub (crate) struct Interpolator {
    steps: Vec<AnnotatedJoints>, // The key points along the trajectory
    tangents: Vec<[f64; 6]>,     // Precomputed tangents for smoother Bezier curves
}

impl Interpolator {
    /// Create a new interpolator with given trajectory steps
    pub fn new(steps: Vec<AnnotatedJoints>) -> Self {
        assert!(steps.len() >= 2, "At least two steps are required for interpolation.");

        // Compute tangents for all points
        let tangents = Self::compute_tangents(&steps);

        Interpolator { steps, tangents }
    }

    /// Precompute tangents using finite differences for smooth Bezier transitions
    fn compute_tangents(steps: &Vec<AnnotatedJoints>) -> Vec<[f64; 6]> {
        let mut tangents = vec![[0.0; 6]; steps.len()];

        for i in 0..steps.len() {
            if i == 0 {
                // Forward difference for the first point
                for j in 0..6 {
                    tangents[i][j] = steps[i + 1].joints[j] - steps[i].joints[j];
                }
            } else if i == steps.len() - 1 {
                // Backward difference for the last point
                for j in 0..6 {
                    tangents[i][j] = steps[i].joints[j] - steps[i - 1].joints[j];
                }
            } else {
                // Central difference for inner points
                for j in 0..6 {
                    tangents[i][j] = 0.5 * (steps[i + 1].joints[j] - steps[i - 1].joints[j]);
                }
            }
        }

        tangents
    }

    /// Compute a control point based on the tangent direction
    fn compute_control_point(
        &self,
        point: &AnnotatedJoints,
        index: usize,
        scalar: f64,
    ) -> AnnotatedJoints {
        let mut control_joints = [0.0; 6];
        for i in 0..6 {
            control_joints[i] = point.joints[i] + scalar * self.tangents[index][i];
        }

        AnnotatedJoints {
            joints: control_joints,
            flags: point.flags, // Inherit flags (can be handled differently if needed)
        }
    }

    /// Map `t` (0.0 to 1.0) onto the correct segment and compute cubic Bezier interpolation
    pub fn interpolate(&self, t: f64) -> AnnotatedJoints {
        // Edge cases with clamping
        if t >= 1.0 {
            return self.steps[self.steps.len() - 1].clone();
        }
        if t <= 0.0 {
            return self.steps[0].clone();
        }

        let num_segments = self.steps.len() - 1;
        let segment_length = 1.0 / num_segments as f64;

        let segment_index = (t / segment_length).floor() as usize;
        let clamped_index = segment_index.min(num_segments - 1);

        let t_local = (t - clamped_index as f64 * segment_length) / segment_length;

        let p0 = &self.steps[clamped_index];
        let p3 = &self.steps[clamped_index + 1];

        let p1 = self.compute_control_point(p0, clamped_index, 1.0 / 3.0);
        let p2 = self.compute_control_point(p3, clamped_index + 1, -1.0 / 3.0);

        self.local_bezier_interpolation(p0, &p1, &p2, p3, t_local)
    }

    /// Local Bezier interpolation between `p0`, `p1`, `p2`, and `p3`
    fn local_bezier_interpolation(
        &self,
        p0: &AnnotatedJoints,
        p1: &AnnotatedJoints,
        p2: &AnnotatedJoints,
        p3: &AnnotatedJoints,
        t: f64,
    ) -> AnnotatedJoints {
        let t1 = 1.0 - t;

        let mut interpolated = [0.0; 6];
        for i in 0..6 {
            interpolated[i] = t1.powi(3) * p0.joints[i]
                + 3.0 * t1.powi(2) * t * p1.joints[i]
                + 3.0 * t1 * t.powi(2) * p2.joints[i]
                + t.powi(3) * p3.joints[i];
        }

        AnnotatedJoints {
            joints: interpolated,
            flags: if t < 0.5 { p0.flags } else { p3.flags },
        }
    }
    
}
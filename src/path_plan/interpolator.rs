use nalgebra::{Vector3, Isometry3};
use crate::annotations::AnnotatedJoints;
use crate::kinematic_traits::Joints;

/// Structure representing the interpolator with the original trajectory steps
pub struct Interpolator {
    steps: Vec<AnnotatedJoints>, // The key points along the trajectory
}

pub fn interpolate_joints(start: &Joints, end: &Joints, t: f64) -> [f64; 6] {
    if t < 0.0 {
        return start.clone();
    } else if t > 1.0 {
        return end.clone();
    }

    let mut interpolated = [0.0; 6];
    for i in 0..6 {
        interpolated[i] = start[i] + t * (end[i] - start[i]);
    }
    interpolated
}


impl Interpolator {
    /// Create a new interpolator with given trajectory steps
    pub fn new(steps: Vec<AnnotatedJoints>) -> Self {
        assert!(steps.len() >= 2, "At least two steps are required for interpolation.");
        Interpolator { steps }
    }

    /// Map `t` (0.0 to 1.0) onto the correct segment and compute the interpolation
    pub fn interpolate(&self, t: f64) -> AnnotatedJoints {
        assert!(t >= 0.0 && t <= 1.0, "t must be in the range [0.0, 1.0].");

        let num_segments = self.steps.len() - 1; // One less than the number of points
        let segment_length = 1.0 / num_segments as f64; // Normalize segments to [0,1]

        // Determine which segment `t` falls into
        let segment_index = (t / segment_length).floor() as usize;
        let clamped_index = segment_index.min(num_segments - 1); // Clamp index to valid range

        // Get the local `t` within the segment [0, 1] range
        let t_local = (t - clamped_index as f64 * segment_length) / segment_length;

        // Extract the two neighboring points for this segment
        let start = &self.steps[clamped_index];
        let end = &self.steps[clamped_index + 1];

        // Perform local interpolation
        self.local_interpolation(start, end, t_local)
    }

    /// Local interpolation between two `AnnotatedJoints` using a cubic Hermite spline
    fn local_interpolation(
        &self,
        start: &AnnotatedJoints,
        end: &AnnotatedJoints,
        t: f64,
    ) -> AnnotatedJoints {
        // Linear interpolation (can be replaced with cubic Hermite or spline later)
        let interpolated_joints = interpolate_joints(&start.joints, &end.joints, t);

        let interpolated_flags = if t < 0.5 {
            start.flags
        } else {
            end.flags
        };

        AnnotatedJoints {
            joints: interpolated_joints,
            flags: interpolated_flags,
        }
    }
}
use crate::annotations::AnnotatedPose;
use nalgebra::{Isometry3, UnitQuaternion, Vector3};

pub struct Smoother {
    /// The smoothing weight of the middle point.
    weight: f64,

    /// The number of points to smooth over (valid values are 1 (off), 3, 5 and 7
    points: usize,
}

impl Smoother {
    pub fn new(weight: f64, points: usize) -> Self {
        if points != 1 && points != 3 && points != 5 && points != 7 {
            panic!(
                "Invalid number of points for smoothing, allowd 1(off) or 3,5,7: {}",
                points
            );
        }
        Self { weight, points }
    }

    pub fn smooth_orientations(&self, path: &Vec<AnnotatedPose>) -> Vec<AnnotatedPose> {
        match self.points {
            1 => path.clone(),                    // No smoothing (return original path)
            3 => self.smooth_orientation_3(path), // Use 3-point smoothing
            5 => self.smooth_orientation_5(path), // Use 5-point smoothing
            7 => self.smooth_orientation_7(path), // Use 7-point smoothing
            _ => panic!("Unsupported smoothing option: {}", self.points), // Panic for invalid values
        }
    }

    // Ensure consistency for an arbitrary number of quaternions
    pub fn ensure_consistent_arbitrary(
        quaternions: &[UnitQuaternion<f64>],
    ) -> Vec<UnitQuaternion<f64>> {
        if quaternions.is_empty() {
            return vec![]; // No quaternions to process
        }

        let mut consistent_quaternions = quaternions.to_vec();

        for i in 1..consistent_quaternions.len() {
            // If the dot product is negative, invert the quaternion
            if consistent_quaternions[i - 1]
                .coords
                .dot(&consistent_quaternions[i].coords)
                < 0.0
            {
                consistent_quaternions[i] =
                    UnitQuaternion::from_quaternion(-consistent_quaternions[i].into_inner());
            }
        }

        consistent_quaternions
    }

    // Combines the smoothed quaternions with the original poses (translations and flags)
    fn zip_with_smoothed_quaternions(
        path: &Vec<AnnotatedPose>,
        quaternions: Vec<UnitQuaternion<f64>>,
    ) -> Vec<AnnotatedPose> {
        path.iter()
            .zip(quaternions)
            .map(|(pose, quaternion)| AnnotatedPose {
                pose: Isometry3::from_parts(pose.pose.translation, quaternion),
                flags: pose.flags.clone(),
            })
            .collect()
    }

    // Smooth a path using 3-point smoothing logic
    pub fn smooth_orientation_3(&self, path: &Vec<AnnotatedPose>) -> Vec<AnnotatedPose> {
        if path.len() < 3 {
            return path.clone(); // Nothing to smooth if fewer than 3 poses
        }

        // Step 1: Extract the quaternions
        let quaternions: Vec<UnitQuaternion<f64>> =
            path.iter().map(|pose| pose.pose.rotation).collect();

        // Step 2: Ensure consistency of the extracted quaternions
        let consistent_quaternions = Self::ensure_consistent_arbitrary(&quaternions);

        // Step 3: Apply 3-point smoothing on quaternions
        let mut smoothed_quaternions = Vec::with_capacity(path.len());
        for window in consistent_quaternions.windows(3) {
            let (q1, q2, q3) = (window[0], window[1], window[2]);

            // Blend the quaternions using spherical linear interpolation (slerp)
            let mid_slerp = q1.slerp(&q3, 0.5);
            let refined_mid = mid_slerp.slerp(&q2, 0.1); // Slight bias toward q2
            smoothed_quaternions.push(refined_mid);
        }

        // Handle the first and last points (no full window for these)
        smoothed_quaternions.insert(
            0,
            consistent_quaternions[0].slerp(&consistent_quaternions[1], 0.5),
        );
        smoothed_quaternions.push(
            consistent_quaternions[consistent_quaternions.len() - 2].slerp(
                &consistent_quaternions[consistent_quaternions.len() - 1],
                0.5,
            ),
        );

        // Step 4: Combine the smoothed quaternions back with the full metadata
        Self::zip_with_smoothed_quaternions(path, smoothed_quaternions)
    }

    // Smooth a path using 5-point smoothing logic
    pub fn smooth_orientation_5(&self, path: &Vec<AnnotatedPose>) -> Vec<AnnotatedPose> {
        if path.len() < 5 {
            return self.smooth_orientation_3(path); // Fall back to 3-point smoothing logic
        }

        // Step 1: Extract the quaternions
        let quaternions: Vec<UnitQuaternion<f64>> =
            path.iter().map(|pose| pose.pose.rotation).collect();

        // Step 2: Ensure consistency of the quaternions
        let consistent_quaternions = Self::ensure_consistent_arbitrary(&quaternions);

        // Step 3: Apply 5-point smoothing on quaternions
        let mut smoothed_quaternions = Vec::with_capacity(path.len());
        for window in consistent_quaternions.windows(5) {
            let (q1, q2, q3, q4, q5) = (window[0], window[1], window[2], window[3], window[4]);

            // Blend the quaternions for smooth transitions
            let mid_slerp = q1.slerp(&q5, 0.5);
            let refined_mid = mid_slerp.slerp(&q3, 0.25); // Slight bias toward the center
            smoothed_quaternions.push(refined_mid);
        }

        // Handle the first two and last two points (no full window for these)
        smoothed_quaternions.insert(
            0,
            consistent_quaternions[0].slerp(&consistent_quaternions[1], 0.5),
        );
        smoothed_quaternions.insert(
            1,
            consistent_quaternions[1].slerp(&consistent_quaternions[2], 0.5),
        );
        smoothed_quaternions.push(
            consistent_quaternions[consistent_quaternions.len() - 3].slerp(
                &consistent_quaternions[consistent_quaternions.len() - 2],
                0.5,
            ),
        );
        smoothed_quaternions.push(
            consistent_quaternions[consistent_quaternions.len() - 2].slerp(
                &consistent_quaternions[consistent_quaternions.len() - 1],
                0.5,
            ),
        );

        // Step 4: Combine the smoothed quaternions back with the full metadata
        Self::zip_with_smoothed_quaternions(path, smoothed_quaternions)
    }

    // Smooth a path using 7-point smoothing logic
    pub fn smooth_orientation_7(&self, path: &Vec<AnnotatedPose>) -> Vec<AnnotatedPose> {
        if path.len() < 7 {
            return self.smooth_orientation_5(path); // Fall back to 5-point smoothing logic
        }

        // Step 1: Extract the quaternions
        let quaternions: Vec<UnitQuaternion<f64>> =
            path.iter().map(|pose| pose.pose.rotation).collect();

        // Step 2: Ensure consistency of the quaternions
        let consistent_quaternions = Self::ensure_consistent_arbitrary(&quaternions);

        // Step 3: Apply 7-point smoothing on quaternions
        let mut smoothed_quaternions = Vec::with_capacity(path.len());
        for window in consistent_quaternions.windows(7) {
            let (q1, q2, q3, q4, q5, q6, q7) = (
                window[0], window[1], window[2], window[3], window[4], window[5], window[6],
            );

            // Blend the quaternions for smooth transitions
            let mid_slerp = q1.slerp(&q7, 0.5); // Blend q1 and q7 evenly
            let refined_mid = mid_slerp
                .slerp(&q4, 0.25) // Slight bias toward the center quaternion
                .slerp(&q3.slerp(&q5, 0.5), 0.25); // Extra refinement using q3 and q5
            smoothed_quaternions.push(refined_mid);
        }

        // Handle the first three and last three points (no full window for these)
        smoothed_quaternions.insert(
            0,
            consistent_quaternions[0].slerp(&consistent_quaternions[1], 0.5),
        );
        smoothed_quaternions.insert(
            1,
            consistent_quaternions[1].slerp(&consistent_quaternions[2], 0.5),
        );
        smoothed_quaternions.insert(
            2,
            consistent_quaternions[2].slerp(&consistent_quaternions[3], 0.5),
        );
        smoothed_quaternions.push(
            consistent_quaternions[consistent_quaternions.len() - 4].slerp(
                &consistent_quaternions[consistent_quaternions.len() - 3],
                0.5,
            ),
        );
        smoothed_quaternions.push(
            consistent_quaternions[consistent_quaternions.len() - 3].slerp(
                &consistent_quaternions[consistent_quaternions.len() - 2],
                0.5,
            ),
        );
        smoothed_quaternions.push(
            consistent_quaternions[consistent_quaternions.len() - 2].slerp(
                &consistent_quaternions[consistent_quaternions.len() - 1],
                0.5,
            ),
        );

        // Step 4: Combine the smoothed quaternions back with the full metadata
        Self::zip_with_smoothed_quaternions(path, smoothed_quaternions)
    }
}

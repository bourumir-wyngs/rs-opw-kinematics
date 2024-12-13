use nalgebra::{Isometry3, Point3, Vector3};

fn transform_points(points: &[(f32, f32)], pose: &Isometry3<f32>) -> Vec<Point3<f32>> {
    let rotation = pose.rotation; // Extract the rotation part of the isometry
    let translation = pose.translation.vector; // Extract the translation part of the isometry

    points
        .iter()
        .map(|&(x, y)| {
            // Convert the 2D point into 3D by embedding it in the XY plane
            let point_3d = Vector3::new(x, y, 0.0);
            // Apply rotation to align the point to the new plane
            let rotated_point = rotation * point_3d;
            // Apply translation to match the isometry's translation
            Point3::from(rotated_point + translation)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transform_points() {
        // Define 2D points
        let points = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];

        // Define a pose (translation and rotation)
        let translation = Vector3::new(10.0, 20.0, 30.0);
        let rotation = Vector3::z() * std::f32::consts::FRAC_PI_4; // 45 degrees about Z-axis
        let pose = Isometry3::new(translation, rotation);

        // Apply the transformation
        let transformed_points = transform_points(&points, &pose);

        // Define expected results (manually calculated or verified)
        let expected_points = vec![
            Point3::new(10.7071, 20.7071, 30.0), // (1.0, 0.0)
            Point3::new(9.2929, 20.7071, 30.0),  // (0.0, 1.0)
            Point3::new(9.2929, 19.2929, 30.0),  // (-1.0, 0.0)
            Point3::new(10.7071, 19.2929, 30.0), // (0.0, -1.0)
        ];

        // Verify that each point is close to the expected point (with a tolerance)
        for (result, expected) in transformed_points.iter().zip(expected_points.iter()) {
            assert!((result.x - expected.x).abs() < 1e-4);
            assert!((result.y - expected.y).abs() < 1e-4);
            assert!((result.z - expected.z).abs() < 1e-4);
        }
    }
}

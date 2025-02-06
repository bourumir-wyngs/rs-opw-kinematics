use nalgebra::{Isometry3, Matrix4, Point3, Rotation3, Transform3, Translation3, UnitQuaternion, Vector3};
use parry3d::math::Point as ParryPoint;

/// Compute the coordinates of the tetrahedron's points: Apex and Base (Red, Green, Blue).
///
/// The tetrahedron stands on its **apex** (0, 0, 0),
/// and the base is an equilateral triangle at a positive height along the Z-axis.
///
/// `bond_length` is the distance between any two connected vertices.
///
/// Returns:
/// - `red`, `green`, `blue`: Three points forming the base in the plane parallel to XY but at height Z.
pub fn compute_tetrahedron_geometry(
    bond_length: f32,
) -> (
    ParryPoint<f32>,
    ParryPoint<f32>,
    ParryPoint<f32>,
) {
    // Compute the height of the base plane from the origin (apex)
    // H_base = bond_length * sqrt(2/3)
    let base_height = bond_length * (2.0_f32 / 3.0).sqrt();

    // Compute the radius of the base's circumcircle in the XY-plane
    // Base radius = bond_length * sqrt(1/3)
    let base_radius = bond_length * (1.0_f32 / 3.0).sqrt();

    // Green point (directly along the +Y direction at height Z = base_height)
    let green = ParryPoint::new(0.0, base_radius, base_height);

    // Red point (in the XY-plane rotated symmetrically)
    let red = ParryPoint::new(
        -base_radius * (3.0_f32.sqrt() / 2.0), // X-coordinate
        -base_radius / 2.0,                    // Y-coordinate
        base_height,                           // Z-coordinate
    );

    // Blue point (mirror of red in the XY-plane)
    let blue = ParryPoint::new(
        base_radius * (3.0_f32.sqrt() / 2.0), // X-coordinate
        -base_radius / 2.0,                   // Y-coordinate
        base_height,                          // Z-coordinate
    );

    (red, green, blue)
}

/// Computes the transformation (scaling, rotation, translation)
/// between a reference equilateral triangle and an observed triangle.
///
/// The function incorporates edge-length redundancy to make the computation more robust.
///
/// Returns:
/// transform that includes translation and rotation, and may also include scaling.
pub fn find_transform(
    red_ref: ParryPoint<f32>,   // Reference Red point
    green_ref: ParryPoint<f32>, // Reference Green point
    blue_ref: ParryPoint<f32>,  // Reference Blue point
    red_obs: ParryPoint<f32>,   // Observed Red point
    green_obs: ParryPoint<f32>, // Observed Green point
    blue_obs: ParryPoint<f32>,  // Observed Blue point
) -> Isometry3<f32> {
    let translation =
        compute_transform_components(red_obs, green_obs, blue_obs, red_ref, green_ref, blue_ref);
    Isometry3::from_parts(translation, UnitQuaternion::identity())
    //create_transform(scaling, rotation, translation)
}

fn create_transform(
    scaling: f32,
    rotation: Rotation3<f32>,
    translation: Translation3<f32>,
) -> Transform3<f32> {
    let scaling_matrix = Matrix4::new_nonuniform_scaling(&Vector3::new(scaling, scaling, scaling));
    Transform3::from_matrix_unchecked(
        translation.to_homogeneous() * rotation.to_homogeneous() * scaling_matrix,
    )
}


fn compute_transform_components(
    red_ref: ParryPoint<f32>,   // Reference Red point
    green_ref: ParryPoint<f32>, // Reference Green point
    blue_ref: ParryPoint<f32>,  // Reference Blue point
    red_obs: ParryPoint<f32>,   // Observed Red point
    green_obs: ParryPoint<f32>, // Observed Green point
    blue_obs: ParryPoint<f32>,  // Observed Blue point
) -> Translation3<f32> {
    // STEP 1: Compute centroids
    let centroid_ref = Point3::new(
        (red_ref.x + green_ref.x + blue_ref.x) / 3.0,
        (red_ref.y + green_ref.y + blue_ref.y) / 3.0,
        (red_ref.z + green_ref.z + blue_ref.z) / 3.0,
    );

    let centroid_obs = Point3::new(
        (red_obs.x + green_obs.x + blue_obs.x) / 3.0,
        (red_obs.y + green_obs.y + blue_obs.y) / 3.0,
        (red_obs.z + green_obs.z + blue_obs.z) / 3.0,
    );

    // STEP 2: Compute translation aligning centroids
    let translation_vector = Vector3::new(
        centroid_obs.x - centroid_ref.x,
        centroid_obs.y - centroid_ref.y,
        centroid_obs.z - centroid_ref.z,
    );
    let translation = Translation3::from(translation_vector);

    translation
}

/// Computes the transformation (scaling, rotation, translation)
/// between a reference equilateral triangle and an observed triangle.
///
/// The function incorporates edge-length redundancy to make the computation more robust.
///
/// Returns:
/// - `scaling` (f32): Scaling factor between the reference and observed triangles.
/// - `translation` (Translation3<f32>): Translation vector aligning triangle centroids.
/// - `rotation` (Rotation3<f32>): Rotation matrix aligning the observed triangle to the reference.
fn compute_transform_components_full(
    red_ref: ParryPoint<f32>,   // Reference Red point
    green_ref: ParryPoint<f32>, // Reference Green point
    blue_ref: ParryPoint<f32>,  // Reference Blue point
    red_obs: ParryPoint<f32>,   // Observed Red point
    green_obs: ParryPoint<f32>, // Observed Green point
    blue_obs: ParryPoint<f32>,  // Observed Blue point
) -> (f32, Translation3<f32>, Rotation3<f32>) {
    // STEP 1: Compute centroids
    let centroid_ref = Point3::new(
        (red_ref.x + green_ref.x + blue_ref.x) / 3.0,
        (red_ref.y + green_ref.y + blue_ref.y) / 3.0,
        (red_ref.z + green_ref.z + blue_ref.z) / 3.0,
    );

    let centroid_obs = Point3::new(
        (red_obs.x + green_obs.x + blue_obs.x) / 3.0,
        (red_obs.y + green_obs.y + blue_obs.y) / 3.0,
        (red_obs.z + green_obs.z + blue_obs.z) / 3.0,
    );

    // STEP 2: Compute translation aligning centroids
    let translation_vector = Vector3::new(
        centroid_obs.x - centroid_ref.x,
        centroid_obs.y - centroid_ref.y,
        centroid_obs.z - centroid_ref.z,
    );
    let translation = Translation3::from(translation_vector);

    // STEP 3: Normalize all points (translate so the centroid is at the origin)
    let red_ref_norm = Vector3::new(
        red_ref.x - centroid_ref.x,
        red_ref.y - centroid_ref.y,
        red_ref.z - centroid_ref.z,
    );
    let green_ref_norm = Vector3::new(
        green_ref.x - centroid_ref.x,
        green_ref.y - centroid_ref.y,
        green_ref.z - centroid_ref.z,
    );
    let blue_ref_norm = Vector3::new(
        blue_ref.x - centroid_ref.x,
        blue_ref.y - centroid_ref.y,
        blue_ref.z - centroid_ref.z,
    );

    let red_obs_norm = Vector3::new(
        red_obs.x - centroid_obs.x,
        red_obs.y - centroid_obs.y,
        red_obs.z - centroid_obs.z,
    );
    let green_obs_norm = Vector3::new(
        green_obs.x - centroid_obs.x,
        green_obs.y - centroid_obs.y,
        green_obs.z - centroid_obs.z,
    );
    let blue_obs_norm = Vector3::new(
        blue_obs.x - centroid_obs.x,
        blue_obs.y - centroid_obs.y,
        blue_obs.z - centroid_obs.z,
    );

    // STEP 4: Compute scaling using redundancy (average edge length ratio)
    let ref_edge_lengths = [
        (red_ref_norm - green_ref_norm).norm(),
        (green_ref_norm - blue_ref_norm).norm(),
        (blue_ref_norm - red_ref_norm).norm(),
    ];

    let obs_edge_lengths = [
        (red_obs_norm - green_obs_norm).norm(),
        (green_obs_norm - blue_obs_norm).norm(),
        (blue_obs_norm - red_obs_norm).norm(),
    ];

    let avg_ref_edge = ref_edge_lengths.iter().copied().sum::<f32>() / 3.0;
    let avg_obs_edge = obs_edge_lengths.iter().copied().sum::<f32>() / 3.0;

    let scaling = avg_obs_edge / avg_ref_edge;

    // STEP 5: Compute rotation using centroid-aligned points (Procrustes with redundancy)
    use nalgebra::Matrix3;

    // Create matrix representations from decomposed points
    let ref_matrix = Matrix3::from_columns(&[red_ref_norm, green_ref_norm, blue_ref_norm]);
    let obs_matrix = Matrix3::from_columns(&[red_obs_norm, green_obs_norm, blue_obs_norm]);

    // Rotation: SVD-based approach
    let svd = (obs_matrix * ref_matrix.transpose()).svd(true, true);
    let u = svd.u.unwrap(); // Left singular vectors
    let v_t = svd.v_t.unwrap(); // Right singular vectors
    let rotation_matrix = u * v_t;

    // Ensure it's a proper orthogonal rotation (determinant = 1, no reflection)
    let det = rotation_matrix.determinant();
    println!("Determinant of the rotation matrix: {}", det);
    let rotation_matrix = if (u * v_t).determinant() < 0.0 {
        let mut u_fixed = u.clone();
        u_fixed.column_mut(2).neg_mut(); // Flip the sign of the last column
        u_fixed * v_t
    } else {
        u * v_t
    };
    let rotation = Rotation3::from_matrix_unchecked(rotation_matrix);

    (scaling, translation, rotation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;

    #[test]
    fn test_compute_transform_with_rotation_all_axes() {
        // Define reference points of an equilateral triangle
        let sqrt_3: f32 = 3.0_f32.sqrt();
        let red_ref = Point3::new(-0.5, -sqrt_3 / 6.0, 0.0);
        let green_ref = Point3::new(0.5, -sqrt_3 / 6.0, 0.0);
        let blue_ref = Point3::new(0.0, sqrt_3 / 3.0, 0.0);

        // Define observed points (scaled, translated, rotated)
        let scaling = 2.0;
        let translation = Vector3::new(1.0, 2.0, 3.0);

        // Define rotations around all three axes
        let angle_x = std::f32::consts::FRAC_PI_6; // 30 degrees around X-axis
        let angle_y = std::f32::consts::FRAC_PI_4; // 45 degrees around Y-axis
        let angle_z = std::f32::consts::FRAC_PI_3; // 60 degrees around Z-axis

        // Create individual rotations
        let rotation_x = Rotation3::from_axis_angle(&Vector3::x_axis(), angle_x);
        let rotation_y = Rotation3::from_axis_angle(&Vector3::y_axis(), angle_y);
        let rotation_z = Rotation3::from_axis_angle(&Vector3::z_axis(), angle_z);

        // Combine the rotations
        let rotation = rotation_z * rotation_y * rotation_x;

        // Calculate observed points based on the transformation
        let red_obs = rotation * (red_ref * scaling) + translation;
        let green_obs = rotation * (green_ref * scaling) + translation;
        let blue_obs = rotation * (blue_ref * scaling) + translation;

        // Call the function to compute the transformation
        let (computed_scaling, computed_translation, computed_rotation) =
            compute_transform_components(
                red_ref, green_ref, blue_ref, red_obs, green_obs, blue_obs,
            );

        // Verify scaling
        assert!((computed_scaling - scaling).abs() < 1e-6);

        // Verify translation
        assert!((computed_translation.vector - translation).norm() < 1e-6);

        // Verify rotation
        let rotation_diff = computed_rotation * rotation.inverse();
        assert!(rotation_diff.angle() < 1e-6); // The angular difference should be negligible
    }

    fn distance(p1: &ParryPoint<f32>, p2: &ParryPoint<f32>) -> f32 {
        ((p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2) + (p2.z - p1.z).powi(2)).sqrt()
    }

    #[test]
    fn test_tetrahedron_geometry() {
        let bond_length = 1.0; // Define bond length
        let (red, green, blue) = compute_tetrahedron_geometry(bond_length);

        // Verify Base Height (Z value of red, green, and blue must match computed base_height)
        let base_height = bond_length * (2.0_f32 / 3.0).sqrt();
        assert!((red.z - base_height).abs() < 1e-6);
        assert!((green.z - base_height).abs() < 1e-6);
        assert!((blue.z - base_height).abs() < 1e-6);

        // Verify Distances in Base (Equilateral Triangle)
        assert!((distance(&red, &green) - bond_length).abs() < 1e-6);
        assert!((distance(&green, &blue) - bond_length).abs() < 1e-6);
        assert!((distance(&blue, &red) - bond_length).abs() < 1e-6);
    }

    #[test]
    fn test_create_transform() {
        use nalgebra::{Point3, Rotation3, Translation3, Vector3};

        // Define scaling, rotation, and translation
        let scaling = 2.0;
        let translation = Translation3::new(1.0, 2.0, 3.0); // Translation vector
        let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), std::f32::consts::FRAC_PI_2); // 90 degrees around Z-axis

        // Create the transform
        let transform = create_transform(scaling, rotation, translation);

        // Define a point to be transformed
        let point = Point3::new(1.0, 3.0, 9.0);

        // Expected transformation: scaling, rotation, and translation applied manually
        let scaled_point = Point3::new(point.x * scaling, point.y * scaling, point.z * scaling); // Scale the point
        let rotated_point = rotation.transform_point(&scaled_point); // Apply rotation
        let expected_point = rotated_point + translation.vector; // Apply translation

        // Use the transform to compute the transformed point
        let transformed_point = transform.transform_point(&point);

        // Validate the transformed point
        assert!(
            relative_eq!(transformed_point, expected_point, epsilon = 1e-6),
            "The transformed point {:?} does not match expected point {:?}",
            transformed_point,
            expected_point
        );

        // Check if the inverse transform brings us back to the original point
        if let Some(inverse_transform) = transform.try_inverse() {
            let original_point = inverse_transform.transform_point(&transformed_point);
            assert!(
                relative_eq!(original_point, point, epsilon = 1e-6),
                "The inverse transform did not return the original point. Expected {:?}, got {:?}",
                point,
                original_point
            );

            println!("Inverse transform successfully returns the original point!");
        } else {
            panic!("Failed to compute the inverse of the transform");
        }

        println!("Transformed point matches expected point!");
    }
}

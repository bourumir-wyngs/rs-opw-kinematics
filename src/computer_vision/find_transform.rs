use nalgebra::{
    Isometry3, Matrix4, Point3, Rotation3, Transform3, Translation3, UnitQuaternion, Vector3,
};
use num_traits::real::Real;
use parry3d::math::Point as ParryPoint;
use parry3d::query::distance;

/// Tetrahedron is standing on the bottom vertex. The top 3 vertices form a plane parallel to XY
/// (while Z = const = 0). They make equilateral triangle. Green vertex belongs to Y
/// axis in a positive direction. "Red" vertex has the larger X coordinate than "Green",
/// "Blue" has the smaller X coordinate than "Green".
pub fn compute_tetrahedron_geometry(
    bond_length: f32,
) -> (ParryPoint<f32>, ParryPoint<f32>, ParryPoint<f32>, ParryPoint<f32>) {
    // Base lies on the XY plane (yellow bubble goes down into negative coordinates)
    let base_height = 0.0; 

    // Distance from centroid to any base vertex in XY
    let base_radius_xy = bond_length * (1.0_f32 / 2.0).sqrt(); 

    // Define base (top) vertices
    let green = ParryPoint::new(
        0.0,
        base_radius_xy, // First vertex along the X-axis
        base_height,
    );

    let rb_y_offset = -base_radius_xy / 2.0;
    let rb_x_offset = (0.75_f32).sqrt() * base_radius_xy;
    let blue = ParryPoint::new(-rb_x_offset, rb_y_offset, base_height);
    let red = ParryPoint::new(rb_x_offset, rb_y_offset, base_height);

    // Compute centroid of red, green, blue
    let centroid_x = (red.x + green.x + blue.x) / 3.0;
    let centroid_y = (red.y + green.y + blue.y) / 3.0;
    
    let dx = red.x - green.x;
    let dy = red.y - green.y;
    let edge = (dx * dx + dy * dy).sqrt();

    // Height from centroid of top triangle to bottom vertex (yellow vertex)
    let yellow_z =  -(edge * 6.0_f32.sqrt()) / 3.0;
    // Compute yellow vertex
    let yellow = ParryPoint::new(centroid_x, centroid_y, yellow_z);

    (red, green, blue, yellow)
}

pub fn base_height(bond_length: f32) -> f32 {
    // The angle between any two bonds, known from the theory about Methane
    let alpha = (-1.0 / 3.0).acos(); // or 109.5

    // The angle between XY plane and any top bond.
    let beta = alpha - 90_f32.to_radians();

    // The height of the plane where the top 3 H atoms belong
    let base_height = bond_length + bond_length * beta.sin();
    base_height
}

/// Computes the transformation (scaling, rotation, translation)
/// between a reference equilateral triangle and an observed triangle.
///
/// The function incorporates edge-length redundancy to make the computation more robust.
///
/// Returns:
/// transform that includes translation and rotation, and may also include scaling.
pub fn find_transform(
    red_ref: ParryPoint<f32>,   
    green_ref: ParryPoint<f32>, 
    blue_ref: ParryPoint<f32>,  
    yellow_ref: ParryPoint<f32>,
    red_obs: ParryPoint<f32>,   
    green_obs: ParryPoint<f32>, 
    blue_obs: ParryPoint<f32>,  
    yellow_obs: ParryPoint<f32>
) -> Transform3<f32> {
    let (scaling, translation, rotation) =
        compute_transform_components(red_obs, green_obs, blue_obs, yellow_obs, red_ref, green_ref, blue_ref, yellow_ref);
    create_transform(scaling, rotation, translation)
}

fn create_transform(
    scaling: f32,
    rotation: Rotation3<f32>,
    translation: Translation3<f32>,
) -> Transform3<f32> {
    let scaling_matrix = Matrix4::new_nonuniform_scaling(&Vector3::new(scaling, scaling, scaling));
    Transform3::from_matrix_unchecked(
        scaling_matrix * rotation.to_homogeneous() * translation.to_homogeneous(),
    )
}

fn _compute_transform_components(
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
fn compute_transform_components(
    red_ref: ParryPoint<f32>,  
    green_ref: ParryPoint<f32>,
    blue_ref: ParryPoint<f32>, 
    yellow_ref: ParryPoint<f32>,
    red_obs: ParryPoint<f32>,  
    green_obs: ParryPoint<f32>,
    blue_obs: ParryPoint<f32>, 
    yellow_obs: ParryPoint<f32>
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
    
    println!("Centroid of the reference triangle: {:?}", centroid_ref);

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
    // Determinant must stay negative to have assumed orientation
    let rotation_matrix = if (u * v_t).determinant() > 0.0 {
        let mut u_fixed = u.clone();
        u_fixed.column_mut(2).neg_mut(); // Flip the sign of the last column
        u_fixed * v_t
    } else {
        u * v_t
    };
    let rotation = Rotation3::from_matrix_unchecked(rotation_matrix);
    //let rotation = UnitQuaternion::identity().to_rotation_matrix();

    (scaling, translation, rotation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::relative_eq;

    fn distance(p1: &ParryPoint<f32>, p2: &ParryPoint<f32>) -> f32 {
        ((p2.x - p1.x).powi(2) + (p2.y - p1.y).powi(2) + (p2.z - p1.z).powi(2)).sqrt()
    }

    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_tetrahedron_geometry_properties() {
        let bond_length = 1.0;

        // Call the function
        let (red, green, blue, yellow) = compute_tetrahedron_geometry(bond_length);

        // Check that all base vertices have the same z-coordinate
        assert_abs_diff_eq!(red.z, green.z, epsilon = 1e-6);
        assert_abs_diff_eq!(green.z, blue.z, epsilon = 1e-6);
        assert_abs_diff_eq!(blue.z, red.z, epsilon = 1e-6);

        // Check that the z-coordinate for the base corresponds with 0.0
        assert_abs_diff_eq!(red.z, 0.0, epsilon = 1e-6);

        // Function to calculate distance between two points
        fn distance(a: &ParryPoint<f32>, b: &ParryPoint<f32>) -> f32 {
            ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
        }

        // Calculate pairwise distances between red, green, and blue points
        let red_green_dist = distance(&red, &green);
        let red_blue_dist = distance(&red, &blue);
        let blue_green_dist = distance(&blue, &green);

        // Check that the base triangle forms an equilateral triangle
        assert_abs_diff_eq!(red_green_dist, blue_green_dist, epsilon = 1e-6);
        assert_abs_diff_eq!(red_blue_dist, blue_green_dist, epsilon = 1e-6);
        assert_abs_diff_eq!(red_blue_dist, red_green_dist, epsilon = 1e-6);

        // Extract the side length (all edges of the tetrahedron must be equal)
        let side = red_green_dist;

        // Calculate distances from the yellow point to red, green, and blue points
        let yellow_red_dist = distance(&yellow, &red);
        let yellow_green_dist = distance(&yellow, &green);
        let yellow_blue_dist = distance(&yellow, &blue);

        // Check that the distances from yellow to all base vertices are equal to the tetrahedron's side length
        assert_abs_diff_eq!(yellow_red_dist, side, epsilon = 1e-6);
        assert_abs_diff_eq!(yellow_green_dist, side, epsilon = 1e-6);
        assert_abs_diff_eq!(yellow_blue_dist, side, epsilon = 1e-6);

        // Verify yellow ball is below others
        assert!(red.z > yellow.z);
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

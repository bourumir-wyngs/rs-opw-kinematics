mod tests {
    use nalgebra::{DMatrix, Isometry3, Vector3};
    use parry3d::shape::HeightField;

    const FAR: f32 = 5.0;

    /// Creates a height field based on an elliptical shape. The height of each point is computed
    /// in relation to its distance from the center of the ellipse. The height is normalized,
    /// such that the center of the ellipse has a maximum height of 1.0 and the edges (on the
    /// boundary of the ellipse) transition smoothly to 0.0. Points outside the ellipse have a height of 0.0.
    ///
    /// # Parameters
    /// - `resolution`: The resolution of the height grid (number of discrete points along each axis).
    /// - `a`: The semi-major axis of the ellipse (along the X-direction).
    /// - `b`: The semi-minor axis of the ellipse (along the Z-direction).
    ///
    /// # Returns
    /// A `HeightField` object representing the elliptical height map, scaled to fit the X-Z plane.
    ///
    /// # Description
    /// - The ellipse is defined using the standard ellipse equation: `(x/a)^2 + (z/b)^2 <= 1`.
    /// - Heights are normalized so that the center of the ellipse (`(0, 0)`) has the maximum height,
    ///   gradually decreasing toward the edges of the ellipse. Heights outside the ellipse are set to `0.0`.
    fn create_ellipse_height_field(resolution: usize, a: f32, b: f32) -> (DMatrix<f32>, Vector<f32>)  {
        let size_x = 2.0 * a; // X range: [-a, a]
        let size_y = 2.0 * b; // Z range: [-b, b]
        let step_x = size_x / (resolution - 1) as f32; // Step size for X grid spacing
        let step_y = size_y / (resolution - 1) as f32; // Step size for Z grid spacing

        // Create a 2D matrix to hold the heights for each grid point
        let mut heights = DMatrix::from_element(resolution, resolution, 0.0);

        for x in 0..resolution {
            for y in 0..resolution {
                // Map grid coordinates to the continuous range [-a, a] for x and [-b, b] for z
                let x_pos = -a + x as f32 * step_x; // Convert grid `x` to ellipse X coordinate
                let z_pos = -b + y as f32 * step_y; // Convert grid `y` to ellipse Z coordinate
                heights[(x, y)] = compute_expected_height(a, b, x_pos, z_pos);
            }
        }

        // Construct the Parry height field with scaling.
        // - X is scaled by the ellipse's width (2 * a)
        // - Z is scaled by the ellipse's height (2 * b)
        // - Heights (Y-axis) range from 0.0 to 1.0, representing the elevation
        let scale = Vector3::new(size_x, 1.0, size_y); // Scale X, Y, and Z dimensions
        (heights, scale)
    }

    /// Helper function to compute the expected height at a given position (x, y)
    fn compute_expected_height(a: f32, b: f32, x: f32, y: f32) -> f32 {
        let distance_squared = (x / a).powi(2) + (y / b).powi(2);
        if distance_squared <= 1.0 {
            (1.0 - distance_squared).sqrt()
        } else {
            0.0 // Outside the elliptical boundary
        }
    }

    use approx::assert_abs_diff_eq;
    use nalgebra::Point3;
    use parry3d::math::Vector;
    use parry3d::query::{Ray, RayCast};
    use crate::depth_field::{DepthField, FieldOrientation};

    const RESOLUTION: usize = 64;
    const SEMI_MAJOR_AX: f32 = 2.0; // X dimension
    const SEMI_MINOR_AX: f32 = 1.0; // Z dimension (aligned with Z-axis, as the heightfield is X-Z aligned)

    #[test]
    fn test_height_field_raytrace() {
        let (heights, scale) =
            create_ellipse_height_field(RESOLUTION, SEMI_MAJOR_AX, SEMI_MINOR_AX);
        let height_field = HeightField::new(heights, scale);

        let y_down = Vector3::new(0.0, -1.0, 0.0);
        let center_ray = Ray::new(Point3::new(0.0, 3.0, 0.0), y_down); // Y downwards
        let center_hit =
            height_field.cast_ray_and_get_normal(&Isometry3::identity(), &center_ray, FAR, true);

        assert!(
            center_hit.is_some(),
            "Ray should intersect the height field at the center."
        );
        let center_intersection = center_hit.unwrap();
        let center_point = center_ray.origin + center_ray.dir * center_intersection.time_of_impact;

        let expected_center_height = 1.0;
        assert_abs_diff_eq!(center_point.y, expected_center_height, epsilon = 1e-2);

        // 2. Raytrace from Y downwards at (0.5, 0.0)
        let x_ray = Ray::new(Point3::new(0.5, 3.0, 0.0), y_down); // Y downwards
        let x_hit = height_field.cast_local_ray_and_get_normal(&x_ray, FAR, false);
        let expected_height_x = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.5, 0.0); // (X, Z)
        assert!(
            x_hit.is_some(),
            "Ray should intersect the height field at (0.5, 0.0)."
        );
        let x_intersection = x_hit.unwrap();
        let x_point = x_ray.origin + x_ray.dir * x_intersection.time_of_impact;

        assert_abs_diff_eq!(x_point.y, expected_height_x, epsilon = 1e-2);

        let z_ray = Ray::new(Point3::new(0.0, 3.0, 0.5), y_down); // Y downwards
        let z_hit = height_field.cast_local_ray_and_get_normal(&z_ray, FAR, false);
        let expected_height_z = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.0, 0.5); // (X, Z)

        assert!(
            z_hit.is_some(),
            "Ray should intersect the height field at (0.0, 0.5)."
        );
        let z_intersection = z_hit.unwrap();
        let z_point = z_ray.origin + z_ray.dir * z_intersection.time_of_impact;

        assert_abs_diff_eq!(z_point.y, expected_height_z, epsilon = 1e-2);
    }

    #[test]
    fn test_depth_field_y() {
        let (heights, scale) =
            create_ellipse_height_field(RESOLUTION, SEMI_MAJOR_AX, SEMI_MINOR_AX);
        let height_field = DepthField::new(heights, scale, FieldOrientation::Y);

        let y_down = Vector3::new(0.0, -1.0, 0.0);
        let center_ray = Ray::new(Point3::new(0.0, 3.0, 0.0), y_down); 
        let center_hit =
            height_field.cast_ray_and_get_normal(&Isometry3::identity(), &center_ray, FAR, true);

        assert!(
            center_hit.is_some(),
            "Ray should intersect the height field at the center."
        );
        let center_intersection = center_hit.unwrap();
        let center_point = center_ray.origin + center_ray.dir * center_intersection.time_of_impact;

        let expected_center_height = 1.0;
        assert_abs_diff_eq!(center_point.y, expected_center_height, epsilon = 1e-2);

        // 2. Raytrace from Y downwards at (0.5, 0.0)
        let x_ray = Ray::new(Point3::new(0.5, 3.0, 0.0), y_down); 
        let x_hit = height_field.cast_local_ray_and_get_normal(&x_ray, FAR, false);
        let expected_height_x = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.5, 0.0); // (X, Z)
        assert!(
            x_hit.is_some(),
            "Ray should intersect the height field at (0.5, 0.0)."
        );
        let x_intersection = x_hit.unwrap();
        let x_point = x_ray.origin + x_ray.dir * x_intersection.time_of_impact;

        assert_abs_diff_eq!(x_point.y, expected_height_x, epsilon = 1e-2);

        let z_ray = Ray::new(Point3::new(0.0, 3.0, 0.5), y_down); 
        let z_hit = height_field.cast_local_ray_and_get_normal(&z_ray, FAR, false);
        let expected_height_z = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.0, 0.5); // (X, Z)

        assert!(
            z_hit.is_some(),
            "Ray should intersect the height field at (0.0, 0.5)."
        );
        let z_intersection = z_hit.unwrap();
        let z_point = z_ray.origin + z_ray.dir * z_intersection.time_of_impact;

        assert_abs_diff_eq!(z_point.y, expected_height_z, epsilon = 1e-2);
    }

    #[test]
    fn test_depth_field_anti_y() {
        let (heights, scale) =
            create_ellipse_height_field(RESOLUTION, SEMI_MAJOR_AX, SEMI_MINOR_AX);
        let height_field = DepthField::new(heights, scale, FieldOrientation::AntiY);

        let y_up = Vector3::new(0.0, 1.0, 0.0);
        let center_ray = Ray::new(Point3::new(0.0, -3.0, 0.0), y_up); 
        let center_hit =
            height_field.cast_ray_and_get_normal(&Isometry3::identity(), &center_ray, FAR, true);

        assert!(
            center_hit.is_some(),
            "Ray should intersect the height field at the center."
        );
        let center_intersection = center_hit.unwrap();
        let center_point = center_ray.origin + center_ray.dir * center_intersection.time_of_impact;

        let expected_center_height = -1.0;
        assert_abs_diff_eq!(center_point.y, expected_center_height, epsilon = 1e-2);

        // 2. Raytrace from Y downwards at (0.5, 0.0)
        let x_ray = Ray::new(Point3::new(0.5, -3.0, 0.0), y_up); 
        let x_hit = height_field.cast_local_ray_and_get_normal(&x_ray, FAR, false);
        let expected_height_x = -compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.5, 0.0); // (X, Z)
        assert!(
            x_hit.is_some(),
            "Ray should intersect the height field at (0.5, 0.0)."
        );
        let x_intersection = x_hit.unwrap();
        let x_point = x_ray.origin + x_ray.dir * x_intersection.time_of_impact;

        assert_abs_diff_eq!(x_point.y, expected_height_x, epsilon = 1e-2);

        let z_ray = Ray::new(Point3::new(0.0, -3.0, 0.5), y_up); 
        let z_hit = height_field.cast_local_ray_and_get_normal(&z_ray, FAR, false);
        let expected_height_z = -compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.0, 0.5); // (X, Z)

        assert!(
            z_hit.is_some(),
            "Ray should intersect the height field at (0.0, 0.5)."
        );
        let z_intersection = z_hit.unwrap();
        let z_point = z_ray.origin + z_ray.dir * z_intersection.time_of_impact;

        assert_abs_diff_eq!(z_point.y, expected_height_z, epsilon = 1e-2);
    }

    #[test]
    fn test_depth_field_z() {
        let (heights, scale) =
            create_ellipse_height_field(RESOLUTION, SEMI_MAJOR_AX, SEMI_MINOR_AX);
        let height_field = DepthField::new(heights, scale, FieldOrientation::Z);

        let z_down = Vector3::new(0.0, 0.0, -1.0);
        let center_ray = Ray::new(Point3::new(0.0, 0.0, 3.0), z_down);
        let center_hit =
            height_field.cast_ray_and_get_normal(&Isometry3::identity(), &center_ray, FAR, true);

        assert!(
            center_hit.is_some(),
            "Ray should intersect the height field at the center."
        );
        let center_intersection = center_hit.unwrap();
        let center_point = center_ray.origin + center_ray.dir * center_intersection.time_of_impact;

        let expected_center_height = 1.0;
        assert_abs_diff_eq!(center_point.z, expected_center_height, epsilon = 1e-2);

        // 2. Raytrace from Y downwards at (0.5, 0.0)
        let x_ray = Ray::new(Point3::new(0.5, 0.0, 3.0), z_down);
        let x_hit = height_field.cast_local_ray_and_get_normal(&x_ray, FAR, false);
        let expected_height_x = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.5, 0.0); // (X, Z)
        assert!(
            x_hit.is_some(),
            "Ray should intersect the height field at (0.5, 0.0)."
        );
        let x_intersection = x_hit.unwrap();
        let x_point = x_ray.origin + x_ray.dir * x_intersection.time_of_impact;

        assert_abs_diff_eq!(x_point.z, expected_height_x, epsilon = 1e-2);

        let y_ray = Ray::new(Point3::new(0.0, 0.5, 3.0), z_down);
        let y_hit = height_field.cast_local_ray_and_get_normal(&y_ray, FAR, false);
        let expected_height_z = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.0, 0.5); // (X, Z)

        assert!(
            y_hit.is_some(),
            "Ray should intersect the height field at (0.0, 0.5)."
        );
        let y_intersection = y_hit.unwrap();
        let y_point = y_ray.origin + y_ray.dir * y_intersection.time_of_impact;

        assert_abs_diff_eq!(y_point.z, expected_height_z, epsilon = 1e-2);
    }

    #[test]
    fn test_depth_field_anti_z() {
        let (heights, scale) =
            create_ellipse_height_field(RESOLUTION, SEMI_MAJOR_AX, SEMI_MINOR_AX);
        let height_field = DepthField::new(heights, scale, FieldOrientation::AntiZ);

        let z_up = Vector3::new(0.0, 0.0, 1.0);
        let center_ray = Ray::new(Point3::new(0.0, 0.0, -3.0), z_up);
        let center_hit =
            height_field.cast_ray_and_get_normal(&Isometry3::identity(), &center_ray, FAR, true);

        assert!(
            center_hit.is_some(),
            "Ray should intersect the height field at the center."
        );
        let center_intersection = center_hit.unwrap();
        let center_point = center_ray.origin + center_ray.dir * center_intersection.time_of_impact;

        let expected_center_height = -1.0;
        assert_abs_diff_eq!(center_point.z, expected_center_height, epsilon = 1e-2);

        // 2. Raytrace from Y downwards at (0.5, 0.0)
        let x_ray = Ray::new(Point3::new(0.5, 0.0, -3.0), z_up);
        let x_hit = height_field.cast_local_ray_and_get_normal(&x_ray, FAR, false);
        let expected_height_x = -compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.5, 0.0); // (X, Z)
        assert!(
            x_hit.is_some(),
            "Ray should intersect the height field at (0.5, 0.0)."
        );
        let x_intersection = x_hit.unwrap();
        let x_point = x_ray.origin + x_ray.dir * x_intersection.time_of_impact;

        assert_abs_diff_eq!(x_point.z, expected_height_x, epsilon = 1e-2);

        let y_ray = Ray::new(Point3::new(0.0, 0.5, -3.0), z_up);
        let y_hit = height_field.cast_local_ray_and_get_normal(&y_ray, FAR, false);
        let expected_height_z = -compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.0, 0.5); // (X, Z)

        assert!(
            y_hit.is_some(),
            "Ray should intersect the height field at (0.0, 0.5)."
        );
        let y_intersection = y_hit.unwrap();
        let y_point = y_ray.origin + y_ray.dir * y_intersection.time_of_impact;

        assert_abs_diff_eq!(y_point.z, expected_height_z, epsilon = 1e-2);
    }

    #[test]
    fn test_depth_field_x() {
        let (heights, scale) =
            create_ellipse_height_field(RESOLUTION, SEMI_MAJOR_AX, SEMI_MINOR_AX);
        let height_field = DepthField::new(heights, scale, FieldOrientation::X);

        let x_down = Vector3::new(-1.0, 0.0, 0.0);
        let center_ray = Ray::new(Point3::new(3.0, 0.0, 0.0), x_down);
        let center_hit =
            height_field.cast_ray_and_get_normal(&Isometry3::identity(), &center_ray, FAR, true);

        assert!(
            center_hit.is_some(),
            "Ray should intersect the height field at the center."
        );
        let center_intersection = center_hit.unwrap();
        let center_point = center_ray.origin + center_ray.dir * center_intersection.time_of_impact;

        let expected_center_height = 1.0;
        assert_abs_diff_eq!(center_point.x, expected_center_height, epsilon = 1e-2);

        let y_ray = Ray::new(Point3::new(3.0, 0.5, 0.0), x_down);
        let y_hit = height_field.cast_local_ray_and_get_normal(&y_ray, FAR, false);
        let expected_height_y = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.5, 0.0); // (X, Z)
        assert!(
            y_hit.is_some(),
            "Ray should intersect the height field at (0.5, 0.0)."
        );
        let y_intersection = y_hit.unwrap();
        let y_point = y_ray.origin + y_ray.dir * y_intersection.time_of_impact;

        assert_abs_diff_eq!(y_point.x, expected_height_y, epsilon = 1e-2);

        let z_ray = Ray::new(Point3::new(3.0, 0.0, 0.5), x_down);
        let z_hit = height_field.cast_local_ray_and_get_normal(&z_ray, FAR, false);
        let expected_height_z = compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.0, 0.5); // (X, Z)

        assert!(
            z_hit.is_some(),
            "Ray should intersect the height field at (0.0, 0.5)."
        );
        let z_intersection = z_hit.unwrap();
        let z_point = z_ray.origin + z_ray.dir * z_intersection.time_of_impact;

        assert_abs_diff_eq!(z_point.x, expected_height_z, epsilon = 1e-2);
    }

    #[test]
    fn test_depth_field_anti_x() {
        let (heights, scale) =
            create_ellipse_height_field(RESOLUTION, SEMI_MAJOR_AX, SEMI_MINOR_AX);
        let height_field = DepthField::new(heights, scale, FieldOrientation::AntiX);

        let x_up = Vector3::new(1.0, 0.0, 0.0);
        let center_ray = Ray::new(Point3::new(-3.0, 0.0, 0.0), x_up);
        let center_hit =
            height_field.cast_ray_and_get_normal(&Isometry3::identity(), &center_ray, FAR, true);

        assert!(
            center_hit.is_some(),
            "Ray should intersect the height field at the center."
        );
        let center_intersection = center_hit.unwrap();
        let center_point = center_ray.origin + center_ray.dir * center_intersection.time_of_impact;

        let expected_center_height = -1.0;
        assert_abs_diff_eq!(center_point.x, expected_center_height, epsilon = 1e-2);

        let y_ray = Ray::new(Point3::new(-3.0, 0.5, 0.0), x_up);
        let y_hit = height_field.cast_local_ray_and_get_normal(&y_ray, FAR, false);
        let expected_height_x = -compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.5, 0.0); // (X, Z)
        assert!(
            y_hit.is_some(),
            "Ray should intersect the height field at (0.5, 0.0)."
        );
        let y_intersection = y_hit.unwrap();
        let y_point = y_ray.origin + y_ray.dir * y_intersection.time_of_impact;

        assert_abs_diff_eq!(y_point.x, expected_height_x, epsilon = 1e-2);

        let z_ray = Ray::new(Point3::new( -3.0, 0.0,0.5), x_up);
        let z_hit = height_field.cast_local_ray_and_get_normal(&z_ray, FAR, false);
        let expected_height_z = -compute_expected_height(SEMI_MAJOR_AX, SEMI_MINOR_AX, 0.0, 0.5); // (X, Z)

        assert!(
            z_hit.is_some(),
            "Ray should intersect the height field at (0.0, 0.5)."
        );
        let z_intersection = z_hit.unwrap();
        let z_point = z_ray.origin + z_ray.dir * z_intersection.time_of_impact;

        assert_abs_diff_eq!(z_point.x, expected_height_z, epsilon = 1e-2);
    }    
}

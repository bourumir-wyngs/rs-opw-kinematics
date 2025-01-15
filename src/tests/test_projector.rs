#[cfg(test)]
mod tests {
    use crate::annotations::AnnotatedPose;
    use crate::engraving::{generate_raster_points, project_flat_to_rect_on_mesh};
    use crate::projector::{Axis, Projector, RayDirection};
    use crate::synthetic_meshes::sphere_mesh;
    use nalgebra::{Quaternion, Translation3, UnitQuaternion};
    use serde_json::Value;
    use std::f32::consts::PI;
    use std::fs::File;
    use std::io::Read;
    use std::time::Instant;

    fn read_isometries_from_file(file_path: &str) -> Result<Vec<AnnotatedPose>, String> {
        let mut file = File::open(file_path).map_err(|e| format!("Failed to open file: {}", e))?;
        let mut content = String::new();
        file.read_to_string(&mut content)
            .map_err(|e| format!("Failed to read file: {}", e))?;

        // Parse the JSON content into a serde_json Value
        let parsed: Value =
            serde_json::from_str(&content).map_err(|e| format!("Failed to parse JSON: {}", e))?;

        // Extract the array of objects
        let isometry_objects = parsed.as_array().ok_or("JSON is not an array")?;

        let mut isometries = Vec::new();

        for obj in isometry_objects {
            // Parse position
            let position = obj.get("position").ok_or("Missing 'position' field")?;
            let px = position
                .get("x")
                .and_then(|x| x.as_f64())
                .ok_or("Missing or invalid 'x' in position")?;
            let py = position
                .get("y")
                .and_then(|y| y.as_f64())
                .ok_or("Missing or invalid 'y' in position")?;
            let pz = position
                .get("z")
                .and_then(|z| z.as_f64())
                .ok_or("Missing or invalid 'z' in position")?;

            let translation = Translation3::new(px as f32, py as f32, pz as f32);

            // Parse rotation
            let rotation = obj.get("rotation").ok_or("Missing 'rotation' field")?;
            let rx = rotation
                .get("x")
                .and_then(|x| x.as_f64())
                .ok_or("Missing or invalid 'x' in rotation")?;
            let ry = rotation
                .get("y")
                .and_then(|y| y.as_f64())
                .ok_or("Missing or invalid 'y' in rotation")?;
            let rz = rotation
                .get("z")
                .and_then(|z| z.as_f64())
                .ok_or("Missing or invalid 'z' in rotation")?;
            let rw = rotation
                .get("w")
                .and_then(|w| w.as_f64())
                .ok_or("Missing or invalid 'w' in rotation")?;

            let quaternion = Quaternion::new(rw as f32, rx as f32, ry as f32, rz as f32);

            // Combine into an Isometry3
            let isometry =
                AnnotatedPose::from_parts(translation, UnitQuaternion::from_quaternion(quaternion));
            isometries.push(isometry);
        }

        Ok(isometries)
    }

    fn assert_path(projections: Vec<AnnotatedPose>, engraving: Vec<AnnotatedPose>) {
        // Assert that the lengths of `projections` and `engraving` are the same
        assert_eq!(
            projections.len(),
            engraving.len(),
            "Projections and engravings must have the same number of elements"
        );

        // Check each pair for approximate equality
        for (i, (proj, engr)) in projections.iter().zip(engraving.iter()).enumerate() {
            assert!(
                isometry_approx_eq(proj, engr, 1E-5),
                "Mismatch at index {}: {:?} != {:?}",
                i,
                proj,
                engr
            );
        }
    }

    fn isometry_approx_eq(a: &AnnotatedPose, b: &AnnotatedPose, tolerance: f64) -> bool {
        // Compare translation components
        let translation_eq =
            (a.pose.translation.vector - b.pose.translation.vector).norm() <= tolerance;

        // Compare rotation components
        let relative_rotation = a.pose.rotation.inverse() * b.pose.rotation; // Relative rotation: `b` relative to `a`
        let angle_difference = relative_rotation.angle(); // Angular difference in radians
        let rotation_eq = angle_difference <= tolerance;

        translation_eq && rotation_eq
    }
     

    static PROJECTOR: Projector = Projector {
        check_points: 24, // Defined number of normals to check
        radius: 0.02,     // Radius, defined as PROJECTOR_RADIUS
    };

    /// Project rectanglar mesh rolled into cylinder, on the surface of sphere.
    fn cyl_on_sphere(axis: Axis) -> Result<(), String> {
        let json_path = format!("src/tests/data/projector/cyl_on_sphere_{:?}.json", axis);
        let projections = read_isometries_from_file(&json_path).expect("Cannot read test data");
        let path = generate_raster_points(40, 200);

        let t_ep = Instant::now();
        let engraving = PROJECTOR.project_cylinder_path(
            &sphere_mesh(0.5, 256),
            &path,
            1.0,
            -1.5..1.5,
            0. ..2.0 * PI,
            axis,
        )?;

        println!(
            "Cylinder, axis {:?}, on sphere: engraving {:?}",
            axis,
            t_ep.elapsed()
        );

        assert_path(projections, engraving);
        Ok(())
    }

    fn dir_on_sphere(axis: Axis, direction: RayDirection) -> Result<(), String> {
        let json_path = format!(
            "src/tests/data/projector/flat_on_sphere_{:?}_{:?}.json",
            axis, direction
        );
        let projections = read_isometries_from_file(&json_path).expect("Cannot read test data");
        let path = generate_raster_points(20, 20);
        let engraving =
            project_flat_to_rect_on_mesh(&PROJECTOR, &sphere_mesh(0.5, 256), &path, axis, direction)?;
        assert_path(projections, engraving);
        Ok(())
    }

    #[test]
    fn test_cyl_on_sphere_x() -> Result<(), String> {
        cyl_on_sphere(Axis::X)?;
        Ok(())
    }

    #[test]
    fn test_cyl_on_sphere_y() -> Result<(), String> {
        cyl_on_sphere(Axis::Y)?;
        Ok(())
    }

    #[test]
    fn test_cyl_on_sphere_z() -> Result<(), String> {
        cyl_on_sphere(Axis::Z)?;
        Ok(())
    }

    #[test]
    fn test_dir_on_sphere_x_pos() -> Result<(), String> {
        dir_on_sphere(Axis::X, RayDirection::FromPositive)?;
        Ok(())
    }

    #[test]
    fn test_dir_on_sphere_y_pos() -> Result<(), String> {
        dir_on_sphere(Axis::Y, RayDirection::FromPositive)?;
        Ok(())
    }

    #[test]
    fn test_dir_on_sphere_z_pos() -> Result<(), String> {
        dir_on_sphere(Axis::Z, RayDirection::FromPositive)?;
        Ok(())
    }

    #[test]
    fn test_dir_on_sphere_x_neg() -> Result<(), String> {
        dir_on_sphere(Axis::X, RayDirection::FromNegative)?;
        Ok(())
    }

    #[test]
    fn test_dir_on_sphere_y_neg() -> Result<(), String> {
        dir_on_sphere(Axis::Y, RayDirection::FromNegative)?;
        Ok(())
    }

    #[test]
    fn test_dir_on_sphere_z_neg() -> Result<(), String> {
        dir_on_sphere(Axis::Z, RayDirection::FromNegative)?;
        Ok(())
    }
}

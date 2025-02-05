use nalgebra::{Point3, Matrix4, Transform3};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct TransformJson {
    serial: String,
    matrix: [[f32; 4]; 4],
}

pub fn transform_to_json(serial: &String, transform: &Transform3<f32>) -> String {
    let json_data = TransformJson {
        serial: serial.clone(),
        matrix: (*transform.matrix()).into(),
    };

    serde_json::to_string_pretty(&json_data).unwrap()
}

pub fn json_to_transform(json_str: &str) -> Transform3<f32> {
    let json_data: TransformJson = serde_json::from_str(json_str).unwrap();
    Transform3::from_matrix_unchecked(Matrix4::from(json_data.matrix))
}

#[cfg(test)]
mod tests {
    use approx::RelativeEq;
    use nalgebra::Point;
    use super::*;

    #[test]
    fn test_transform_serialization() {
        let theta = std::f32::consts::FRAC_PI_3;
        let serial: String = "1234565".to_string();

        let original_transform = Transform3::from_matrix_unchecked(Matrix4::new(
            1.5 * theta.cos(), -1.5 * theta.sin(), 0.0, 1.0,  // First row
            1.5 * theta.sin(), 1.5 * theta.cos(), 0.0, 2.0,  // Second row
            0.0, 0.0, 1.5, 3.0,  // Third row
            0.0, 0.0, 0.0, 1.0,  // Fourth row
        ));


        let json = transform_to_json(&serial, &original_transform);
        let loaded_transform = json_to_transform(&json);

        let test_point = Point3::new(1.0, 3.0, 9.0);
        let transformed_point_original = original_transform.transform_point(&test_point);
        let transformed_point_loaded = loaded_transform.transform_point(&test_point);

        assert!(
            transformed_point_original.relative_eq(&transformed_point_loaded, 1e-6, 1e-6),
            "Transformed points do not match! Original: {:?}, Loaded: {:?}",
            transformed_point_original,
            transformed_point_loaded
        );
    }
}

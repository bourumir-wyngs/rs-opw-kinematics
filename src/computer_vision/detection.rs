use crate::computer_vision::{detect_circle_mat, Detection};
use crate::hsv::{ColorId, DefinedColor};
use opencv::core::Mat;
use rayon::prelude::*;
use std::collections::HashMap;

pub fn detect_circles(img: &Mat) -> HashMap<ColorId, Detection> {
    // List of primary colors to probe
    let colors = vec![
        DefinedColor::red(),
        DefinedColor::green(),
        DefinedColor::blue(),
    ];

    // Use parallel iterator to process each color and collect results into a vector
    let results: Vec<(ColorId, Detection)> = colors
        .into_par_iter()
        .filter_map(|color| {
            // Try detecting circles for the current color
            if let Ok(detection) = detect_circle_mat(img, &color) {
                // Return the color ID and detection if successfully detected
                return Some((color.id(), detection));
            }
            None // Skip if detection is unsuccessful
        })
        .collect();

    // Convert the vector of results into a HashMap
    results.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use opencv::imgcodecs;

    #[test]
    fn test_detect_circles() -> Result<(), Box<dyn std::error::Error>> {
        let image_path = "src/tests/data/vision/b_Color.png";

        let img = imgcodecs::imread(image_path, imgcodecs::IMREAD_COLOR)?;
        let detections = detect_circles(&img);

        // Expected detections
        let expected_detections = HashMap::from([
            (
                ColorId::Green,
                Detection {
                    color: ColorId::Green,
                    x: 228,
                    y: 96,
                },
            ),
            (
                ColorId::Blue,
                Detection {
                    color: ColorId::Blue,
                    x: 308,
                    y: 97,
                },
            ),
            (
                ColorId::Red,
                Detection {
                    color: ColorId::Red,
                    x: 218,
                    y: 124,
                },
            ),
        ]);

        println!("Detections: {:?}", detections);

        // Ensure the number of detections matches
        assert_eq!(
            detections.len(),
            expected_detections.len(),
            "Number of detections does not match"
        );

        // Ensure all expected detections are present
        for (color, detection) in expected_detections {
            assert!(
                detections.get(&color).is_some(),
                "Missing detection for color: {:?}",
                color
            );
            assert_eq!(
                detections[&color], detection,
                "Detection for color {:?} does not match. Expected {:?}, got {:?}",
                color, detection, detections[&color]
            );
        }

        Ok(())
    }
}

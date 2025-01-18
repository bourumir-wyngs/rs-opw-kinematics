use crate::computer_vision::{detect_circle, Detection};
use crate::hsv::{ColorId, DefinedColor};
use image::DynamicImage;
use std::collections::HashMap;
use rayon::prelude::*;

pub fn detect_circles(img: &DynamicImage) -> HashMap<ColorId, Detection> {
    // List of primary colors to probe
    let colors = vec![
        DefinedColor::red(),
        DefinedColor::green(),
        DefinedColor::blue(),
        DefinedColor::yellow(),
        DefinedColor::cyan(),
        DefinedColor::magenta(),
    ];

    // Use parallel iterator to process each color and collect results into a vector
    let results: Vec<(ColorId, Detection)> = colors
        .into_par_iter()
        .filter_map(|color| {
            // Try detecting circles for the current color
            if let Ok(detection) = detect_circle(img, &color) {
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
    use image::io::Reader as ImageReader;

    #[test]
    fn test_detect_circles() -> Result<(), String> {
        let image_path = "src/tests/data/vision/rg_e.png";

        let img = ImageReader::open(image_path)
            .map_err(|e| format!("Failed to open image: {}", e))?
            .decode()
            .map_err(|e| format!("Failed to decode image: {}", e))?;

        let detections = detect_circles(&img);

        // Expected detections
        let expected_detections = HashMap::from([
            (ColorId::Yellow, Detection { x: 332, y: 202, r: 32 }),
            (ColorId::Green, Detection { x: 534, y: 434, r: 32 }),
            (ColorId::Blue, Detection { x: 536, y: 202, r: 32 }),
            (ColorId::Red, Detection { x: 332, y: 432, r: 32 }),
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

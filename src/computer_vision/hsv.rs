use image::Rgba;

/// Represents a range in the HSV color space.
/// It supports ranges for hue, saturation, and value components.
/// Hue ranges can wrap around 360 in HSV space (e.g., red, which spans [0, 24] and [350, 360]).
#[derive(Debug, Clone)]
pub struct HSVRange {
    hue_ranges: Vec<(f64, f64)>, // A list of hue subranges, supporting wrap-around
    saturation_range: (f64, f64), // Min and max saturation
    value_range: (f64, f64),     // Min and max value
}

/// Converts an RGB pixel to HSV.
/// HSV ranges:
/// - H: [0, 360] (hue)
/// - S: [0, 1] (saturation)
/// - V: [0, 1] (value)
pub fn rgb_to_hsv(r: u8, g: u8, b: u8) -> (f64, f64, f64) {
    let r = r as f64 / 255.0;
    let g = g as f64 / 255.0;
    let b = b as f64 / 255.0;

    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let delta = max - min;

    let h = if delta == 0.0 {
        0.0
    } else if max == r {
        60.0 * (((g - b) / delta) % 6.0)
    } else if max == g {
        60.0 * (((b - r) / delta) + 2.0)
    } else {
        60.0 * (((r - g) / delta) + 4.0)
    };

    let hue = if h < 0.0 { h + 360.0 } else { h };
    let saturation = if max == 0.0 { 0.0 } else { delta / max };
    let value = max;

    (hue, saturation, value)
}

impl HSVRange {
    /// Creates a new `HSVRange`.
    /// Takes multiple hue ranges to support hue wrap-around (e.g., red color).
    pub fn new(
        hue_ranges: Vec<(f64, f64)>,
        saturation_range: (f64, f64),
        value_range: (f64, f64),
    ) -> Self {
        HSVRange {
            hue_ranges,
            saturation_range,
            value_range,
        }
    }

    /// Checks if the given RGB value falls within this HSV range.
    pub fn this_color(&self, r: u8, g: u8, b: u8) -> bool {
        let (h, s, v) = rgb_to_hsv(r, g, b);

        // Check if the saturation and value are within their respective ranges
        let in_saturation_range = s >= self.saturation_range.0 && s <= self.saturation_range.1;
        let in_value_range = v >= self.value_range.0 && v <= self.value_range.1;

        if in_saturation_range && in_value_range {
            // Check if the hue falls within any of the defined hue ranges
            self.hue_ranges
                .iter()
                .any(|&(min_h, max_h)| h >= min_h && h <= max_h)
        } else {
            false
        }
    }
}

/// Enum representing primary colors with associated HSV ranges.
#[derive(Debug, Clone)]
pub enum PrimaryColor {
    Red { range: HSVRange },
    Green { range: HSVRange },
    Blue { range: HSVRange },
    Yellow { range: HSVRange },
    Cyan { range: HSVRange },
    Magenta { range: HSVRange },
}

impl PrimaryColor {
    /// Creates an instance of the Red color with its HSV range.
    pub fn red() -> Self {
        PrimaryColor::Red {
            range: HSVRange {
                hue_ranges: vec![(0.0, 24.0), (350.0, 360.0)], // Red hue range
                saturation_range: (0.5, 1.0),                  // Saturation range
                value_range: (0.2, 1.0),                       // Value range
            },
        }
    }

    /// Creates an instance of the Green color with its HSV range.
    pub fn green() -> Self {
        PrimaryColor::Green {
            range: HSVRange {
                hue_ranges: vec![(85.0, 160.0)], // Green hue range
                saturation_range: (0.5, 1.0),
                value_range: (0.2, 1.0),
            },
        }
    }

    /// Creates an instance of the Blue color with its HSV range.
    pub fn blue() -> Self {
        PrimaryColor::Blue {
            range: HSVRange {
                hue_ranges: vec![(210.0, 270.0)], // Blue hue range
                saturation_range: (0.5, 1.0),
                value_range: (0.2, 1.0),
            },
        }
    }

    /// Creates an instance of the Yellow color with its HSV range.
    pub fn yellow() -> Self {
        PrimaryColor::Yellow {
            range: HSVRange {
                hue_ranges: vec![(40.0, 65.0)], // Yellow hue range
                saturation_range: (0.5, 1.0),
                value_range: (0.2, 1.0),
            },
        }
    }

    /// Creates an instance of the Cyan color with its HSV range.
    pub fn cyan() -> Self {
        PrimaryColor::Cyan {
            range: HSVRange {
                hue_ranges: vec![(180.0, 200.0)], // Cyan hue range
                saturation_range: (0.5, 1.0),
                value_range: (0.2, 1.0),
            },
        }
    }

    /// Creates an instance of the Magenta color with its HSV range.
    pub fn magenta() -> Self {
        PrimaryColor::Magenta {
            range: HSVRange {
                hue_ranges: vec![(290.0, 320.0)], // Magenta hue range
                saturation_range: (0.5, 1.0),
                value_range: (0.2, 1.0),
            },
        }
    }

    /// Determines if the given RGB value matches the HSV range for this primary color.
    pub fn this_color(&self, color: &Rgba<u8>) -> bool {
        let (r, g, b) = (color.0[0], color.0[1], color.0[2]);
        match self {
            PrimaryColor::Red { range }
            | PrimaryColor::Green { range }
            | PrimaryColor::Blue { range }
            | PrimaryColor::Yellow { range }
            | PrimaryColor::Cyan { range }
            | PrimaryColor::Magenta { range } => range.this_color(r, g, b),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hsv_range_red_detection() {
        // Define two hue ranges for the red color
        let red_hsv_range = HSVRange::new(
            vec![(0.0, 24.0), (350.0, 360.0)], // Red hue ranges
            (0.5, 1.0),                        // Saturation range
            (0.2, 1.0),                        // Value range
        );

        // Test cases for red
        let rgb_red = (255, 0, 0); // Bright red
        let rgb_dark_red = (139, 0, 0); // Dark red
        let rgb_pink = (255, 20, 147); // Pink (not pure red)
        let rgb_blue = (0, 0, 255); // Blue (not red)

        // Ensure red and dark red are in range
        assert!(red_hsv_range.this_color(rgb_red.0, rgb_red.1, rgb_red.2));
        assert!(red_hsv_range.this_color(rgb_dark_red.0, rgb_dark_red.1, rgb_dark_red.2));

        // Pink and blue should NOT be in range
        assert!(!red_hsv_range.this_color(rgb_pink.0, rgb_pink.1, rgb_pink.2));
        assert!(!red_hsv_range.this_color(rgb_blue.0, rgb_blue.1, rgb_blue.2));
    }

    #[test]
    fn test_primary_color_detection() {
        // Create instances of each primary color
        let red = PrimaryColor::red();
        let green = PrimaryColor::green();
        let blue = PrimaryColor::blue();
        let yellow = PrimaryColor::yellow();
        let cyan = PrimaryColor::cyan();
        let magenta = PrimaryColor::magenta();

        // Define RGB values for each primary color
        let red_rgb: Rgba<u8> = Rgba([255, 0, 0, 0]); // Bright red
        let green_rgb: Rgba<u8> = Rgba([0, 255, 0, 0]); // Bright green
        let blue_rgb: Rgba<u8> = Rgba([0, 0, 255, 0]); // Bright blue
        let yellow_rgb: Rgba<u8> = Rgba([255, 255, 0, 0]); // Bright yellow (mix of red and green)
        let cyan_rgb: Rgba<u8> = Rgba([0, 255, 255, 0]); // Bright cyan (mix of green and blue)
        let magenta_rgb: Rgba<u8> = Rgba([255, 0, 255, 0]); // Bright magenta (mix of red and blue)
        let gray_rgb: Rgba<u8> = Rgba([128, 128, 128, 0]); // Gray (no saturation, shouldn't match any color)

        // Check that the color is properly detected for each primary color
        assert!(red.this_color(&red_rgb), "Red color detection failed");
        assert!(green.this_color(&green_rgb), "Green color detection failed");
        assert!(blue.this_color(&blue_rgb), "Blue color detection failed");
        assert!(
            yellow.this_color(&yellow_rgb),
            "Yellow color detection failed"
        );
        assert!(cyan.this_color(&cyan_rgb), "Cyan color detection failed");
        assert!(
            magenta.this_color(&magenta_rgb),
            "Magenta color detection failed"
        );

        // Check that gray is never matched by any primary color
        assert!(!red.this_color(&gray_rgb), "Gray incorrectly matched as Red");
        assert!(
            !green.this_color(&gray_rgb),
            "Gray incorrectly matched as Green"
        );
        assert!(
            !blue.this_color(&gray_rgb),
            "Gray incorrectly matched as Blue"
        );
        assert!(
            !yellow.this_color(&gray_rgb),
            "Gray incorrectly matched as Yellow"
        );
        assert!(
            !cyan.this_color(&gray_rgb),
            "Gray incorrectly matched as Cyan"
        );
        assert!(
            !magenta.this_color(&gray_rgb),
            "Gray incorrectly matched as Magenta"
        );
    }
}

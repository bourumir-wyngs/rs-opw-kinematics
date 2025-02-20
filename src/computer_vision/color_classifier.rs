use anyhow::Result;
use csv::ReaderBuilder;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use std::path::Path;
use once_cell::sync::Lazy;
use crate::colors::ColorId;
use crate::computer_vision::IS_MATHING_MIN_THR;

// Define the static instance (this will be initialized lazily)
static COLOR_CLASSIFIER: Lazy<ColorClassifier> = Lazy::new(|| {
    ColorClassifier::new(&[
        "scripts/colors_red.csv",
        "scripts/colors_green.csv",
        "scripts/colors_blue.csv",
        "scripts/colors_yellow.csv",
        "scripts/colors_metal.csv",
        "scripts/colors_other.csv",
        "scripts/colors_black.csv",
        "scripts/colors_copper.csv",        
    ])
        .expect("Failed to initialize ColorClassifier")
});

pub fn classify_color(color: (u8, u8, u8)) -> ColorId {
    COLOR_CLASSIFIER.classify(color)
}

pub fn estimate_color_score(expect: ColorId, color: (u8, u8, u8)) -> u16 {
    COLOR_CLASSIFIER.score(expect, color)
}


#[derive(Debug, Clone)]
struct ColorClassifier {
    tree: KdTree<f32, ColorId, [f32; 3]>, // KDTree to store RGB colors and their classifications
}

impl ColorClassifier {
    /// Create a new classifier by reading three CSV files
    pub fn new(file_paths: &[&str]) -> Result<Self> {
        let mut tree = KdTree::new(3); // Create a 3-dimensional KDTree

        for path in file_paths {
            ColorClassifier::read_file_and_add_to_tree(path, &mut tree)?;
        }

        Ok(ColorClassifier { tree })
    }

    /// Reads a single CSV file and adds its colors to the KD-tree
    fn read_file_and_add_to_tree(path: &str, tree: &mut KdTree<f32, ColorId, [f32; 3]>) -> Result<()> {
        let mut reader = ReaderBuilder::new().has_headers(true).from_path(Path::new(path))?;
        for record in reader.records() {
            let record = record?;
            let channel = record[0].to_string();
            let label  = match channel.as_str() {
                "r" => ColorId::Red,
                "g" => ColorId::Green,
                "b" => ColorId::Blue,
                "m" => ColorId::Blue, // Magenta is currenly synonym of blue
                "y" => ColorId::Yellow,
                "x" => ColorId::Other,
                _ => panic!("Invalid channel: {}", channel),
            };
            
            // Read the "r", "g", or "b" label
            let r: f32 = record[1].parse()?;    
            let g: f32 = record[2].parse()?;    
            let b: f32 = record[3].parse()?;    
            tree.add([r, g, b], label)?;
        }
        Ok(())
    }

    pub fn score(&self, expecting: ColorId, color: (u8, u8, u8)) -> u16 {
        // Convert input color to f32 for querying
        let r = color.0 as f32;
        let g = color.1 as f32;
        let b = color.2 as f32;
        let query = [r, g, b];

        let num = 5;
        let min_required = 3; 
        if let Ok(nearest) = self.tree.nearest(&query, num, &squared_euclidean) {
            // Count the number of neighbors matching the expected color
            let expected_count = nearest
                .into_iter()
                .filter(|(_, label)| **label == expecting)
                .count();

            // Calculate the percentage of matching neighbors
            return IS_MATHING_MIN_THR + (expected_count - min_required + 1) as u16
        }
        0
    }
    

    /// Classify a color (R, G, B) using the nearest neighbor approach
    pub fn classify(&self, color: (u8, u8, u8)) -> ColorId {
        if color == (0, 0, 0) {
            return ColorId::Other;
        }
        let r = color.0 as f32;
        let g = color.1 as f32;
        let b = color.2 as f32;
        let query = [r, g, b];

        // Perform nearest neighbor search
        if let Ok(nearest) = self.tree.nearest(&query, 1, &squared_euclidean) {
            if let Some((nearest, label)) = nearest.into_iter().next() {
                if nearest < 25.0 {
                    //println!("Nearest neighbor: {} ({})", nearest, label);                    
                    return *label                    
                }
            }
        }
        ColorId::Other
    }
}


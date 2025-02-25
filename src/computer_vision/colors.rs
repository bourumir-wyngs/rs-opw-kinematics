use std::fmt;
use crate::color_classifier::{classify_color, estimate_color_score};
use crate::computer_vision::{BASIC_MULTIPLIER, IS_MATHING_MIN_THR, MIN_BRIGHTNESS};

#[derive(Debug, Clone, Eq, PartialEq, Hash, Copy)]
pub enum ColorId {
    Red,
    Green,
    Blue,
    Yellow,
    Magenta,
    Other,
}

// Implement the `Display` trait for `ColorId`
impl fmt::Display for ColorId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Match the enum variant to a human-readable string
        match self {
            ColorId::Red => write!(f, "Red"),
            ColorId::Green => write!(f, "Green"),
            ColorId::Blue => write!(f, "Blue"),
            ColorId::Yellow => write!(f, "Yellow"),
            ColorId::Magenta => write!(f, "Magenta"),
            ColorId::Other => write!(f, "Other"),
        }
    }
}


/// Enum representing primary colors with associated HSV ranges.
#[derive(Debug, Clone)]
pub enum DefinedColor {
    Red { id: ColorId },
    Green { id: ColorId },
    LightBlue { id: ColorId },
    Yellow { id: ColorId },
    Magenta { id: ColorId },
}

impl DefinedColor {
    pub(crate) fn color_score(&self, r: u8, g: u8, b: u8) -> u16 {
        if r == 0 && g == 0 && b == 0 {
            return 0; // parts are black as they are masked
        }
        match self {
            DefinedColor::Red { .. } => {
                estimate_color_score(ColorId::Red,(r, g, b))
            }
            DefinedColor::Green { .. } => {
                estimate_color_score(ColorId::Green, (r, g, b))                
            }
            DefinedColor::LightBlue { .. } => {
                estimate_color_score(ColorId::Blue, (r, g, b))                
            }
            DefinedColor::Yellow { .. } => {
                estimate_color_score(ColorId::Yellow, (r, g, b))                
            }
            DefinedColor::Magenta { .. } => {
                estimate_color_score(ColorId::Magenta, (r, g, b))
            }            
        }
    }
}


impl DefinedColor {
    /// Creates an instance of the Red color with its HSV range.
    pub fn red() -> Self {
        DefinedColor::Red { id: ColorId::Red }
    }

    /// Creates an instance of the Green color with its HSV range.
    pub fn green() -> Self {
        DefinedColor::Green { id: ColorId::Green }
    }

    /// Creates an instance of the Blue color with its HSV range.
    pub fn blue() -> Self {
        DefinedColor::LightBlue { id: ColorId::Blue }
    }
    
    pub fn yellow() -> Self {
        DefinedColor::Yellow { id: ColorId::Yellow }
    }

    pub fn magenta() -> Self {
        DefinedColor::Magenta { id: ColorId::Magenta }
    }    

    /// Retrieve the unique `ColorId` associated with the `DefinedColor`.
    pub fn id(&self) -> ColorId {
        match self {
            DefinedColor::Red { id, .. }
            | DefinedColor::Green { id, .. }
            | DefinedColor::LightBlue { id, .. } 
            | DefinedColor::Yellow { id, .. }
            | DefinedColor::Magenta { id, .. }
            => id.clone(),
        }
    }
}

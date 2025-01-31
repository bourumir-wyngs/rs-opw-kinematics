use crate::computer_vision::{BASIC_MULTIPLIER, MIN_BRIGHTNESS};

/// Enum representing primary colors with associated HSV ranges.
#[derive(Debug, Clone)]
pub enum DefinedColor {
    Red { id: ColorId },
    Green { id: ColorId },
    LightBlue { id: ColorId },
}

impl DefinedColor {

    pub(crate) fn color_score(&self, r: u8, g: u8, b: u8) -> u16 {
        let total = r as usize + g as usize + b as usize;
        if total < MIN_BRIGHTNESS {
            // It is too dark
            return 0;
        }
        match self {
            DefinedColor::Red { .. } => {
                ((BASIC_MULTIPLIER *  r as usize) / total) as u16
            },
            DefinedColor::Green { .. } => {
                ((BASIC_MULTIPLIER *  g as usize) / total) as u16
            },
            DefinedColor::LightBlue { .. } => {                
                ((BASIC_MULTIPLIER *  (77 * b as usize + 56 * g as usize)/100) / total) as u16
            }
        }        
    }    
}


#[derive(Debug, Clone, Eq, PartialEq, Hash, Copy)]
pub enum ColorId {
    Red,
    Green,
    Blue,
}

impl DefinedColor {
    /// Creates an instance of the Red color with its HSV range.
    pub fn red() -> Self {
        DefinedColor::Red {
            id: ColorId::Red,
        }
    }

    /// Creates an instance of the Green color with its HSV range.
    pub fn green() -> Self {
        DefinedColor::Green {
            id: ColorId::Green,
        }
    }

    /// Creates an instance of the Blue color with its HSV range.
    pub fn blue() -> Self {
        DefinedColor::LightBlue {
            id: ColorId::Blue,
        }
    }    

    /// Retrieve the unique `ColorId` associated with the `DefinedColor`.
    pub fn id(&self) -> ColorId {
        match self {
            DefinedColor::Red { id, .. }
            | DefinedColor::Green { id, .. }
            | DefinedColor::LightBlue { id, .. } => id.clone(),
        }
    }
}


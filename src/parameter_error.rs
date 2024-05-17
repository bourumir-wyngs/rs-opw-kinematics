//! Error handling for parameter extractors

use std::io;

/// Unified error to report failures during both YAML and URDF/XACRO parsing.
#[derive(Debug)]
pub enum ParameterError {
    IoError(io::Error),
    ParseError(String),
    MissingField(String),
    WrongAngle(String),
    InvalidLength { expected: usize, found: usize },
    XmlProcessingError(String),
    ParameterPopulationError(String),
    KinematicsConfigurationError(String),
}

impl std::fmt::Display for ParameterError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            ParameterError::IoError(ref err) => 
                write!(f, "IO Error: {}", err),
            ParameterError::ParseError(ref msg) => 
                write!(f, "Parse Error: {}", msg),
            ParameterError::WrongAngle(ref msg) =>
                write!(f, "Wrong angle representation: {}", msg),            
            ParameterError::MissingField(ref field) => 
                write!(f, "Missing Field: {}", field),
            ParameterError::InvalidLength { expected, found } => 
                write!(f, "Invalid Length: expected {}, found {}", expected, found),
            ParameterError::XmlProcessingError(ref err) => 
                write!(f, "XML Processing Error: {}", err),
            ParameterError::ParameterPopulationError(ref err) => 
                write!(f, "Parameter Population Error: {}", err),
            ParameterError::KinematicsConfigurationError(ref err) => 
                write!(f, "Kinematics Configuration Error: {}", err),
        }
    }
}

impl From<io::Error> for ParameterError {
    fn from(err: io::Error) -> Self {
        ParameterError::IoError(err)
    }
}

use std::fs::File;
use std::io::Read;
use std::path::Path;
use serde::Deserialize;
use thiserror::Error;
use crate::parameters::opw_kinematics::Parameters;

/// https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m16ib_support/config/opw_parameters_m16ib20.yaml
#[derive(Error, Debug)]
pub enum ParametersError {
    #[error("Failed to read the robot description file")]
    FileReadError(#[from] std::io::Error),
    #[error("failed to parse YAML in robot description file")]
    YamlParseError(#[from] serde_yaml::Error),
}

impl Parameters {
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, ParametersError> {
        let mut file = File::open(path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;
        let deserialized: YamlParameters = serde_yaml::from_str(&contents)?;

        Ok(Parameters {
            a1: deserialized.opw_kinematics_geometric_parameters.a1,
            a2: deserialized.opw_kinematics_geometric_parameters.a2,
            b: deserialized.opw_kinematics_geometric_parameters.b,
            c1: deserialized.opw_kinematics_geometric_parameters.c1,
            c2: deserialized.opw_kinematics_geometric_parameters.c2,
            c3: deserialized.opw_kinematics_geometric_parameters.c3,
            c4: deserialized.opw_kinematics_geometric_parameters.c4,
            offsets: deserialized.opw_kinematics_joint_offsets,
            sign_corrections: deserialized.opw_kinematics_joint_sign_corrections,
        })
    }
}

/// Helper struct for deserialization
#[derive(Debug, Deserialize)]
struct YamlParameters {
    opw_kinematics_geometric_parameters: GeometricParameters,
    opw_kinematics_joint_offsets: [f64; 6],
    opw_kinematics_joint_sign_corrections: [i8; 6],
}

#[derive(Debug, Deserialize)]
struct GeometricParameters {
    a1: f64,
    a2: f64,
    b: f64,
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
}

// Utility function for degree to radian conversion, if needed
fn deg(degree: f64) -> f64 {
    degree * std::f64::consts::PI / 180.0
}

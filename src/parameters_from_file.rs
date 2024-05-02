use std::fs::File;
use std::io::Read;
use std::path::Path;
use regex::Regex;
use serde::Deserialize;
use thiserror::Error;
use crate::parameters::opw_kinematics::Parameters;

/// https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m16ib_support/config/opw_parameters_m16ib20.yaml
/// Defines the parameters loading error
#[derive(Error, Debug)]
pub enum ParametersError {
    #[error("failed to read robot definition file")]
    FileReadError(#[from] std::io::Error),
    #[error("failed to parse YAML in the robot definition file")]
    YamlParseError(#[from] serde_yaml::Error),
    #[error("failed to process YAML content")]
    YamlProcessError(#[from] regex::Error),
}

impl Parameters {
    
    ///
    /// Read the robot configuration from YAML file. YAML file like this is supported:
    /// 
    /// # FANUC m16ib20
    /// opw_kinematics_geometric_parameters:
    ///   a1: 0.15
    ///   a2: -0.10
    ///   b: 0.0
    ///   c1: 0.525
    ///   c2: 0.77
    ///   c3: 0.74
    ///   c4: 0.10
    /// opw_kinematics_joint_offsets: [0.0, 0.0, deg(-90.0), 0.0, 0.0, deg(180.0)]
    /// opw_kinematics_joint_sign_corrections: [1, 1, -1, -1, -1, -1]
    ///
    /// ROS-Industrial provides many such files for FANUC robots on GitHub
    /// (ros-industrial/fanuc, see fanuc_m10ia_support/config/opw_parameters_m10ia.yaml
    /// YAML extension to parse the deg(angle) function is supported.  
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, ParametersError> {
        let mut file = File::open(path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;

        let processed_contents = preprocess_yaml_contents(&contents)?;

        let deserialized: YamlParameters = serde_yaml::from_str(&processed_contents)?;

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

fn preprocess_yaml_contents(contents: &str) -> Result<String, regex::Error> {
    let re = Regex::new(r"deg\(([^)]+)\)")?;
    let processed_contents = re.replace_all(contents, |caps: &regex::Captures| {
        format!("{}", deg(caps[1].parse::<f64>().unwrap()))
    }).to_string();

    Ok(processed_contents)
}

fn deg(degree: f64) -> f64 {
    degree * std::f64::consts::PI / 180.0
}

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

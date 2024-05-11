use std::{
    fs::File,
    io::Read,
    path::Path,
};
use yaml_rust2::{Yaml, YamlLoader};
use crate::parameter_error::ParameterError;
use crate::parameters::opw_kinematics::Parameters;

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
    /// (ros-industrial/fanuc, see fanuc_m10ia_support/config/opw_parameters_m10ia.yaml)
    /// YAML extension to parse the deg(angle) function is supported.
    ///  
    /// See https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m16ib_support/config/opw_parameters_m16ib20.yaml
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, ParameterError> {
        let mut file = File::open(path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;

        let docs = YamlLoader::load_from_str(&contents).map_err(
            |e| ParameterError::ParseError(e.to_string()))?;
        let doc = &docs[0];

        let params = &doc["opw_kinematics_geometric_parameters"];
        let offsets_yaml = &doc["opw_kinematics_joint_offsets"];
        let sign_corrections_yaml = &doc["opw_kinematics_joint_sign_corrections"];

        let offsets: [f64; 6] = offsets_yaml.as_vec()
            .ok_or_else(|| ParameterError::MissingField("offsets array".into()))?
            .iter()
            .map(|item| match item {
                Yaml::String(s) => Self::parse_degrees(s),
                Yaml::Real(s) => s.parse::<f64>()
                    .map_err(|_| ParameterError::ParseError("Failed to parse angle".into())),
                _ => Err(ParameterError::ParseError(
                    "Offset entry is not a number or deg() function".into())),
            })
            .collect::<Result<Vec<_>, _>>()?
            .try_into()
            .map_err(|_| ParameterError::InvalidLength {
                expected: 6,
                found: offsets_yaml.as_vec()
                    .unwrap().len(),
            })?;

        let sign_corrections: [i8; 6] = sign_corrections_yaml.as_vec()
            .ok_or_else(|| ParameterError::MissingField("sign corrections array".into()))?
            .iter()
            .map(|item| item.as_i64().ok_or(
                ParameterError::ParseError("Sign correction not an integer".into()))
                .map(|x| x as i8))
            .collect::<Result<Vec<_>, _>>()?
            .try_into()
            .map_err(|_| ParameterError::InvalidLength {
                expected: 6,
                found: sign_corrections_yaml.as_vec().unwrap().len(),
            })?;

        Ok(Parameters {
            a1: params["a1"].as_f64().ok_or_else(|| ParameterError::MissingField("a1".into()))?,
            a2: params["a2"].as_f64().ok_or_else(|| ParameterError::MissingField("a2".into()))?,
            b: params["b"].as_f64().ok_or_else(|| ParameterError::MissingField("b".into()))?,
            c1: params["c1"].as_f64().ok_or_else(|| ParameterError::MissingField("c1".into()))?,
            c2: params["c2"].as_f64().ok_or_else(|| ParameterError::MissingField("c2".into()))?,
            c3: params["c3"].as_f64().ok_or_else(|| ParameterError::MissingField("c3".into()))?,
            c4: params["c4"].as_f64().ok_or_else(|| ParameterError::MissingField("c4".into()))?,
            offsets,
            sign_corrections,
        })
    }

    /// Parses angles from strings in degrees format or plain floats.
    fn parse_degrees(s: &str) -> Result<f64, ParameterError> {
        if let Some(angle) = s.strip_prefix("deg(")
            .and_then(|s| s.strip_suffix(")")) {
            angle.trim().parse::<f64>()
                .map_err(
                    |_| ParameterError::ParseError(format!("Failed to parse degrees from {}", s)))
                .map(|deg| deg.to_radians())
        } else {
            s.parse::<f64>().map_err(
                |_| ParameterError::ParseError(format!("Failed to parse degrees from {}", s)))
        }
    }
    
}

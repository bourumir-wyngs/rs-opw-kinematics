//! Supports extracting OPW parameters from YAML file (optional)

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
    /// ```yaml
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
    /// dof: 6
    /// ```
    /// offsets, sign corrections and DOF are optional
    ///
    /// ROS-Industrial provides many such files for FANUC robots on GitHub
    /// (ros-industrial/fanuc, see fanuc_m10ia_support/config/opw_parameters_m10ia.yaml)
    /// YAML extension to parse the deg(angle) function is supported.
    ///  
    /// See [fanuc_m16ib_support in ROS Industrial](https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m16ib_support/config/opw_parameters_m16ib20.yaml).
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, ParameterError> {
        let mut file = File::open(path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;

        let docs = YamlLoader::load_from_str(&contents).map_err(
            |e| ParameterError::ParseError(e.to_string()))?;

        let doc = &docs[0];
        let params = &doc["opw_kinematics_geometric_parameters"];
        let dof = params["dof"].as_i64().unwrap_or(6) as i8;
        let mut sign_corrections = Self::read_sign_corrections(&doc["opw_kinematics_joint_sign_corrections"])?;
        if dof == 5 {
            // Block J6 at 0 by default for 5DOF robot.
            sign_corrections[5] = 0;
        }

        Ok(Parameters {
            a1: params["a1"].as_f64().ok_or_else(|| ParameterError::MissingField("a1".into()))?,
            a2: params["a2"].as_f64().ok_or_else(|| ParameterError::MissingField("a2".into()))?,
            b: params["b"].as_f64().ok_or_else(|| ParameterError::MissingField("b".into()))?,
            c1: params["c1"].as_f64().ok_or_else(|| ParameterError::MissingField("c1".into()))?,
            c2: params["c2"].as_f64().ok_or_else(|| ParameterError::MissingField("c2".into()))?,
            c3: params["c3"].as_f64().ok_or_else(|| ParameterError::MissingField("c3".into()))?,
            c4: params["c4"].as_f64().ok_or_else(|| ParameterError::MissingField("c4".into()))?,
            dof: dof,
            offsets: Self::read_offsets(&doc["opw_kinematics_joint_offsets"])?,
            sign_corrections: sign_corrections,
        })
    }


    fn read_sign_corrections(doc: &Yaml) -> Result<[i8; 6], ParameterError> {
        // Store the temporary vector in a variable for longer lifetime
        let default_sign_corrections = vec![Yaml::Integer(1); 6];

        // Check if the sign corrections field exists, if not default to all 0
        let sign_corrections_yaml = doc.as_vec().unwrap_or(&default_sign_corrections);

        let mut sign_corrections: Vec<i8> = sign_corrections_yaml
            .iter()
            .map(|item| item.as_i64().unwrap_or(0) as i8)  // Default missing or invalid entries to 0
            .collect();

        // Ensure length is either 5 or 6, and pad with 0 if necessary
        if sign_corrections.len() == 5 {
            sign_corrections.push(0); // Add 0 as the 6th element
        }

        if sign_corrections.len() != 6 {
            return Err(ParameterError::InvalidLength {
                expected: 6,
                found: sign_corrections.len(),
            });
        }

        let sign_corrections: [i8; 6] = sign_corrections.try_into().unwrap(); // Safe now, we ensured it's of length 6
        Ok(sign_corrections)
    }


    fn read_offsets(offsets_yaml: &Yaml) -> Result<[f64; 6], ParameterError> {
        // Check if the offsets field exists, if not default to all 0
        let default_offsets = &vec![Yaml::Integer(0); 6];
        let offsets_yaml = offsets_yaml.as_vec().unwrap_or(&default_offsets);
        let mut offsets: Vec<f64> = offsets_yaml
            .iter()
            .map(|item| match item {
                Yaml::String(s) => Self::parse_degrees(s),
                Yaml::Real(s) => s.parse::<f64>()
                    .map_err(|_| ParameterError::ParseError("Failed to parse angle".into())),
                Yaml::Integer(s) => Ok(*s as f64),
                _ => Ok(0.0),  // Default any invalid entry to 0
            })
            .collect::<Result<Vec<_>, _>>()?;

        // Ensure length is either 5 or 6, and pad with 0 if necessary
        if offsets.len() == 5 {
            offsets.push(0.0); // Add 0 as the 6th element
        }

        if offsets.len() != 6 {
            return Err(ParameterError::InvalidLength {
                expected: 6,
                found: offsets.len(),
            });
        }

        let offsets: [f64; 6] = offsets.try_into().unwrap(); // Safe now, we ensured it's of length 6
        Ok(offsets)
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

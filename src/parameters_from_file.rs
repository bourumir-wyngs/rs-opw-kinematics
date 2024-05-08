use std::fs::File;
use std::io::Read;
use std::path::Path;

use yaml_rust2::{Yaml, YamlLoader};
use crate::parameters::opw_kinematics::Parameters;

/// See https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m16ib_support/config/opw_parameters_m16ib20.yaml

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
    pub fn from_yaml_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let mut file = File::open(path).map_err(|e| e.to_string())?;
        let mut contents = String::new();
        file.read_to_string(&mut contents).map_err(|e| e.to_string())?;
        let docs = YamlLoader::load_from_str(&contents).map_err(|e| e.to_string())?;
        let doc = &docs[0];

        /// We support the deg(angle) function, even if it is not a standard YAML. It is
        /// found in the files we need to parse, so what?
        fn parse_degrees(s: &str) -> Result<f64, String> {
            if s.starts_with("deg(") && s.ends_with(")") {
                let len = s.len();
                s[4..len - 1].trim().parse::<f64>()
                    .map_err(|_| format!("Failed to parse degrees from {}", s))
                    .map(|deg| deg.to_radians())
            } else {
                s.parse::<f64>().map_err(|_| format!("Failed to parse deg(x) argument from {}", s))
            }
        }

        let geometric_params = &doc["opw_kinematics_geometric_parameters"];
        let offsets_yaml = &doc["opw_kinematics_joint_offsets"];
        let sign_corrections_yaml = &doc["opw_kinematics_joint_sign_corrections"];

        let offsets: [f64; 6] = offsets_yaml.as_vec().ok_or("Missing offsets array")?
            .iter()
            .map(|item| match item {
                Yaml::String(s) if s.starts_with("deg(") => parse_degrees(s),
                Yaml::Real(s) | Yaml::String(s) => s.parse::<f64>().map_err(|_| "Failed to parse angle".to_string()),
                _ => Err("Offset entry is not a number or deg() function".to_string()),
            })
            .collect::<Result<Vec<_>, _>>()?
            .try_into()
            .map_err(|_| "Incorrect number of offsets, must be 6".to_string())?;

        let sign_corrections: [i8; 6] = sign_corrections_yaml.as_vec().ok_or("Missing sign corrections array")?
            .iter()
            .map(|item| item.as_i64().ok_or("Sign correction not an integer".to_string()).map(|x| x as i8))
            .collect::<Result<Vec<_>, _>>()?
            .try_into()
            .map_err(|_| "Incorrect number of sign corrections, must be 6".to_string())?;

        Ok(Parameters {
            a1: geometric_params["a1"].as_f64().ok_or("Missing field: a1")?,
            a2: geometric_params["a2"].as_f64().ok_or("Missing field: a2")?,
            b: geometric_params["b"].as_f64().ok_or("Missing field: b")?,
            c1: geometric_params["c1"].as_f64().ok_or("Missing field: c1")?,
            c2: geometric_params["c2"].as_f64().ok_or("Missing field: c2")?,
            c3: geometric_params["c3"].as_f64().ok_or("Missing field: c3")?,
            c4: geometric_params["c4"].as_f64().ok_or("Missing field: c4")?,
            offsets,
            sign_corrections,
        })
    }
}


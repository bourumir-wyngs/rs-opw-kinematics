use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::Read;
use serde_yaml;

#[derive(Debug, Serialize, Deserialize)]
struct Pose {
    translation: [f64; 3],
    quaternion: [f64; 4], // Assuming [x, y, z, w] ordering here
}

#[derive(Debug, Serialize, Deserialize)]
struct Case {
    id: i32,
    parameters: String,
    joints: [f64; 6],
    solutions: Vec<[f64; 6]>,
    pose: Pose,
}

impl Case {
    // Method to return joints in radians
    pub fn joints_in_radians(&self) -> [f64; 6] {
        let mut joints_in_radians = [0.0; 6];
        for (i, &joint) in self.joints.iter().enumerate() {
            joints_in_radians[i] = joint.to_radians();
        }
        joints_in_radians
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct Cases {
    cases: Vec<Case>,
}

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

impl Pose {
    pub fn to_isometry(&self) -> Isometry3<f64> {
        let translation = Translation3::new(self.translation[0], self.translation[1], self.translation[2]);

        // Adjusting quaternion creation to match [x, y, z, w] ordering
        let quaternion = UnitQuaternion::from_quaternion(Quaternion::new(
            self.quaternion[3], // w
            self.quaternion[0], // x
            self.quaternion[1], // y
            self.quaternion[2], // z
        ));

        Isometry3::from_parts(translation, quaternion)
    }

    pub fn from_isometry(isometry: &Isometry3<f64>) -> Self {
        let translation = isometry.translation.vector;

        // Extract the quaternion from isometry (rotation part)
        let quaternion = isometry.rotation.quaternion();

        Pose {
            translation: [translation.x, translation.y, translation.z],
            quaternion: [quaternion.i, quaternion.j, quaternion.k, quaternion.w], // [x, y, z, w] ordering
        }
    }
}

fn load_yaml(filename: &str) -> Result<Cases, serde_yaml::Error> {
    let mut file = File::open(filename).expect("Unable to open file");
    let mut contents = String::new();
    file.read_to_string(&mut contents).expect("Unable to read the file");
    serde_yaml::from_str(&contents)
}

fn are_isometries_approx_equal(a: &Isometry3<f64>, b: &Isometry3<f64>, tolerance: f64) -> bool {
    let translation_diff = a.translation.vector - b.translation.vector;
    if translation_diff.norm() > tolerance {
        return false;
    }

    // Check if the rotation components are approximately equal
    // This part is a bit more complex due to quaternion properties.
    // One way is to calculate the angle between the two quaternions and see if it's within the tolerance.
    // This involves converting the unit quaternion difference into an angle.
    let rotation_diff = a.rotation.inverse() * b.rotation;
    let angle = rotation_diff.angle();

    angle.abs() <= tolerance
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use crate::kinematic_traits::kinematics_traits::Kinematics;
    use crate::parameters::opw_kinematics::Parameters;
    use crate::kinematics_impl::OPWKinematics;
    use super::*;

    #[test]
    fn test_load_yaml() {
        let filename = "src/tests/cases.yaml";
        let result = load_yaml(filename);

        if let Err(e) = &result {
            println!("Error loading or parsing YAML file: {}", e);
        }

        assert!(result.is_ok(), "Failed to load or parse the YAML file");

        let cases_struct = result.expect("Expected a valid Cases struct after parsing");

        // Example assertion: the list of cases should not be empty.
        assert!(!cases_struct.cases.is_empty(), "No cases were loaded from the YAML file");
    }

    #[test]
    fn test_forward_ik() {
        let filename = "src/tests/cases.yaml";
        let result = load_yaml(filename);
        assert!(result.is_ok(), "Failed to load or parse the YAML file");
        let cases = result.expect("Expected a valid Cases struct after parsing");

        // Create map to get actual parameters that are not in the yaml file (maybe should be?)
        let mut all_parameters: HashMap<String, Parameters> = HashMap::new();
        all_parameters.insert(String::from("Irb2400_10"), Parameters::irb2400_10());
        all_parameters.insert(String::from("KukaKR6_R700_sixx"), Parameters::kuka_kr6_r700_sixx());

        println!("{} test cases", cases.cases.len());

        for case in cases.cases.iter() {
            let parameters = all_parameters.get(&case.parameters).unwrap_or_else(|| {
                panic!("Parameters for the robot [{}] are unknown", &case.parameters)
            });            let kinematics = OPWKinematics::new(parameters.clone());

            // Try forward on the initial data set first.
            let ik = kinematics.forward(&case.joints_in_radians());
            let pose = Pose::from_isometry(&ik);

            if !are_isometries_approx_equal(&ik, &case.pose.to_isometry(), 0.00001) {
                println!("Seems not equal");
                println!("joints: {:?} ", &case.joints);
                println!("case: {:?} ", &pose);
                println!("IK  : {:?} ", &case.pose);
                println!();

                panic!("Forward kinematics of the primary pose seems not equal");
            }
        }
    }
}

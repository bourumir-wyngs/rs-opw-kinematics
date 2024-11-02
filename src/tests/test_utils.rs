use std::collections::HashMap;
use std::f64::consts::PI;
use std::fs::File;
use std::io::Read;

#[derive(Debug)]
pub(crate) struct Pose {
    translation: [f64; 3],
    quaternion: [f64; 4], // Assuming [x, y, z, w] ordering here
}

#[derive(Debug)]
pub struct Case {
    pub id: i32,
    pub(crate) parameters: String,
    pub(crate) joints: [f64; 6],
    pub _solutions: Vec<[f64; 6]>,
    pub(crate) pose: Pose,
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

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use yaml_rust2::{Yaml, YamlLoader};
use crate::kinematic_traits::{Joints, Solutions};
use crate::parameters::opw_kinematics::Parameters;

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

pub(crate) fn load_yaml(file_path: &str) -> Result<Vec<Case>, String> {
    let mut file = File::open(file_path).map_err(|e| e.to_string())?;
    let mut contents = String::new();
    file.read_to_string(&mut contents).map_err(|e| e.to_string())?;

    let docs = YamlLoader::load_from_str(&contents).map_err(|e| e.to_string())?;
    let cases_yaml = &docs[0]["cases"];

    let mut cases: Vec<Case> = Vec::new();
    if let Some(cases_vec) = cases_yaml.as_vec() {
        for case_yaml in cases_vec {
            let id = case_yaml["id"].as_i64().ok_or("Missing id")? as i32;
            let parameters = case_yaml["parameters"].as_str().ok_or("Missing parameters")?.to_string();
            let joints = parse_array::<f64, 6>(&case_yaml["joints"])?;
            let solutions = parse_solutions(&case_yaml["solutions"])?;
            let pose = parse_pose(&case_yaml["pose"])?;

            cases.push(Case {
                id,
                parameters,
                joints,
                _solutions: solutions,
                pose,
            });
        }
    } else {
        return Err("Expected 'cases' to be a sequence".to_string());
    }

    Ok(cases)
}

fn parse_array<T: Copy + std::str::FromStr, const N: usize>(yaml: &Yaml) -> Result<[T; N], String>
where
    <T as std::str::FromStr>::Err: std::fmt::Display, // Ensure the error type of T's FromStr can be displayed
{
    let vec = yaml.as_vec().ok_or("Expected an array in YAML")?;

    if vec.is_empty() {
        return Err("Array in YAML is empty, cannot initialize".to_string());
    }

    // Initialize the array using the first element if it can be parsed, otherwise return an error
    let first_value = vec.get(0)
        .ok_or_else(|| "Array is non-empty but no first item found".to_string())
        .and_then(|item| {
            match item {
                Yaml::Real(s) => s.parse::<T>()
                    .map_err(|e| format!("Failed to parse first item as real: {}, found: '{}'", e, s)),
                Yaml::Integer(i) => i.to_string().parse::<T>()
                    .map_err(|e| format!("Failed to parse first item as integer: {}, found: '{}'", e, i)),
                _ => Err(format!("First item is not a real or integer value, found: {:?}", item))
            }
        })?;

    let mut array: [T; N] = [first_value; N];  // Use the first parsed value to initialize the array

    // Parse each element in the vector and fill the array
    for (i, item) in vec.iter().enumerate() {
        array[i] = match item {
            Yaml::Real(s) => s.parse::<T>()
                .map_err(|e| format!("Error parsing real at index {}: {}, found: '{}'", i, e, s))?,
            Yaml::Integer(i) => i.to_string()
                .parse::<T>().map_err(|e| format!("Error parsing integer at index {}: {}, found: '{}'", i, e, i))?,
            _ => return Err(format!("Expected a real or integer value at index {}, found: {:?}", i, item))
        };
    }

    Ok(array)
}

fn parse_solutions(yaml: &Yaml) -> Result<Vec<[f64; 6]>, String> {
    let mut solutions = Vec::new();
    for solution_yaml in yaml.as_vec().ok_or("Expected solutions array")? {
        solutions.push(parse_array::<f64, 6>(solution_yaml)?);
    }
    Ok(solutions)
}

fn parse_pose(yaml: &Yaml) -> Result<Pose, String> {
    let translation = parse_array::<f64, 3>(&yaml["translation"])?;
    let quaternion = parse_array::<f64, 4>(&yaml["quaternion"])?;

    Ok(Pose {
        translation,
        quaternion,
    })
}

pub fn are_isometries_approx_equal(a: &Isometry3<f64>, b: &Isometry3<f64>, tolerance: f64) -> bool {
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

pub(crate) fn create_parameter_map() -> HashMap<String, Parameters> {
    // Create map to get actual parameters that are not in the yaml file (maybe should be?)
    let all_parameters: HashMap<String, Parameters> = vec![
        (String::from("Irb2400_10"), Parameters::irb2400_10()),
        (String::from("KukaKR6_R700_sixx"), Parameters::kuka_kr6_r700_sixx()),
        (String::from("Fanuc_r2000ib_200r"), Parameters::fanuc_r2000ib_200r()),
        (String::from("Staubli_tx40"), Parameters::staubli_tx40()),
        (String::from("Irb2600_12_165"), Parameters::irb2600_12_165()),
        (String::from("Irb4600_60_205"), Parameters::irb4600_60_205()),
        (String::from("Staubli_tx2_140"), Parameters::staubli_tx2_140()),
        (String::from("Staubli_tx2_160"), Parameters::staubli_tx2_160()),
        (String::from("Staubli_tx2_160l"), Parameters::staubli_tx2_160l()),
    ]
        .into_iter()
        .collect();
    all_parameters
}

/// Check if 'expected' exists in the given vector of solutions. This function is also
/// used by other tests.
pub fn found_joints_approx_equal(solutions: &Solutions, expected: &Joints, tolerance: f64) -> Option<i32> {
    for sol_idx in 0..solutions.len() {
        // println!("Checking solution at index {}", sol_idx);

        let mut solution_matches = true;
        for joint_idx in 0..6 {
            let computed = solutions[sol_idx][joint_idx];
            let asserted = expected[joint_idx];

            let diff = (computed - asserted).abs();
            //println!("Column value: {}, Expected value: {}, Difference: {}",
            //         computed, asserted, diff);

            if diff >= tolerance && (diff - 2. * PI).abs() > tolerance {
                // For angles, 360 degree difference means the same angle.
                solution_matches = false;
                break;
            }
        }

        if solution_matches {
            return Some(sol_idx as i32); // Return the index of the matching solution
        }
    }

    println!("No matching solution found");
    return None; // Explicitly indicate that no matching column was found
}
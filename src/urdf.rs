extern crate sxd_document;

use std::collections::HashMap;
use sxd_document::{parser, dom, QName};
use std::error::Error;
use std::fs::read_to_string;
use std::path::Path;
use regex::Regex;
use crate::constraints::{BY_PREV, Constraints};
use crate::kinematic_traits::{Joints, JOINTS_AT_ZERO};
use crate::kinematics_impl::OPWKinematics;
use crate::parameter_error::ParameterError;
use crate::parameters::opw_kinematics::Parameters;

/// Simplified reading from URDF file. This function assumes sorting of results by closest to
/// previous (BY_PREV) and no joint offsets (zero offsets). URDF file is expected in the input
/// but XACRO file may also work. Robot joints must be named joint1 to joint6 in the file
/// (as macro prefix, underscore and non-word chars is dropped, it can also be something like
/// `${prefix}JOINT_1`). It may be more than one robot described in URDF file but they must all
/// be identical.
///
/// # Parameters
/// - `path`: the location of URDF or XACRO file to load from.
///
/// # Returns
/// - Returns an instance of `OPWKinematics`, which contains the kinematic parameters
///   extracted from the specified URDF file, including constraints as defined there.
///
/// # Example
/// ```
/// let kinematics = rs_opw_kinematics::urdf::from_urdf_file("/path/to/robot.urdf");
/// println!("{:?}", kinematics);
/// ```
///
/// # Errors
/// - The function might panic if the file cannot be found, is not accessible, or is incorrectly 
///   formatted. Users should ensure the file path is correct and the file is properly formatted as 
///   URDF or XACRO file.
pub fn from_urdf_file<P: AsRef<Path>>(path: P) -> OPWKinematics {
    let xml_content = read_to_string(path).expect("Failed to read xacro/urdf file");

    let joint_data = process_joints(&xml_content)
        .expect("Failed to process XML joints");

    let opw_parameters = populate_opw_parameters(joint_data)
        .expect("Failed to read OpwParameters");

    opw_parameters.to_robot(BY_PREV, &JOINTS_AT_ZERO)
}

/// Parses URDF XML content to construct OPW kinematics parameters for a robot.
/// This function provides detailed error handling through the `ParameterError` type,
/// and returns intermediate type from where both Parameters and Constraints can be taken
/// and inspected or modified if so required.
///
/// # Parameters
/// - `xml_content`: A `String` containing the XML data of the URDF file.
///
/// # Returns
/// - Returns a `Result<URDFParameters, ParameterError>`. On success, it contains the OPW kinematics
///   configuration for the robot. On failure, it returns a detailed error.
///   Ust to_robot method to convert OpwParameters directly to the robot instance.
///
/// # Example showing full control over how the inverse kinematics solver is constructed:
/// ```
/// use std::f64::consts::PI;
/// use rs_opw_kinematics::constraints::BY_PREV;
/// use rs_opw_kinematics::kinematic_traits::{Joints, JOINTS_AT_ZERO, Kinematics};
/// use rs_opw_kinematics::kinematics_impl::OPWKinematics;
/// use rs_opw_kinematics::urdf::from_urdf;
/// // Exactly this string would fail. Working URDF fragment would be too long for this example. 
/// let xml_data = String::from("<robot><joint ...></joint></robot>"); 
/// let offsets = [ 0., PI, 0., 0.,0.,0.];
/// let opw_params = from_urdf(xml_data);
/// match opw_params {
///     Ok(opw_params) => {
///         println!("Building the IK solver {:?}", opw_params);
///         let parameters = opw_params.parameters(&JOINTS_AT_ZERO); // Zero joint offsets
///         let constraints =opw_params.constraints(BY_PREV); 
///         let robot = OPWKinematics::new_with_constraints(parameters, constraints);
///         /// let joints = robot.inverse( ... )    
///
///     }
///     Err(e) => println!("Error processing URDF: {}", e),
/// }
/// ```
pub fn from_urdf(xml_content: String) -> Result<URDFParameters, ParameterError> {
    let joint_data = process_joints(&xml_content)
        .map_err(|e|
            ParameterError::XmlProcessingError(format!("Failed to process XML joints: {}", e)))?;

    let opw_parameters = populate_opw_parameters(joint_data)
        .map_err(|e|
            ParameterError::ParameterPopulationError(format!("Failed to interpret robot model: {}", e)))?;

    Ok(opw_parameters)
}


#[derive(Debug, Default, PartialEq)]
struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
}

impl Vector3 {
    pub fn non_zero(&self) -> Result<f64, String> {
        let mut non_zero_values = vec![];

        if self.x != 0.0 {
            non_zero_values.push(self.x);
        }
        if self.y != 0.0 {
            non_zero_values.push(self.y);
        }
        if self.z != 0.0 {
            non_zero_values.push(self.z);
        }

        match non_zero_values.len() {
            0 => Ok(0.0),
            1 => Ok(non_zero_values[0]),
            _ => Err(format!("More than one non-zero value in URDF offset {:?}", self).to_string()),
        }
    }
}

#[derive(Debug, PartialEq)]
struct JointData {
    name: String,
    vector: Vector3,
    sign_correction: i32,
    from: f64,
    to: f64,
}

fn process_joints(xml: &str) -> Result<HashMap<String, JointData>, Box<dyn Error>> {
    let package = parser::parse(xml)?;
    let document = package.as_document();

    // Access the root element
    let root_element = document.root().children().into_iter()
        .find_map(|e| e.element())
        .ok_or("No root element found")?;

    // Collect all joint data
    let mut joints = Vec::new();
    collect_joints(root_element, &mut joints)?;

    convert_to_map(joints)
}

/// Simplify joint name: remove macro construct in the prefix, 
/// underscores and non-word characters, set lowercase.
fn preprocess_joint_name(joint_name: &str) -> String {
    // Create a regex to find the ${prefix} pattern
    let re_prefix = Regex::new(r"\$\{[^}]+\}").unwrap();
    // Replace the pattern with an empty string, effectively removing it
    let processed_name = re_prefix.replace_all(joint_name, "");

    // Create a regex to remove all non-alphanumeric characters, including underscores
    let re_non_alphanumeric = Regex::new(r"[^\w]|_").unwrap();
    let clean_name = re_non_alphanumeric.replace_all(&processed_name, "");

    clean_name.to_lowercase()
}

// Recursive function to collect joint data
fn collect_joints(element: dom::Element, joints: &mut Vec<JointData>) -> Result<(), Box<dyn Error>> {
    let origin_tag = QName::new("origin");
    let joint_tag = QName::new("joint");
    let axis_tag = QName::new("axis");
    let limit_tag = QName::new("limit");

    for child in element.children().into_iter().filter_map(|e| e.element()) {
        if child.name() == joint_tag {
            let name = preprocess_joint_name(&child.attribute("name")
                .map(|attr| attr.value().to_string())
                .unwrap_or_else(|| "Unnamed".to_string()));
            let axis_element = child.children().into_iter()
                .find_map(|e| e.element().filter(|el| el.name() == axis_tag));
            let origin_element = child.children().into_iter()
                .find_map(|e| e.element().filter(|el| el.name() == origin_tag));
            let limit_element = child.children().into_iter()
                .find_map(|e| e.element().filter(|el| el.name() == limit_tag));

            let mut joint_data = JointData {
                name,
                vector: origin_element.map_or_else(|| Ok(Vector3::default()), get_xyz_from_origin)?,
                sign_correction: axis_element.map_or(Ok(1), get_axis_sign)?,
                from: 0.,
                to: 0., // 0 to 0 in our notation is 'full circle'
            };

            if let Some((from, to)) = limit_element.map(get_limits).transpose()? {
                joint_data.from = from;
                joint_data.to = to;
            }

            joints.push(joint_data);
        }

        collect_joints(child, joints)?;
    }

    Ok(())
}


fn get_xyz_from_origin(element: dom::Element) -> Result<Vector3, Box<dyn Error>> {
    let xyz_attr = element.attribute("xyz").ok_or("xyz attribute not found")?;
    let coords: Vec<f64> = xyz_attr.value().split_whitespace()
        .map(str::parse)
        .collect::<Result<_, _>>()?;

    if coords.len() != 3 {
        return Err("XYZ attribute does not contain exactly three values".into());
    }

    Ok(Vector3 {
        x: coords[0],
        y: coords[1],
        z: coords[2],
    })
}

fn get_axis_sign(axis_element: dom::Element) -> Result<i32, Box<dyn Error>> {
    // Fetch the 'xyz' attribute, assuming the element is correctly passed
    let axis_attr = axis_element.attribute("xyz").ok_or_else(|| {
        format!("'xyz' attribute not found in element supposed to represent the axis")
    })?;

    // Parse the xyz attribute to determine the sign corrections
    let axis_values: Vec<f64> = axis_attr.value().split_whitespace()
        .map(str::parse)
        .collect::<Result<_, _>>()?;

    // Filter and count non-zero values, ensuring exactly one non-zero which must be -1 or 1
    let non_zero_values: Vec<i32> = axis_values.iter()
        .filter(|&&v| v != 0.0)
        .map(|&v| if v < 0.0 { -1 } else { 1 })
        .collect();

    // Check that exactly one non-zero value exists and it is either -1 or 1
    if non_zero_values.len() == 1 && (non_zero_values[0] == -1 || non_zero_values[0] == 1) {
        Ok(non_zero_values[0])
    } else {
        Err("Invalid axis direction, must have exactly one non-zero value \
        that is either -1 or 1".into())
    }
}


fn get_limits(element: dom::Element) -> Result<(f64, f64), Box<dyn Error>> {
    let lower_attr = element.attribute("lower")
        .ok_or("lower limit not found")?.value().parse::<f64>()?;
    let upper_attr = element.attribute("upper")
        .ok_or("upper limit not found")?.value().parse::<f64>()?;

    Ok((lower_attr, upper_attr))
}

fn convert_to_map(joints: Vec<JointData>) -> Result<HashMap<String, JointData>, Box<dyn Error>> {
    let mut map: HashMap<String, JointData> = HashMap::new();

    for joint in joints {
        if let Some(existing) = map.get(&joint.name) {
            // Check if the existing entry is different from the new one
            if existing != &joint {
                return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidData, format!("Duplicate joint name with different data found: {}", joint.name))));
            }
        } else {
            map.insert(joint.name.clone(), joint);
        }
    }

    Ok(map)
}

/// OPW parameters as extrancted from URDF file, including constraints 
/// (joint offsets are not directly defined in URDF). This structure
/// can provide robot parameters, constraints and sign corrections,
/// or alterntively can be converted to the robot directly. 
#[derive(Default, Debug, Clone, Copy)]
pub struct URDFParameters {
    pub a1: f64,
    pub a2: f64,
    pub b: f64,
    pub c1: f64,
    pub c2: f64,
    pub c3: f64,
    pub c4: f64,
    pub sign_corrections: [i8; 6],
    pub from: Joints, // Array to store the lower limits
    pub to: Joints,   // Array to store the upper limits
}

impl URDFParameters {
    pub fn to_robot(self, sorting_weight: f64, offsets: &Joints) -> OPWKinematics {
        OPWKinematics::new_with_constraints(
            Parameters {
                a1: self.a1,
                a2: self.a2,
                b: self.b,
                c1: self.c1,
                c2: self.c2,
                c3: self.c3,
                c4: self.c4,
                sign_corrections: self.sign_corrections,
                offsets: *offsets,
            },
            Constraints::new(
                self.from,
                self.to,
                sorting_weight,
            ),
        )
    }

    /// Return extracted constraints.
    pub fn constraints(self, sorting_weight: f64) -> Constraints {
        Constraints::new(
            self.from,
            self.to,
            sorting_weight,
        )
    }

    /// Return extracted parameters
    pub fn parameters(self, offsets: &Joints) -> Parameters {
        Parameters {
            a1: self.a1,
            a2: self.a2,
            b: self.b,
            c1: self.c1,
            c2: self.c2,
            c3: self.c3,
            c4: self.c4,
            sign_corrections: self.sign_corrections,
            offsets: *offsets,
        }
    }
}

fn populate_opw_parameters(joint_map: HashMap<String, JointData>) -> Result<URDFParameters, String> {
    let mut opw_parameters = URDFParameters::default();
    
    opw_parameters.b = 0.0; // We only support robots with b = 0 so far.
    
    for (name, joint) in joint_map {
        match name.as_str() {
            "joint1" => {
                opw_parameters.c1 = joint.vector.non_zero()?;
                opw_parameters.sign_corrections[0] = joint.sign_correction as i8;
                opw_parameters.from[0] = joint.from;
                opw_parameters.to[0] = joint.to;
            }
            "joint2" => {
                opw_parameters.a1 = joint.vector.non_zero()?;
                opw_parameters.sign_corrections[1] = joint.sign_correction as i8;
                opw_parameters.from[1] = joint.from;
                opw_parameters.to[1] = joint.to;
            }
            "joint3" => {
                opw_parameters.c2 = joint.vector.z;
                opw_parameters.b = joint.vector.x;

                opw_parameters.sign_corrections[2] = joint.sign_correction as i8;
                opw_parameters.from[2] = joint.from;
                opw_parameters.to[2] = joint.to;
            }
            "joint4" => {
                opw_parameters.a2 = -joint.vector.non_zero()?;
                opw_parameters.sign_corrections[3] = joint.sign_correction as i8;
                opw_parameters.from[3] = joint.from;
                opw_parameters.to[3] = joint.to;
            }
            "joint5" => {
                opw_parameters.c3 = joint.vector.non_zero()?;
                opw_parameters.sign_corrections[4] = joint.sign_correction as i8;
                opw_parameters.from[4] = joint.from;
                opw_parameters.to[4] = joint.to;
            }
            "joint6" => {
                opw_parameters.c4 = joint.vector.non_zero()?;
                opw_parameters.sign_corrections[5] = joint.sign_correction as i8;
                opw_parameters.from[5] = joint.from;
                opw_parameters.to[5] = joint.to;
            }
            _ => {
                // Other joints for now ignored
            }
        }
    }

    Ok(opw_parameters)
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_process_joints() {
        let xml = r#"
            <robot>
                <joint name="${prefix}JOint_2!">
                    <origin xyz="4.0 5.0 6.0"></origin>
                    <axis xyz="0 0 1"/>
                    <limit lower="-3.15" upper="4.62" effort="0" velocity="3.67"/>
                </joint>            
                <joint name="joint1">
                    <origin xyz="1.0 2.0 3.0"></origin>
                    <axis xyz="0 -1 0"/>
                    <limit lower="-3.14" upper="4.61" effort="0" velocity="3.67"/>
                </joint>
            </robot>
        "#;

        let joint_data = process_joints(xml)
            .expect("Failed to process XML joints");

        assert_eq!(joint_data.len(), 2, "Should have extracted two joints");

        let j1 = &joint_data["joint1"];
        let j2 = &joint_data["joint2"];

        // Tests for joint1
        assert_eq!(j1.name, "joint1", "Joint1 name incorrect");
        assert_eq!(j1.vector.x, 1.0, "Joint1 X incorrect");
        assert_eq!(j1.vector.y, 2.0, "Joint1 Y incorrect");
        assert_eq!(j1.vector.z, 3.0, "Joint1 Z incorrect");
        assert_eq!(j1.sign_correction, -1, "Joint1 sign correction incorrect");
        assert_eq!(j1.from, -3.14, "Joint1 lower limit incorrect");
        assert_eq!(j1.to, 4.61, "Joint1 upper limit incorrect");

        // Tests for joint2
        assert_eq!(j2.name, "joint2", "Joint2 name incorrect");
        assert_eq!(j2.vector.x, 4.0, "Joint2 X incorrect");
        assert_eq!(j2.vector.y, 5.0, "Joint2 Y incorrect");
        assert_eq!(j2.vector.z, 6.0, "Joint2 Z incorrect");
        assert_eq!(j2.sign_correction, 1, "Joint2 sign correction incorrect");
        assert_eq!(j2.from, -3.15, "Joint2 lower limit incorrect");
        assert_eq!(j2.to, 4.62, "Joint2 upper limit incorrect");
    }
}

  

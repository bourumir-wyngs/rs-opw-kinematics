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
/// # Exampls
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
/// This function provides detailed error handling through the `ParameterError` type.
///
/// # Parameters
/// - `xml_content`: A `String` containing the XML data of the URDF file.
/// - `offsets`: joint offsets data (array of f64 one value per joint)
/// - `sorting_weight`: A `f64` value used for sorting joints, BY_PREV = 0 - by previous
///                     BY_CONSTRAINTS = 1 - by center of constraints, intermediate values
///                     possible.
///
/// # Returns
/// - Returns a `Result<OPWKinematics, ParameterError>`. On success, it contains the OPW kinematics
///   configuration for the robot. On failure, it returns a detailed error.
///
/// # Examples
/// ```
/// use std::f64::consts::PI;
/// use rs_opw_kinematics::constraints::BY_PREV;
/// use rs_opw_kinematics::kinematic_traits::Joints;
/// use rs_opw_kinematics::urdf::from_urdf;
/// // Exactly this string would fail. Working URDF fragment would be too long for this example. 
/// let xml_data = String::from("<robot><joint ...></joint></robot>"); 
/// let offsets = [ 0., PI, 0., 0.,0.,0.];
/// let result = from_urdf(xml_data, &offsets, BY_PREV);
/// match result {
///     Ok(kinematics) => println!("{:?}", kinematics),
///     Err(e) => println!("Error processing URDF: {}", e),
/// }
/// ```
pub fn from_urdf(xml_content: String, offsets: &Joints, sorting_weight: f64) -> Result<OPWKinematics, ParameterError> {
    let joint_data = process_joints(&xml_content)
        .map_err(|e|
            ParameterError::XmlProcessingError(format!("Failed to process XML joints: {}", e)))?;

    let opw_parameters = populate_opw_parameters(joint_data)
        .map_err(|e|
            ParameterError::ParameterPopulationError(format!("Failed to interpret robot model: {}", e)))?;

    Ok(opw_parameters.to_robot(sorting_weight, offsets))
}


#[derive(Debug, Default, PartialEq)]
struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
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

#[derive(Default, Debug)]
struct OpwParameters {
    a1: f64,
    a2: f64,
    b: f64,
    c1: f64,
    c2: f64,
    c3: f64,
    c4: f64,
    sign_corrections: [i8; 6],
    from: Joints, // Array to store the lower limits
    to: Joints,   // Array to store the upper limits
}

impl OpwParameters {
    fn to_robot(self, sorting_weight: f64, offsets: &Joints) -> OPWKinematics {
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
}

fn populate_opw_parameters(joint_map: HashMap<String, JointData>) -> Result<OpwParameters, String> {
    let mut opw_parameters = OpwParameters::default();
    for (name, joint) in joint_map {
        match name.as_str() {
            "joint1" => {
                opw_parameters.c1 = joint.vector.z;
                opw_parameters.sign_corrections[0] = joint.sign_correction as i8;
                opw_parameters.from[0] = joint.from;
                opw_parameters.to[0] = joint.to;
            }
            "joint2" => {
                opw_parameters.a1 = joint.vector.x;
                opw_parameters.sign_corrections[1] = joint.sign_correction as i8;
                opw_parameters.from[1] = joint.from;
                opw_parameters.to[1] = joint.to;
            }
            "joint3" => {
                opw_parameters.c2 = joint.vector.z;
                opw_parameters.sign_corrections[2] = joint.sign_correction as i8;
                opw_parameters.from[2] = joint.from;
                opw_parameters.to[2] = joint.to;
            }
            "joint4" => {
                opw_parameters.a2 = -joint.vector.z;
                opw_parameters.sign_corrections[3] = joint.sign_correction as i8;
                opw_parameters.from[3] = joint.from;
                opw_parameters.to[3] = joint.to;
            }
            "joint5" => {
                opw_parameters.c3 = joint.vector.x;
                opw_parameters.sign_corrections[4] = joint.sign_correction as i8;
                opw_parameters.from[4] = joint.from;
                opw_parameters.to[4] = joint.to;
            }
            "joint6" => {
                opw_parameters.c4 = joint.vector.x;
                opw_parameters.b = joint.vector.y; // this is questionable, more likely sum of all dy
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
    use std::fs::read_to_string;
    use std::path::Path;
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

    #[test]
    fn test_opw_parameters_extraction() {
        let filename = "src/tests/m10ia_macro.xacro";
        let path = Path::new(filename);
        let xml_content = read_to_string(path).expect("Failed to read xacro file");

        let joint_data = process_joints(&xml_content)
            .expect("Failed to process XML joints");

        let opw_parameters = populate_opw_parameters(joint_data)
            .expect("Failed to read OpwParameters");

        // Output the results or further process
        println!("{:?}", opw_parameters);

        // opw_kinematics_geometric_parameters:
        //   a1: 0.15
        //   a2: -0.20
        //   b: 0.0
        //   c1: 0.45
        //   c2: 0.60
        //   c3: 0.64
        //   c4: 0.10
        // opw_kinematics_joint_offsets: [0.0, 0.0, deg(-90.0), 0.0, 0.0, deg(180.0)]
        // opw_kinematics_joint_sign_corrections: [1, 1, -1, -1, -1, -1]

        assert_eq!(opw_parameters.a1, 0.15, "a1 parameter mismatch");
        assert_eq!(opw_parameters.a2, -0.2, "a2 parameter mismatch");
        assert_eq!(opw_parameters.b, 0.0, "b parameter mismatch");
        assert_eq!(opw_parameters.c1, 0.45, "c1 parameter mismatch");
        assert_eq!(opw_parameters.c2, 0.6, "c2 parameter mismatch");
        assert_eq!(opw_parameters.c3, 0.64, "c3 parameter mismatch");
        assert_eq!(opw_parameters.c4, 0.1, "c4 parameter mismatch");

        let expected_sign_corrections: [i32; 6] = [1, 1, -1, -1, -1, -1];
        let expected_from: [f64; 6] = [-3.14, -1.57, -3.14, -3.31, -3.31, -6.28];
        let expected_to: [f64; 6] = [3.14, 2.79, 4.61, 3.31, 3.31, 6.28];

        for (i, &val) in expected_sign_corrections.iter().enumerate() {
            assert_eq!(opw_parameters.sign_corrections[i], val as i8,
                       "Mismatch in sign_corrections at index {}", i);
        }

        for (i, &val) in expected_from.iter().enumerate() {
            assert_eq!(opw_parameters.from[i], val, "Mismatch in from at index {}", i);
        }

        for (i, &val) in expected_to.iter().enumerate() {
            assert_eq!(opw_parameters.to[i], val, "Mismatch in to at index {}", i);
        }
    }
}

  

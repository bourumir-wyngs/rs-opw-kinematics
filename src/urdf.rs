//! Supports extracting OPW parameters from URDF (optional)

extern crate sxd_document;

use crate::simplify_joint_name::preprocess_joint_name;
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
/// let kinematics = rs_opw_kinematics::urdf::from_urdf_file("src/tests/data/fanuc/m6ib_macro.xacro");
/// println!("{:?}", kinematics);
/// ```
///
/// # Errors
/// - The function might panic if the file cannot be found, is not accessible, or is incorrectly 
///   formatted. Users should ensure the file path is correct and the file is properly formatted as 
///   URDF or XACRO file.
pub fn from_urdf_file<P: AsRef<Path>>(path: P) -> OPWKinematics {
    let xml_content = read_to_string(path).expect("Failed to read xacro/urdf file");

    let joint_data = process_joints(&xml_content, &None)
        .expect("Failed to process XML joints");

    let opw_parameters = populate_opw_parameters(joint_data, &None)
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
/// - `joint_names`: An optional array containing joint names. This may be required if
///                  names do not follow typical naming convention, or there are multiple
///                  robots defined in URDF. 
///                  For 5 DOF robots, use the name of the tool center point instead of "joint6"
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
///
/// // Let's assume the joint names have prefix and the joints are zero base numbered. 
/// let joints = ["lf_joint_0", "lf_joint_1", "lf_joint_2", "lf_joint_3", "lf_joint_4", "lf_joint_5"];
/// let offsets = [ 0., PI, 0., 0.,0.,0.];
/// let opw_params = from_urdf(xml_data, &Some(joints));
/// match opw_params {
///     Ok(opw_params) => {
///         println!("Building the IK solver {:?}", opw_params);
///         let parameters = opw_params.parameters(&JOINTS_AT_ZERO); // Zero joint offsets
///         let constraints =opw_params.constraints(BY_PREV); 
///         let robot = OPWKinematics::new_with_constraints(parameters, constraints);
///         // let joints = robot.inverse( ... )    
///
///     }
///     Err(e) => println!("Error processing URDF: {}", e),
/// }
/// ```
pub fn from_urdf(xml_content: String, joint_names: &Option<[&str; 6]>) -> Result<URDFParameters, ParameterError> {
    let joint_data = process_joints(&xml_content, joint_names)
        .map_err(|e|
            ParameterError::XmlProcessingError(format!("Failed to process XML joints: {}", e)))?;

    let opw_parameters = populate_opw_parameters(joint_data, joint_names)
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

/// Helper that returns the non-zero value from a pair of numbers if exactly one
/// of them is non-zero. If both are zero, `Ok(0.0)` is returned. Returns an
/// error when both values are non-zero.
fn non_zero_pair(a: f64, b: f64) -> Result<f64, String> {
    match (a, b) {
        (0.0, 0.0) => Ok(0.0),
        (0.0, b) => Ok(b),
        (a, 0.0) => Ok(a),
        (_, _) => Err(String::from("Both potential values are non-zero")),
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

fn process_joints(xml: &str, joint_names: &Option<[&str; 6]>) -> Result<HashMap<String, JointData>, Box<dyn Error>> {
    let package = parser::parse(xml)?;
    let document = package.as_document();

    // Access the root element
    let root_element = document.root().children().into_iter()
        .find_map(|e| e.element())
        .ok_or("No root element found")?;

    // Collect all joint data
    let mut joints = Vec::new();
    collect_joints(root_element, &mut joints, joint_names)?;

    convert_to_map(joints)
}

// Recursive function to collect joint data
fn collect_joints(element: dom::Element, joints: &mut Vec<JointData>, joint_names: &Option<[&str; 6]>) -> Result<(), Box<dyn Error>> {
    let origin_tag = QName::new("origin");
    let joint_tag = QName::new("joint");
    let axis_tag = QName::new("axis");
    let limit_tag = QName::new("limit");

    for child in element.children().into_iter().filter_map(|e| e.element()) {
        if child.name() == joint_tag {
            let name;
            let urdf_name = &child.attribute("name")
                .map(|attr| attr.value().to_string())
                .unwrap_or_else(|| "Unnamed".to_string());
            if joint_names.is_some() {
                // If joint names are explicitly given, they are expected to be as they are.
                name = urdf_name.clone();
            } else {
                // Otherwise effort is done to "simplify" the names into joint1 to joint6
                name = preprocess_joint_name(urdf_name);
            }
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

            match limit_element.map(get_limits).transpose() {
                Ok(Some((from, to))) => {
                    joint_data.from = from;
                    joint_data.to = to;
                }
                Ok(None) => {}
                Err(e) => {
                    println!("Joint limits defined but not not readable for {}: {}",
                             joint_data.name, e.to_string());
                }
            }

            joints.push(joint_data);
        }

        collect_joints(child, joints, joint_names)?;
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
        "'xyz' attribute not found in element supposed to represent the axis"
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
        Ok(0) // This is a fixed joint
    }
}

fn parse_angle(attr_value: &str) -> Result<f64, ParameterError> {
    // Regular expression to match the ${radians(<number>)} format that is common in xacro
    let re = Regex::new(r"^\$\{radians\((-?\d+(\.\d+)?)\)\}$")
        .map_err(|_| ParameterError::ParseError("Invalid regex pattern".to_string()))?;

    // Check if the input matches the special format
    if let Some(caps) = re.captures(attr_value) {
        let degrees_str = caps.get(1)
            .ok_or(ParameterError::WrongAngle(format!("Bad representation: {}",
                                                      attr_value).to_string()))?.as_str();
        let degrees: f64 = degrees_str.parse()
            .map_err(|_| ParameterError::WrongAngle(attr_value.to_string()))?;
        Ok(degrees.to_radians())
    } else {
        // Try to parse the input as a plain number in that case it is in radians
        let radians: f64 = attr_value.parse()
            .map_err(|_| ParameterError::WrongAngle(attr_value.to_string()))?;
        Ok(radians)
    }
}

fn get_limits(element: dom::Element) -> Result<(f64, f64), ParameterError> {
    let lower_attr = element.attribute("lower")
        .ok_or_else(|| ParameterError::MissingField("lower limit not found".into()))?
        .value();
    let lower_limit = parse_angle(lower_attr)?;

    let upper_attr = element.attribute("upper")
        .ok_or_else(|| ParameterError::MissingField("upper limit not found".into()))?
        .value();
    let upper_limit = parse_angle(upper_attr)?;

    Ok((lower_limit, upper_limit))
}

fn convert_to_map(joints: Vec<JointData>) -> Result<HashMap<String, JointData>, Box<dyn Error>> {
    let mut map: HashMap<String, JointData> = HashMap::new();

    for joint in joints {
        if let Some(existing) = map.get(&joint.name) {
            // Check if the existing entry is different from the new one
            if existing != &joint {
                return Err(Box::new(std::io::Error::new(std::io::ErrorKind::InvalidData,
                                                        format!("Duplicate joint name with different data found: {}", joint.name))));
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
    pub dof: i8
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
                dof: self.dof
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
            dof: self.dof
        }
    }
}

fn populate_opw_parameters(joint_map: HashMap<String, JointData>, joint_names: &Option<[&str; 6]>)
                           -> Result<URDFParameters, String> {
    let mut opw_parameters = URDFParameters::default();

    let names = joint_names.unwrap_or_else(
        || ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]);
    
    let is_six_dof = joint_map.contains_key(names[5]);
    opw_parameters.c4 = 0.0; // joint6 would be something like "tcp" for the 5 DOF robot. Otherwise, 0 is assumed.

    for j in 0..6 {
        let joint = joint_map
            .get(names[j]).ok_or_else(|| format!("Joint {} not found: {}", j, names[j]))?;

        opw_parameters.sign_corrections[j] = joint.sign_correction as i8;
        opw_parameters.from[j] = joint.from;
        opw_parameters.to[j] = joint.to;

        match j + 1 { // Joint number 1 to 6 inclusive
            1 => {
                opw_parameters.c1 = joint.vector.non_zero()?;
            }
            2 => {
                opw_parameters.a1 = joint.vector.non_zero()?;
            }
            3 => {
                // There is more divergence here. 
                match joint.vector.non_zero() {
                    Ok(value) => {
                        // If there is only one value, it is value for c2. Most of the 
                        // modern robots we tested do follow this design.
                        opw_parameters.c2 = value;
                        opw_parameters.b = 0.0;
                    }
                    Err(_err) => {
                        // If there are multiple values, we assume b is given here as y.
                        // c2 is given either in z or in x, other being 0.
                        opw_parameters.c2 = non_zero_pair(joint.vector.x, joint.vector.z)?;
                        opw_parameters.b = joint.vector.y;
                    }
                }
            }
            4 => {
                match joint.vector.non_zero() {
                    Ok(value) => {
                        opw_parameters.a2 = -value;
                    }
                    Err(_err) => {
                        // If there are multiple values, we assume a2 is given here as z.
                        // c3 is given either in y or in x, other being 0.
                        opw_parameters.a2 = -joint.vector.z;
                        if opw_parameters.c3 != 0.0 {
                            return Err(String::from("C3 seems defined twice (J4)"));
                        }
                        opw_parameters.c3 = non_zero_pair(joint.vector.x, joint.vector.y)?;
                    }
                }
            }
            5 => {
                let candidate = joint.vector.non_zero()?;
                if candidate != 0.0 {
                    if opw_parameters.c3 != 0.0 {
                        return Err(String::from("C3 seems defined twice (J5)"));
                    } else {
                        opw_parameters.c3 = candidate;
                    }
                }
            }
            6 => {
                opw_parameters.c4 = joint.vector.non_zero()?;
            }
            _ => {
                // Other joints not in use
            }
        }
    }

    if is_six_dof {
        opw_parameters.dof = 6    
    } else {
        opw_parameters.dof = 5;
        
        // Set reasonable values for non-existing joint 6.
        // Constraint checker will still be checking this range.
        opw_parameters.sign_corrections[5] = 0; // Always suppress
        // With from=to, constraint is suppressed.
        opw_parameters.from[5] = 0.0; 
        opw_parameters.to[5] = 0.0;       
    }
    

    Ok(opw_parameters)
}

#[allow(dead_code)]
// This function is not in use and exists for references only (old version)
fn populate_opw_parameters_explicit(joint_map: HashMap<String, JointData>, joint_names: &Option<[&str; 6]>)
                                    -> Result<URDFParameters, String> {
    let mut opw_parameters = URDFParameters::default();

    opw_parameters.b = 0.0; // We only support robots with b = 0 so far.

    let names = joint_names.unwrap_or_else(
        || ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]);

    for j in 0..6 {
        let joint = joint_map
            .get(names[j]).ok_or_else(|| format!("Joint {} not found: {}", j, names[j]))?;

        opw_parameters.sign_corrections[j] = joint.sign_correction as i8;
        opw_parameters.from[j] = joint.from;
        opw_parameters.to[j] = joint.to;

        match j + 1 { // Joint number 1 to 6 inclusive
            1 => {
                opw_parameters.c1 = joint.vector.z;
            }
            2 => {
                opw_parameters.a1 = joint.vector.x;
            }
            3 => {
                opw_parameters.c2 = joint.vector.z;
                opw_parameters.b = joint.vector.y;
                //opw_parameters.c2 = joint.vector.x; // Kuka                
            }
            4 => {
                opw_parameters.a2 = -joint.vector.z;
            }
            5 => {
                opw_parameters.c3 = joint.vector.x;
                opw_parameters.c3 = joint.vector.z; // TX40                
            }
            6 => {
                opw_parameters.c4 = joint.vector.x;
                opw_parameters.c4 = joint.vector.z; // TX40                
            }
            _ => {
                // Other joints not in use
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

        let joint_data = process_joints(xml, &None)
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
    fn test_populate_named_joints() {
        // Let's say they are zero-base numbered and have the side prefix in front.
        // Also, there are joints for another robot with different prefix (multiple robots in URDF).
        // This test shows that multiple robots are supported (if URDF defines a multi-robot cell)
        let xml = r#"
            <robot>
                <joint name="right_joint_0" type="revolute"> <!-- must be ignored -->
                  <origin xyz="0 0 0.950" rpy="0 0 0"/>
                  <parent link="right_base_link"/>
                  <child link="right_link_1"/>
                  <axis xyz="0 0 1"/>
                  <limit lower="-2.9670" upper="2.9670" effort="0" velocity="2.6179"/>
                </joint>            
                <joint name="left_joint_0" type="revolute"> <!-- numbered from 0 -->
                  <origin xyz="0 0 0.450" rpy="0 0 0"/>
                  <parent link="left_base_link"/>
                  <child link="left_link_1"/>
                  <axis xyz="0 0 1"/>
                  <limit lower="-2.9670" upper="2.9670" effort="0" velocity="2.6179"/>
                </joint>
                <joint name="left_joint_1" type="revolute">
                  <origin xyz="0.150 0 0" rpy="0 0 0"/>
                  <parent link="left_link_1"/>
                  <child link="left_link_2"/>
                  <axis xyz="0 1 0"/>
                  <limit lower="-1.5707" upper="2.7925" effort="0" velocity="2.7925"/>
                </joint>
                <joint name="left_joint_2" type="revolute">
                  <origin xyz="0 0 0.600" rpy="0 0 0"/>
                  <parent link="left_link_2"/>
                  <child link="left_link_3"/>
                  <axis xyz="0 -1 0"/>
                  <limit lower="-2.9670" upper="2.9670" effort="0" velocity="2.9670"/>
                </joint>
                <joint name="left_joint_3" type="revolute">
                  <origin xyz="0 0 0.100" rpy="0 0 0"/>
                  <parent link="left_link_3"/>
                  <child link="left_link_4"/>
                  <axis xyz="-1 0 0"/>
                  <limit lower="-3.3161" upper="3.3161" effort="0" velocity="6.9813"/>
                </joint>
                <joint name="left_joint_4" type="revolute">
                  <origin xyz="0.615 0 0" rpy="0 0 0"/>
                  <parent link="left_link_4"/>
                  <child link="left_link_5"/>
                  <axis xyz="0 -1 0"/>
                  <limit lower="-2.4434" upper="2.4434" effort="0" velocity="6.9813"/>
                </joint>
                <joint name="left_joint_5" type="revolute">
                  <origin xyz="0.100 0 0" rpy="0 0 0"/>
                  <parent link="left_link_5"/>
                  <child link="left_link_6"/>
                  <axis xyz="-1 0 0"/>
                  <limit lower="-6.2831" upper="6.2831" effort="0" velocity="9.0757"/>
                </joint>                               
            </robot>
        "#;

        let joints = ["left_joint_0", "left_joint_1", "left_joint_2",
            "left_joint_3", "left_joint_4", "left_joint_5"];

        let opw_parameters =
            from_urdf(xml.to_string(), &Some(joints)).expect("Failed to parse parameters");

        assert_eq!(opw_parameters.a1, 0.15, "a1 parameter mismatch");
        assert_eq!(opw_parameters.a2, -0.10, "a2 parameter mismatch");
        assert_eq!(opw_parameters.b, 0.0, "b parameter mismatch");
        assert_eq!(opw_parameters.c1, 0.45, "c1 parameter mismatch");
        assert_eq!(opw_parameters.c2, 0.6, "c2 parameter mismatch");
        assert_eq!(opw_parameters.c3, 0.615, "c3 parameter mismatch");
        assert_eq!(opw_parameters.c4, 0.10, "c4 parameter mismatch");
    }
}

  

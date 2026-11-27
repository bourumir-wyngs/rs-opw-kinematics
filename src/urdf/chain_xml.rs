use super::{UrdfJointLimit, continuous_position_limits, parse_finite_xml_number};
use crate::kinematic_traits::Pose;
use crate::parameter_error::ParameterError::{self, XmlProcessingError};
use glam::{DQuat, DVec3, EulerRot};
use std::collections::{HashMap, HashSet};
use sxd_document::{QName, dom::Element, parser};

pub(super) struct ChainData {
    pub(super) origins: [Pose; 6],
    pub(super) axes: [DVec3; 6],
    pub(super) tip: Pose,
    pub(super) limits: [UrdfJointLimit; 6],
}

pub(super) fn read_chain(
    xml: &str,
    base_link: &str,
    tip_link: &str,
) -> Result<ChainData, ParameterError> {
    let package = parser::parse(xml).map_err(|e| XmlProcessingError(e.to_string()))?;
    let root = package
        .as_document()
        .root()
        .children()
        .into_iter()
        .find_map(|node| node.element())
        .filter(|root| root.name() == QName::new("robot"))
        .ok_or_else(|| XmlProcessingError("expected a URDF robot element".into()))?;
    if let Some(version) = root.attribute_value("version")
        && version != "1.0"
    {
        return Err(XmlProcessingError(format!(
            "unsupported URDF version {version:?}; expected 1.0"
        )));
    }
    let path = joint_path(root, base_link, tip_link)?;
    let mut data = ChainData {
        origins: [Pose::IDENTITY; 6],
        axes: [DVec3::ZERO; 6],
        limits: [UrdfJointLimit::Unspecified; 6],
        tip: Pose::IDENTITY,
    };
    let mut count = 0;
    let mut names = HashSet::new();
    for joint in path {
        let name = joint
            .attribute_value("name")
            .filter(|name| !name.is_empty())
            .ok_or_else(|| XmlProcessingError("joint on selected chain has no name".into()))?;
        if !names.insert(name) {
            return Err(XmlProcessingError(format!(
                "duplicate joint name {name:?} on selected chain"
            )));
        }
        data.add_joint(joint, &mut count)
            .map_err(|error| XmlProcessingError(format!("joint {name:?}: {error}")))?;
    }
    if count != 6 {
        return Err(XmlProcessingError(format!(
            "chain from {base_link:?} to {tip_link:?} has {count} rotary joints; expected six"
        )));
    }
    Ok(data)
}

fn child<'d>(element: Element<'d>, tag: &str) -> Option<Element<'d>> {
    element
        .children()
        .into_iter()
        .filter_map(|node| node.element())
        .find(|child| child.name() == QName::new(tag))
}

// Index only physical joint definitions, then walk towards the base. XML order and
// references inside transmission, ros2_control and other extensions are immaterial.
fn joint_path<'d>(
    robot: Element<'d>,
    base: &str,
    tip: &str,
) -> Result<Vec<Element<'d>>, ParameterError> {
    let mut incoming: HashMap<_, Vec<_>> = HashMap::new();
    for joint in robot
        .children()
        .into_iter()
        .filter_map(|node| node.element())
        .filter(|node| node.name() == QName::new("joint"))
    {
        if let Some(link) = child(joint, "child").and_then(|e| e.attribute_value("link")) {
            // Multiple incoming joints matter only if this link is on our path.
            incoming.entry(link).or_default().push(joint);
        }
    }
    let mut path = Vec::new();
    let mut visited = HashSet::new();
    let mut link = tip;
    let mut reached_base = false;
    // Continue through the base to catch a loop that closes via a mounting link.
    loop {
        if !visited.insert(link) {
            return Err(XmlProcessingError(format!(
                "cycle at link {link:?} in ancestry of {tip:?}"
            )));
        }
        reached_base |= link == base;
        let Some(parents) = incoming.get(link) else {
            if reached_base {
                break;
            }
            return Err(XmlProcessingError(format!(
                "no path from {base:?} to {tip:?}: no parent joint for link {link:?}"
            )));
        };
        let &[joint] = parents.as_slice() else {
            return Err(XmlProcessingError(format!(
                "multiple parent joints for link {link:?}"
            )));
        };
        if !reached_base {
            path.push(joint);
        }
        link = child(joint, "parent")
            .and_then(|e| e.attribute_value("link"))
            .ok_or_else(|| XmlProcessingError(format!("missing parent link for child {link:?}")))?;
    }
    path.reverse();
    Ok(path)
}

impl ChainData {
    fn add_joint(&mut self, joint: Element<'_>, count: &mut usize) -> Result<(), String> {
        if let Some(origin) = child(joint, "origin") {
            if origin.attribute_value("quat_xyzw").is_some() {
                return Err("quat_xyzw is unsupported in URDF 1.0; use rpy".into());
            }
            let xyz = vector(origin.attribute_value("xyz").unwrap_or("0 0 0"), "xyz")?;
            let rpy = vector(origin.attribute_value("rpy").unwrap_or("0 0 0"), "rpy")?;
            // URDF specifies fixed-axis roll, pitch, yaw.
            let rotation = DQuat::from_euler(EulerRot::ZYX, rpy.z, rpy.y, rpy.x);
            self.tip =
                Pose::try_from_parts(self.tip.transform_point(xyz), self.tip.rotation * rotation)
                    .map_err(|e| e.to_string())?;
        }
        let kind = joint.attribute_value("type").ok_or("missing joint type")?;
        match kind {
            "fixed" => return Ok(()),
            "revolute" | "continuous" => {}
            other => return Err(format!("unsupported joint type {other:?}")),
        }
        if child(joint, "mimic").is_some() {
            return Err("mimic joints cannot be used as independent rotary axes".into());
        }
        if *count >= self.axes.len() {
            return Err("selected chain has more than six rotary joints".into());
        }
        let axis = match child(joint, "axis") {
            None => DVec3::X,
            Some(e) => vector(e.attribute_value("xyz").ok_or("missing axis xyz")?, "axis")?,
        };
        let length = axis.length();
        if (length - 1.0).abs() > 1e-6 {
            return Err(format!("expected a unit axis, found length {length}"));
        }
        self.axes[*count] = self.tip.transform_vector(axis / length);
        self.origins[*count] = self.tip;
        self.limits[*count] = position_limits(kind, child(joint, "limit"))?;
        *count += 1;
        Ok(())
    }
}

fn vector(value: &str, field: &str) -> Result<DVec3, String> {
    let mut numbers = value.split_whitespace();
    let mut xyz = [0.0; 3];
    for component in &mut xyz {
        let number = numbers
            .next()
            .ok_or_else(|| format!("{field} expects three numbers"))?;
        *component = parse_finite_xml_number(number, field)
            .map_err(|e| format!("invalid {field} component {number:?}: {e}"))?;
    }
    if numbers.next().is_some() {
        return Err(format!("{field} expects three numbers"));
    }
    Ok(DVec3::from_array(xyz))
}

fn position_limits(kind: &str, limit: Option<Element<'_>>) -> Result<UrdfJointLimit, String> {
    let parse_bound = |name: &str, value: &str| {
        parse_finite_xml_number(value, name)
            .map_err(|e| format!("invalid {name} position limit {value:?}: {e}"))
    };
    if kind == "continuous" {
        return continuous_position_limits(limit, parse_bound);
    }
    let Some(limit) = limit else {
        return Ok(UrdfJointLimit::Unspecified);
    };
    // In URDF 1.0 each omitted endpoint of a present limit element defaults to zero.
    let lower = parse_bound("lower", limit.attribute_value("lower").unwrap_or("0"))?;
    let upper = parse_bound("upper", limit.attribute_value("upper").unwrap_or("0"))?;
    if lower > upper {
        return Err(format!(
            "lower position limit {lower} exceeds upper limit {upper}"
        ));
    }
    Ok(UrdfJointLimit::Bounded { lower, upper })
}

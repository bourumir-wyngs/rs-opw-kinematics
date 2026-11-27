use super::{chain_xml::read_chain, geometry::extract_geometry};
use crate::constraints::Constraints;
use crate::kinematic_traits::Pose;
use crate::kinematics_impl::OPWKinematics;
use crate::parameter_error::ParameterError;
use crate::parameters::opw_kinematics::Parameters;
use crate::tool::{Base, Tool};
use std::f64::consts::PI;
use std::sync::Arc;

/// Position limits in URDF joint coordinates, in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum UrdfJointLimit {
    /// A revolute joint with no limit element in the input.
    Unspecified,
    /// A continuous joint with unlimited rotation.
    Continuous,
    /// A revolute joint with inclusive travel limits.
    Bounded { lower: f64, upper: f64 },
}

/// OPW geometry, joint limits and frames for a selected URDF chain.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
pub struct UrdfChain {
    /// OPW geometry, including signs and offsets for the URDF joint coordinates.
    pub parameters: Parameters,
    /// Pose of the OPW base in the requested base link.
    pub base: Pose,
    /// Pose of the requested tip link in the OPW flange frame.
    pub tool: Pose,
    /// Position limits in base-to-tip joint order.
    pub limits: [UrdfJointLimit; 6],
}

/// Imports a six-axis arm from an expanded URDF 1.0 document.
///
/// `xml` supplies the document. `base_link` and `tip_link` are exact link names,
/// with the base an ancestor of the tip. Fixed joints may appear anywhere on
/// that path. The result uses the six rotary joints in path order.
///
/// At zero the separation of the shoulder and elbow axes is along the base
/// axis. The six axis directions in the arm's mount frame are Z, Y, Y, X,
/// Y, X (either sign), with coincident wrist roll axes and an intersecting
/// wrist pitch axis. Mount and tool transforms can have arbitrary orientations.
///
/// # Errors
///
/// Returns a [`ParameterError`] for invalid XML, an ambiguous or incomplete
/// path, unsupported joints, or geometry outside this layout.
///
/// ```
/// use rs_opw_kinematics::constraints::BY_PREV;
/// use rs_opw_kinematics::kinematic_traits::Kinematics;
/// use rs_opw_kinematics::urdf::from_urdf_chain;
///
/// let xml = include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/src/tests/data/abb/irb120_3_58.urdf"));
/// let chain = from_urdf_chain(xml, "base_link", "tool0").unwrap();
/// assert_eq!(chain.parameters.c1, 0.29);
/// let tip_at_home = chain.to_robot(BY_PREV).unwrap().forward(&[0.0; 6]);
/// assert!((tip_at_home.translation.x - 0.374).abs() < 1e-10);
/// ```
pub fn from_urdf_chain(
    xml: &str,
    base_link: &str,
    tip_link: &str,
) -> Result<UrdfChain, ParameterError> {
    let chain = read_chain(xml, base_link, tip_link)?;
    let (parameters, base, tool) = extract_geometry(&chain.origins, &chain.axes, chain.tip)?;
    Ok(UrdfChain {
        parameters,
        base,
        tool,
        limits: chain.limits,
    })
}

impl UrdfChain {
    /// Builds a solver whose forward and inverse poses use the requested base and tip.
    /// Intermediate joint poses use the solver's OPW frames. URDF link meshes
    /// can have different local origins and orientations.
    ///
    /// `sorting_weight` has the meaning used by [`Constraints::new`]. Unspecified
    /// and continuous joints are unrestricted. Bounded ranges shorter than one
    /// turn repeat every revolution in the solver. Wider ranges retain absolute
    /// travel stops. The original URDF bounds are available in [`Self::limits`].
    ///
    /// # Errors
    ///
    /// Equal endpoints and exactly one full turn both mean unrestricted rotation
    /// in [`Constraints`]. A bounded joint with either range returns an error.
    ///
    /// # Panics
    ///
    /// Manually changing `parameters.offsets` to values outside ±2π or to
    /// non-finite values causes [`OPWKinematics::new_with_constraints`] to panic.
    pub fn to_robot(&self, sorting_weight: f64) -> Result<Tool, ParameterError> {
        let mut from = [0.0; 6];
        let mut to = [0.0; 6];
        for (index, limit) in self.limits.iter().enumerate() {
            if let UrdfJointLimit::Bounded { lower, upper } = *limit {
                if !lower.is_finite() || !upper.is_finite() || lower > upper {
                    return Err(ParameterError::KinematicsConfigurationError(format!(
                        "J{} has invalid position limits [{lower}, {upper}]",
                        index + 1
                    )));
                }
                if lower == upper || upper.midpoint(-lower) == PI {
                    return Err(ParameterError::KinematicsConfigurationError(format!(
                        "J{} is bounded to [{lower}, {upper}], which Constraints treats as unrestricted",
                        index + 1
                    )));
                }
                from[index] = lower;
                to[index] = upper;
            }
        }
        let robot = OPWKinematics::new_with_constraints(
            self.parameters,
            Constraints::new(from, to, sorting_weight),
        );
        let mounted = Base {
            robot: Arc::new(robot),
            base: self.base,
        };
        Ok(Tool {
            robot: Arc::new(mounted),
            tool: self.tool,
        })
    }
}

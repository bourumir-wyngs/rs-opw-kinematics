//! Provides tool and base for the robot. 
//! Both Tool and Base take arbitrary implementation of Kinematics and are such
//! implementations themselves. Hence, they can be cascaded, like base, having the robot,
//! that robot having a tool:
//! // Robot with the tool, standing on a base:
//! ```
//! use std::sync::Arc;
//! use nalgebra::{Isometry3, Translation3, UnitQuaternion};
//! use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose};
//! use rs_opw_kinematics::kinematics_impl::OPWKinematics;
//! use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
//! let robot_alone = OPWKinematics::new(Parameters::staubli_tx2_160l());
//! 
//! // Half meter high pedestal
//! let pedestal = 0.5;
//! let base_translation = Isometry3::from_parts(
//!   Translation3::new(0.0, 0.0, pedestal).into(),
//!   UnitQuaternion::identity(),
//! );
//!
//! let robot_with_base = rs_opw_kinematics::tool::Base {
//!   robot: Arc::new(robot_alone),
//!   base: base_translation,
//! };
//!
//! // Tool extends 1 meter in the Z direction, envisioning something like sword
//! let sword = 1.0;
//! let tool_translation = Isometry3::from_parts(
//!   Translation3::new(0.0, 0.0, sword).into(),
//!   UnitQuaternion::identity(),
//! );
//!
//! // Create the Tool instance with the transformation
//! let robot_complete = rs_opw_kinematics::tool::Tool {
//!   robot: Arc::new(robot_with_base),
//!   tool: tool_translation,
//! };
//!
//! let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; // Joints are alias of [f64; 6]
//! let tcp_pose: Pose = robot_complete.forward(&joints);
//! println!("The sword tip is at: {:?}", tcp_pose);
//! ```

extern crate nalgebra as na;

use std::sync::Arc;
use na::{Isometry3};
use nalgebra::Translation3;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};


/// Defines the fixed tool that can be attached to the last joint (joint 6) of robot.
/// The tool moves with the robot, providing additional translation and, if needed,
/// rotation. The tool itself fully implements the Kinematics,
/// providing both inverse and forward kinematics for the robot with a tool (with
/// "pose" being assumed as the position and rotation of the tip of the tool (tool center point). 
#[derive(Clone)]
pub struct Tool {
    pub robot: Arc<dyn Kinematics>,  // The robot

    /// Transformation from the robot's tip joint to the tool's TCP.    
    pub tool: Isometry3<f64>,
}

/// Defines the fixed base that can hold the robot.
/// The base moves the robot to its installed location, providing also rotation if 
/// required (physical robots work well and may be installed upside down, or at some 
/// angle like 45 degrees). Base itself fully implements the Kinematics,
/// providing both inverse and forward kinematics for the robot on a base.
#[derive(Clone)]
pub struct Base {
    pub robot: Arc<dyn Kinematics>,  // The robot

    /// Transformation from the world origin to the robots base.
    pub base: Isometry3<f64>,
}

impl Kinematics for Tool {
    fn inverse(&self, tcp: &Pose) -> Solutions {
        self.robot.inverse(&(tcp * self.tool.inverse()))
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        self.robot.inverse_continuing(&(tcp * self.tool.inverse()), previous)
    }

    fn forward(&self, qs: &Joints) -> Pose {
        // Calculate the pose of the tip joint using the robot's kinematics
        let tip_joint = self.robot.forward(qs);
        let tcp = tip_joint * self.tool;
        tcp
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.robot.kinematic_singularity(qs)
    }
}

impl Kinematics for Base {
    fn inverse(&self, tcp: &Pose) -> Solutions {
        self.robot.inverse(&(self.base.inverse() * tcp))
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        self.robot.inverse_continuing(&(self.base.inverse() * tcp), &previous)
    }

    fn forward(&self, joints: &Joints) -> Pose {
        self.base * self.robot.forward(joints)
    }

    fn kinematic_singularity(&self, qs: &Joints) -> Option<Singularity> {
        self.robot.kinematic_singularity(qs)
    }
}

// Define the Cart (linear axis, prismatic joint) structure that can hold the robot. 
// The cart is moving in parallel to Cartesian axis (x, y, z) and provides the forward kinematics.
// Same as joint positions for the robot, cart prismatic joints are not stored
// in this structure. The linear axis can be itself placed on the base.
#[derive(Clone)]
pub struct LinearAxis {
    robot: Arc<dyn Kinematics>,
    axis: u32,
    /// The base of the axis (not the robot on the axis)
    pub base: Isometry3<f64>,
}

/// A platform for a robot that can ride in x, y and z directions. This way it is less
/// restricted than LinearAxis but tasks focusing with moving in one dimension only
/// may prefer abstracting which dimension it is.
#[derive(Clone)]
pub struct Gantry {
    robot: Arc<dyn Kinematics>,
    /// The base of the gantry crane (not the robot on the gantry crane)
    pub base: Isometry3<f64>,
}

/// A platform for a robot that can ride in one of the x, y and z directions. 
/// (a more focused version of the Gantry, intended for algorithms that focus with the single
/// direction variable.) 
impl LinearAxis {
    // Compute the forward transformation including the cart's offset and the robot's kinematics
    pub fn forward(&self, distance: f64, joint_angles: &[f64; 6]) -> Isometry3<f64> {
        let cart_translation = match self.axis {
            0 => Translation3::new(distance, 0.0, 0.0),
            1 => Translation3::new(0.0, distance, 0.0),
            2 => Translation3::new(0.0, 0.0, distance),
            _ => panic!("Invalid axis index; must be 0 (x), 1 (y), or 2 (z)"),
        };
        let robot_pose = self.robot.forward(joint_angles);
        self.base * cart_translation * robot_pose
    }
}

impl Gantry {
    // Compute the forward transformation including the cart's offset and the robot's kinematics
    pub fn forward(&self, translation: &Translation3<f64>, joint_angles: &[f64; 6]) -> Isometry3<f64> {
        let robot_pose = self.robot.forward(joint_angles);
        self.base * translation * robot_pose
    }
}


#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use super::*;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;

    /// Asserts that two `Translation3<f64>` instances are approximately equal within a given tolerance.
    pub(crate) fn assert_diff(a: &Translation3<f64>, b: &Translation3<f64>, expected_diff: [f64; 3], epsilon: f64) {
        let actual_diff = a.vector - b.vector;

        assert!(
            (actual_diff.x - expected_diff[0]).abs() <= epsilon,
            "X difference is not as expected: actual difference = {}, expected difference = {}",
            actual_diff.x, expected_diff[0]
        );
        assert!(
            (actual_diff.y - expected_diff[1]).abs() <= epsilon,
            "Y difference is not as expected: actual difference = {}, expected difference = {}",
            actual_diff.y, expected_diff[1]
        );
        assert!(
            (actual_diff.z - expected_diff[2]).abs() <= epsilon,
            "Z difference is not as expected: actual difference = {}, expected difference = {}",
            actual_diff.z, expected_diff[2]
        );
    }

    fn diff(robot_without: &dyn Kinematics, robot_with: &dyn Kinematics, joints: &[f64; 6]) -> (Pose, Pose) {
        let tcp_without_tool = robot_without.forward(&joints);
        let tcp_with_tool = robot_with.forward(&joints);
        (tcp_without_tool, tcp_with_tool)
    }
   
    #[test]
    fn test_tool() {
        // Parameters for Staubli TX40 robot are assumed to be correctly set in OPWKinematics::new
        let robot_without_tool = OPWKinematics::new(Parameters::staubli_tx2_160l());

        // Tool extends 1 meter in the Z direction
        let tool_translation = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.0).into(),
            UnitQuaternion::identity(),
        );

        // Create the Tool instance with the transformation
        let robot_with_tool = Tool {
            robot: Arc::new(robot_without_tool),
            tool: tool_translation,
        };

        // Joints are all at zero position
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation, [0., 0., 1.], 1E-6);

        // Rotating J6 by any angle should not change anything.
        // Joints are all at zero position
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, PI / 6.0];
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation, [0., 0., 1.], 1E-6);

        // Rotating J5 by 90 degrees result in offset
        let joints = [0.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation, [1.0, 0.0, 0.], 1E-6);

        // Rotate base joint around, sign must change.
        let joints = [PI, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation, [-1.0, 0.0, 0.], 1E-6);

        // Rotate base joint 90 degrees, must become Y
        let joints = [PI / 2.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation, [0.0, 1.0, 0.], 1E-6);

        // Rotate base joint 45 degrees, must divide between X and Y
        let joints = [PI / 4.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let catet = 45.0_f64.to_radians().sin();
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation,
                    [catet, catet, 0.], 1E-6);

        // Rotate base joint 45 degrees, must divide between X and Y, and also raise 45 deg up
        let joints = [PI / 4.0, 0.0, 0.0, 0.0, PI / 4.0, 0.0];
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation,
                    [0.5, 0.5, 2.0_f64.sqrt() / 2.0], 1E-6);
    }

    #[test]
    fn test_base() {
        // Parameters for Staubli TX40 robot are assumed to be correctly set in OPWKinematics::new
        let robot_without_base = OPWKinematics::new(Parameters::staubli_tx2_160l());

        // 1 meter high pedestal
        let base_translation = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.0).into(),
            UnitQuaternion::identity(),
        );

        let robot_with_base = Base {
            robot: Arc::new(robot_without_base),
            base: base_translation,
        };

        // Joints are all at zero position
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

        let (tcp_without_base, tcp_with_base) = diff(&robot_without_base, &robot_with_base, &joints);
        assert_diff(&tcp_with_base.translation, &tcp_without_base.translation, [0., 0., 1.], 1E-6);

        // Rotate base joint around, sign must not change.
        let joints = [PI / 3.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_without_base, tcp_with_base) = diff(&robot_without_base, &robot_with_base, &joints);
        assert_diff(&tcp_with_base.translation, &tcp_without_base.translation, [0.0, 0.0, 1.0], 1E-6);
    }

    #[test]
    fn test_linear_axis_forward() {
        let robot_without_base = OPWKinematics::new(Parameters::staubli_tx2_160l());
        let base_translation = Isometry3::from_parts(
            Translation3::new(0.1, 0.2, 0.3).into(),
            UnitQuaternion::identity(),
        );

        let cart = LinearAxis {
            robot: Arc::new(robot_without_base),
            axis: 1,
            base: base_translation,
        };

        let joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let result = cart.forward(7.0, &joint_angles);

        assert_eq!(result.translation.vector.y,
                   robot_without_base.forward(&joint_angles).translation.vector.y + 7.0 + 0.2);
    }

    #[test]
    fn test_gantry_forward() {
        let robot_without_base = OPWKinematics::new(Parameters::staubli_tx2_160l());

        let base_translation = Isometry3::from_parts(
            Translation3::new(0.1, 0.2, 0.3).into(),
            UnitQuaternion::identity(),
        );

        let gantry = Gantry {
            robot: Arc::new(robot_without_base),
            base: base_translation,
        };

        let joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let r = gantry.forward(
            &Translation3::new(7.0, 8.0, 9.0),
            &joint_angles).translation;

        let alone = robot_without_base.forward(&joint_angles).translation;

        // Gantry riding plus gantry base.
        assert_eq!(r.x, alone.x + 7.1);
        assert_eq!(r.y, alone.y + 8.2);
        assert_eq!(r.z, alone.z + 9.3);
    }

    /// Complete test that includes robot on linear axis, standing on the base and equipped
    /// witht he tool.
    #[test]
    fn test_complete_robot() {
        fn diff(alone: &dyn Kinematics, riding: &LinearAxis, axis: f64, joints: &[f64; 6]) -> (Pose, Pose) {
            let tcp_alone = alone.forward(&joints);
            let tcp = riding.forward(axis, &joints);
            (tcp_alone, tcp)
        }

        let robot_alone = OPWKinematics::new(Parameters::staubli_tx2_160l());

        // Half meter high pedestal
        let pedestal = 0.5;
        let base_translation = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, pedestal).into(),
            UnitQuaternion::identity(),
        );

        let robot_with_base = Base {
            robot: Arc::new(robot_alone),
            base: base_translation,
        };

        // Tool extends 1 meter in the Z direction, envisioning something like sword
        let sword = 1.0;
        let tool_translation = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, sword).into(),
            UnitQuaternion::identity(),
        );

        // Create the Tool instance with the transformation
        let robot_complete = Tool {
            robot: Arc::new(robot_with_base),
            tool: tool_translation,
        };


        // Gantry is based with 0.75 horizontal offset along y
        let gantry_base = 0.75;
        let gantry_translation = Isometry3::from_parts(
            Translation3::new(0.0, gantry_base, 0.0).into(),
            UnitQuaternion::identity(),
        );

        let riding_robot = LinearAxis {
            robot: Arc::new(robot_complete),
            axis: 0,
            base: gantry_translation,
        };

        // Joints are all at zero position
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let axis = 0.0;

        let (tcp_alone, tcp) = diff(&robot_alone, &riding_robot, axis, &joints);
        assert_diff(&tcp.translation, &tcp_alone.translation, 
                    [0., gantry_base, pedestal + sword], 1E-6);

        // Rotating J6 by any angle should not change anything.
        // Joints are all at zero position
        let joints = [0.0, 0.0, 0.0, 0.0, 0.0, PI / 6.0];
        let (tcp_alone, tcp) = diff(&robot_alone, &riding_robot, axis, &joints);
        assert_diff(&tcp.translation, &tcp_alone.translation,
                    [0., gantry_base, pedestal + sword], 1E-6);

        // Rotating J5 by 90 degrees result in offset horizontally for the sword.
        let joints = [0.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_alone, tcp) = diff(&robot_alone, &riding_robot, axis, &joints);
        assert_diff(&tcp.translation, &tcp_alone.translation, 
                    [sword, gantry_base, pedestal], 1E-6);

        // Rotate base joint around, sign for the sword must change.
        let joints = [PI, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_alone, tcp) = diff(&robot_alone, &riding_robot, axis, &joints);
        assert_diff(&tcp.translation, &tcp_alone.translation,
                    [-sword, gantry_base, pedestal], 1E-6);

        // Rotate base joint 90 degrees, swords contributes to Y now
        let joints = [PI / 2.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_alone, tcp) = diff(&robot_alone, &riding_robot, axis, &joints);
        assert_diff(&tcp.translation, &tcp_alone.translation,
                    [0.0, gantry_base + sword, pedestal], 1E-6);

        // Rotate base joint 45 degrees, the effect of sword must divide between X and Y
        let joints = [PI / 4.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let catet = 45.0_f64.to_radians().sin();
        let (tcp_alone, tcp) = diff(&robot_alone, &riding_robot, axis, &joints);
        assert_diff(&tcp.translation, &tcp_alone.translation,
                    [catet, catet + gantry_base, pedestal], 1E-6);

        // Rotate base joint 45 degrees, must divide between X and Y, and also raise 45 deg up
        let joints = [PI / 4.0, 0.0, 0.0, 0.0, PI / 4.0, 0.0];
        let (tcp_alone, tcp) = diff(&robot_alone, &riding_robot, axis, &joints);
        assert_diff(&tcp.translation, &tcp_alone.translation,
                    [sword * 0.5, sword * 0.5 + gantry_base, sword * 2.0_f64.sqrt() / 2.0 + pedestal], 1E-6);


        // Ride the gantry 10 meters along x.
        let ride = 10.0;
        let tcp_translation = riding_robot.forward(ride, &joints).translation;
        assert_diff(&tcp_translation, &tcp_alone.translation,
                    [sword * 0.5 + ride, sword * 0.5 + gantry_base, 
                        sword * 2.0_f64.sqrt() / 2.0 + pedestal], 1E-6);       
    }
}


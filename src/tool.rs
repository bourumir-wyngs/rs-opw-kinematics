extern crate nalgebra as na;

use na::{Isometry3};
use nalgebra::Translation3;
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;


struct Tool {
    robot: Box<dyn Kinematics>,  // The robot

    /// Transformation from the robot's tip joint to the tool's TCP.    
    pub tool: Isometry3<f64>,
}

struct Base {
    robot: Box<dyn Kinematics>,  // The robot

    /// Transformation from the world origin to the robots base.
    pub base: Isometry3<f64>,
}

impl Kinematics for Tool {
    fn inverse(&self, tcp: &Pose) -> Solutions {
        let tip_joint = tcp * self.tool.inverse();
        self.robot.inverse(&tip_joint)
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        let tip_joint = tcp * self.tool.inverse();
        self.robot.inverse_continuing(&tip_joint, previous)
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
        let robot_transform = self.base.inverse() * tcp;
        self.robot.inverse(&robot_transform)
    }

    fn inverse_continuing(&self, tcp: &Pose, previous: &Joints) -> Solutions {
        let robot_transform = self.base.inverse() * tcp;
        self.robot.inverse_continuing(&robot_transform, &previous)
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
// in this structure.
pub struct LinearAxis {
    robot: Box<dyn Kinematics>,
    axis: u32,
}

/// A platform for a robot that can ride in x, y and z directions. This way it is less
/// restricted than LinearAxis but tasks focusing with moving in one dimension only
/// may prefer abstracting which dimension it is.
pub struct Gantry {
    robot: Box<dyn Kinematics>,
}


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
        cart_translation * robot_pose
    }
}

impl Gantry {
    // Compute the forward transformation including the cart's offset and the robot's kinematics
    pub fn forward(&self, translation: &Translation3<f64>, joint_angles: &[f64; 6]) -> Isometry3<f64> {
        let robot_pose = self.robot.forward(joint_angles);
        translation * robot_pose
    }
}


#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use super::*;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};

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

    pub(crate) fn diff(robot_without: &dyn Kinematics, robot_with: &dyn Kinematics, joints: &[f64; 6]) -> (Pose, Pose) {
        let tcp_without_tool = robot_without.forward(&joints);
        let tcp_with_tool = robot_with.forward(&joints);
        (tcp_without_tool, tcp_with_tool)
    }

    #[test]
    fn test_tool() {
        // Parameters for Staubli TX40 robot are assumed to be correctly set in OPWKinematics::new
        let robot_without_tool = OPWKinematics::new(Parameters::staubli_tx2());

        // Tool extends 1 meter in the Z direction
        let tool_translation = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.0).into(),
            UnitQuaternion::identity(),
        );

        // Create the Tool instance with the transformation
        let robot_with_tool = Tool {
            robot: Box::new(robot_without_tool.clone()),
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
        let robot_without_base = OPWKinematics::new(Parameters::staubli_tx2());

        // Tool extends 1 meter in the Z direction
        let base_translation = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.0).into(),
            UnitQuaternion::identity(),
        );

        // Create the Tool instance with the transformation
        let robot_with_base = Base {
            robot: Box::new(robot_without_base.clone()),
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
        let robot_without_base = OPWKinematics::new(Parameters::staubli_tx2());
        let cart = LinearAxis {
            robot: Box::new(robot_without_base.clone()),
            axis: 1,
        };

        let joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let result = cart.forward(7.0, &joint_angles);

        assert_eq!(result.translation.vector.y,
                   robot_without_base.forward(&joint_angles).translation.vector.y + 7.0);
    }

    #[test]
    fn test_gantry_forward() {
        let robot_without_base = OPWKinematics::new(Parameters::staubli_tx2());
        let gantry = Gantry {
            robot: Box::new(robot_without_base.clone()),
        };

        let joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let r = gantry.forward(
            &Translation3::new(7.0, 8.0, 9.0),
            &joint_angles).translation;
        
        let alone = robot_without_base.forward(&joint_angles).translation;

        assert_eq!(r.x, alone.x + 7.0);
        assert_eq!(r.y, alone.y + 8.0);
        assert_eq!(r.z, alone.z + 9.0);
    }
}


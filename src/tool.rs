extern crate nalgebra as na;

use na::{Isometry3, Vector3, UnitQuaternion};
use crate::kinematic_traits::{Joints, Kinematics, Pose, Singularity, Solutions};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;


struct Tool {
    robot: Box<dyn Kinematics>,  // Using Box<dyn Kinematics> to handle trait objects.
    tool: Isometry3<f64>,        // Transformation from the robot's tip joint to the tool's TCP.
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

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use super::*;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};

    /// Asserts that two `Translation3<f64>` instances are approximately equal within a given tolerance.
    fn assert_diff(a: &Translation3<f64>, b: &Translation3<f64>, expected_diff: [f64; 3], epsilon: f64) {
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

    #[test]
    fn test_robot_with_and_without_tool_extension() {
        // Parameters for Staubli TX40 robot are assumed to be correctly set in OPWKinematics::new
        let robot_without_tool = OPWKinematics::new(Parameters::staubli_tx2());

        // Tool extends 1 meter in the Z direction
        let tool_translation = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 1.0).into(),
            UnitQuaternion::identity(),
        );

        fn diff(robot_without_tool: &dyn Kinematics, robot_with_tool: &dyn Kinematics, joints: &[f64; 6]) -> (Pose, Pose) {
            let tcp_without_tool = robot_without_tool.forward(&joints);
            let tcp_with_tool = robot_with_tool.forward(&joints);
            (tcp_without_tool, tcp_with_tool)
        }        

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
        let joints = [PI/2.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation, [0.0, 1.0, 0.], 1E-6);

        // Rotate base joint 45 degrees, must divide between X and Y
        let joints = [PI/4.0, 0.0, 0.0, 0.0, PI / 2.0, 0.0];
        let catet = 45.0_f64.to_radians().sin();
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation, 
                    [catet, catet, 0.], 1E-6);

        // Rotate base joint 45 degrees, must divide between X and Y, and also raise 45 deg up
        let joints = [PI/4.0, 0.0, 0.0, 0.0, PI / 4.0, 0.0];
        let (tcp_without_tool, tcp_with_tool) = diff(&robot_without_tool, &robot_with_tool, &joints);
        assert_diff(&tcp_with_tool.translation, &tcp_without_tool.translation,
                    [0.5, 0.5, 2.0_f64.sqrt() / 2.0], 1E-6);        
    }
}


fn main() {
    // Example usage:
    let robot = Box::new(OPWKinematics::new(Parameters::staubli_tx40()));
    let tool_translation = Isometry3::from_parts(
        Vector3::new(0.0, 0.0, 0.5).into(), UnitQuaternion::identity());
    let tool = Tool {
        robot,
        tool: tool_translation,
    };

    let joints = Joints::from([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
    let tool_pose = tool.forward(&joints);

    println!("Tool TCP Pose: {:?}", tool_pose);
}

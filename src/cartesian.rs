//! Cartesian stroke

use std::sync::Arc;
use crate::kinematic_traits::{Joints, Kinematics, Pose};
use crate::kinematics_with_shape::KinematicsWithShape;

/// Class doing Cartesian planning
struct Cartesian {
    /// Robot, that can be either KinematicsWithShape (for collision aware planning)
    /// or some kind of OPWKinematics (if collisions are not expected). 
    pub robot: Arc<dyn Kinematics>,

    /// Check step size in meters. Objects and features of the robotic cell smaller 
    /// than this may not be noticed during collision checks.
    pub check_step_m: f64,

    /// Check step size in radians. Objects and features of the robotic cell smaller 
    /// than this may not be noticed during collision checks.
    pub check_step_rad: f64,
}

impl Cartesian {
    /// Plans a Cartesian stroke
    ///
    /// # Description
    /// Thsi function translates the sequence of Cartesian poses into sequence of 
    /// joint configurations.
    ///
    /// In the main stroke, there is normally one joint configuration per provided pose.
    /// No additional poses are inserted. However the trajectory is checked in more places,
    /// making sure that any two poses checked are no more far apart as by check_step_m
    /// (for translation) and check_step_rad (for rotation). It is assumed, that the robot
    /// has the own Cartesian engine (that is now common) and will do Cartesian stroke between
    /// any two positions, regardless how far they are, by just providing some flag "this move
    /// is Cartesian". Most often it is beneficial for performance to do as long linear strokes
    /// as possible. 
    ///
    /// It this is not the case for your robot, provide much more dense
    /// stroke to plan, so that transition between adjacent poses would be approximately linear
    /// anyway. 
    ///
    /// The function begins from the initial position 'depart' that must be far away enough from
    /// various objects the robot could potentially collide with, into Cartesian position 'land'
    /// that may still be a little bit far away from the environment objects. The stroke itself
    /// is expected to be quite close to the surface. If you do not need such a care, pass
    /// 'None' for 'land' and 'park'. 
    ///
    /// Movements are generated to relocate the robot to the beginning of the Cartesian path, then 
    /// move to the path, then drive robot out of potentially close vicinity to the target into 
    /// the pose 'park'.  
    ///
    /// # Parameters
    /// - `depart`: The initial joint configuration (`Joints`) from which the motion will start.
    /// - `land`: The position still far enough from the potential obstacles.
    /// - `steps`: A vector of references to `Pose` objects representing the desired intermediate
    ///   Cartesian positions for the motion path. These poses will be checked for collision
    ///   along they linear path.
    /// - `park`: The final resting pose (`Pose`) that the robot will move to after completing the
    ///   Cartesian path defined by `steps`.
    ///
    /// # Returns
    /// Returns a vector of `Joints`, representing the joint configurations at each step of the Cartesian path,
    /// including the departure, intermediate steps, and park positions.
    ///
    /// # Note
    /// This function assumes a valid sequence of poses (`steps`) that are within the robot's reachable workspace
    /// and that the `depart` and `park` poses are feasible configurations for the robot.
    ///
    pub fn plan_cartesian(depart: &Joints, land: &Pose, steps: Vec<&Pose>, park: &Pose) -> Vec<Joints> {
        Vec::new()
    }

    pub fn generate_intermediate_poses(
        &self,
        start: &Pose,
        end: &Pose,
    ) -> Vec<Pose> {
        let mut poses = Vec::new();

        // Calculate the translation difference and distance
        let translation_diff = end.translation.vector - start.translation.vector;
        let translation_distance = translation_diff.norm();

        // Calculate the rotation difference and angle
        let rotation_diff = end.rotation * start.rotation.inverse();
        let rotation_angle = rotation_diff.angle();

        // Calculate the number of steps required for translation and rotation
        let translation_steps = (translation_distance / self.check_step_m).ceil() as usize;
        let rotation_steps = (rotation_angle / self.check_step_rad).ceil() as usize;

        // Choose the greater step count to achieve finer granularity between poses
        let steps = translation_steps.max(rotation_steps).max(1);

        // Calculate incremental translation and rotation per chosen step count
        let translation_step = translation_diff / steps as f64;

        // Generate each intermediate pose
        for i in 0..=steps {
            let fraction = i as f64 / steps as f64;

            // Interpolate translation and rotation
            let intermediate_translation = start.translation.vector + translation_step * i as f64;
            let intermediate_rotation = start.rotation.slerp(&end.rotation, fraction);

            // Construct the intermediate pose
            let intermediate_pose = Pose::from_parts(
                intermediate_translation.into(),
                intermediate_rotation,
            );

            poses.push(intermediate_pose);
        }

        poses
    }
}
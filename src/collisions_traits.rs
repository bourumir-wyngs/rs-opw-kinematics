use crate::joint_body::CollisionShape;
use crate::kinematic_traits::{Joints, Kinematics};

/// A trait for detecting collisions between a robot's joints and shapes.
///
/// The `CollisionDetector` trait provides two methods: one to check if any collisions occur,
/// and another to retrieve detailed information about which shapes are involved in collisions.
///
/// # Requirements
/// - Implementations of this trait must be `Send` and `Sync` for concurrent safety.
pub trait CollisionDetector: Send + Sync {
    /// Checks whether the given joint configuration results in any collisions.
    ///
    /// # Arguments
    /// * `qs` - A reference to the joint angles or positions (Joints) to be checked for collision.
    /// * `kinematics` - A reference to the kinematic model used to calculate the joint poses.
    ///
    /// # Returns
    /// * `true` if a collision occurs between any shapes in the robot, otherwise `false`.
    fn collides(&self, qs: &Joints, kinematics: &dyn Kinematics) -> bool;

    /// Provides detailed information about which shapes are involved in collisions.
    ///
    /// This method uses forward kinematics to compute the poses of all 6 joints and
    /// then performs collision detection. If any collisions are found, it returns
    /// a vector containing references to the pairs of `CollisionShape` instances that are in collision.
    /// Some algorithms may return only one pair of colliding joints even if multiple collide.
    ///
    /// # Arguments
    /// * `qs` - A reference to the joint angles or positions (Joints) for which to detect collisions.
    /// * `kinematics` - A reference to the kinematic model used to calculate the joint poses.
    ///
    /// # Returns
    /// * A vector of tuples where each tuple contains two references to `CollisionShape`s that are in collision.
    ///   If no collisions are detected, an empty vector is returned.
    fn collision_details(&self, qs: &Joints, kinematics: &dyn Kinematics) -> Vec<(&CollisionShape, &CollisionShape)>;
}

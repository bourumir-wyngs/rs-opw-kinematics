//! Defines the OPW parameter data structure

pub mod opw_kinematics {
    use crate::utils::deg;

    /// Parameters for the robot. See [parameters_robots.rs](parameters_robots.rs) for examples of concrete robot models.
    #[derive(Debug, Clone, Copy)]
    /// Parameters for the kinematic model of the robot.
    pub struct Parameters {
        /// The length of the first link of the robot (distance between joint 1 and joint 2).
        pub a1: f64,

        /// The length of the second link of the robot (distance between joint 2 and joint 3).
        pub a2: f64,

        /// The offset in the y-direction between joint 1 and joint 2.
        /// This can be 0 for robots without a lateral offset that is very common.
        pub b: f64,

        /// The vertical distance from the base (joint 1) to joint 2 along the z-axis.
        pub c1: f64,

        /// The vertical distance between joints 2 and 3 along the z-axis.
        pub c2: f64,

        /// The offset between joint 3 and joint 4, typically along the x-axis.
        pub c3: f64,

        /// The distance from the wrist center (last joint) to the end-effector mount
        /// In 5-DOF robots, this defines the offset between joint 5 and the end-effector mount
        pub c4: f64,

        /// Offsets applied to each joint angle to adjust the reference zero position.
        /// There are 6 values corresponding to each joint in a 6-DOF robot.
        pub offsets: [f64; 6],

        /// Specifies the direction of positive rotation from the zero angle for each joint.
        /// A value of `-1` reverses the default rotation direction for that joint.
        pub sign_corrections: [i8; 6],

        /// Degrees of freedom for the robot.
        /// This can either be 5 for 5-DOF robots or 6 for 6-DOF robots.
        pub dof: i8
    }


    impl Parameters {
        /// Convert to string yaml representation (quick viewing, etc).
        pub fn to_yaml(&self) -> String {
            format!(
                "opw_kinematics_geometric_parameters:\n  \
              a1: {}\n  \
              a2: {}\n  \
              b: {}\n  \
              c1: {}\n  \
              c2: {}\n  \
              c3: {}\n  \
              c4: {}\n\
            opw_kinematics_joint_offsets: [{}]\n\
            opw_kinematics_joint_sign_corrections: [{}]\n\
            dof: {}\n",
                self.a1,
                self.a2,
                self.b,
                self.c1,
                self.c2,
                self.c3,
                self.c4,
                self.offsets.iter().map(|x| deg(x))
                    .collect::<Vec<_>>().join(","),
                self.sign_corrections.iter().map(|x| x.to_string())
                    .collect::<Vec<_>>().join(","),
                self.dof
            )
        }        
    }
}

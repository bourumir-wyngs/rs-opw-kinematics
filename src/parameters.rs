//! Defines the OPW parameter data structure

pub mod opw_kinematics {
    use crate::utils::deg;

    /// Parameters for the robot. See parameters_robots.rs for examples for concrete robot models.
    #[derive(Debug, Clone)]
    pub struct Parameters {
        pub a1: f64,
        pub a2: f64,
        pub b: f64,
        pub c1: f64,
        pub c2: f64,
        pub c3: f64,
        pub c4: f64,
        pub offsets: [f64; 6],
        pub sign_corrections: [i8; 6],
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
            opw_kinematics_joint_sign_corrections: [{}]\n",
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
                    .collect::<Vec<_>>().join(",")
            )
        }        
    }
}

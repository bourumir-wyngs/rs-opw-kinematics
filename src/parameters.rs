pub mod opw_kinematics {

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
}

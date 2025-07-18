//! Hardcoded OPW parameters for a few robots

pub mod opw_kinematics {
    use crate::parameters::opw_kinematics::Parameters;
    use std::f64::consts::PI;

    #[allow(dead_code)]
    impl Parameters {
        // Provides default values
        pub fn new() -> Self {
            Parameters {
                a1: 0.0,
                a2: 0.0,
                b: 0.0,
                c1: 0.0,
                c2: 0.0,
                c3: 0.0,
                c4: 0.0,
                offsets: [0.0; 6],
                sign_corrections: [1; 6],
                dof: 6,
            }
        }

        pub fn igus_rebel() -> Self {
            Parameters {
                a1: 0.149,
                a2: -0.119,
                b: 0.,
                c1: 0.1,
                c2: 0.2384,
                c3: 0.17,
                c4: 0.1208,
                offsets: [0.0; 6],
                sign_corrections: [-1, 1, 1, 1, 1, 1],
                dof: 6,
            }
        }

        pub fn irb2400_10() -> Self {
            Parameters {
                a1: 0.100,
                a2: -0.135,
                b: 0.000,
                c1: 0.615,
                c2: 0.705,
                c3: 0.755,
                c4: 0.085,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                ..Self::new()
            }
        }

        // See https://www.staubli.com/content/dam/robotics/products/robots/tx2/TX2-140-160-datasheet-EN.pdf.
        // These three Staubli robots have spherical wrist and mostly identical plan, with only
        // two parameters being different. This function does not create usable parameters alone.
        fn staubli_tx2() -> Self {
            Parameters {
                a1: 0.150,
                a2: 0.000,
                b: 0.000, // axis aligned
                c1: 0.550,
                // c2: model specific
                // c3: model specific
                c4: 0.110,
                offsets: [0.0; 6],
                ..Self::new()
            }
        }
        pub fn staubli_tx2_140() -> Self {
            Parameters {
                c2: 0.625,
                c3: 0.625,
                ..Self::staubli_tx2()
            }
        }

        pub fn staubli_tx2_160() -> Self {
            Parameters {
                c2: 0.825,
                c3: 0.625,
                ..Self::staubli_tx2()
            }
        }

        pub fn staubli_tx2_160l() -> Self {
            Parameters {
                c2: 0.825,
                c3: 0.925,
                ..Self::staubli_tx2()
            }
        }

        pub fn fanuc_r2000ib_200r() -> Self {
            Parameters {
                a1: 0.720,
                a2: -0.225,
                b: 0.000,
                c1: 0.600,
                c2: 1.075,
                c3: 1.280,
                c4: 0.235,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                ..Self::new()
            }
        }

        pub fn kuka_kr6_r700_sixx() -> Self {
            Parameters {
                a1: 0.025,
                a2: -0.035,
                b: 0.000,
                c1: 0.400,
                c2: 0.315,
                c3: 0.365,
                c4: 0.080,
                offsets: [0.0, -PI / 2.0, 0.0, 0.0, 0.0, 0.0],
                sign_corrections: [-1, 1, 1, -1, 1, -1],
                ..Self::new()
            }
        }

        pub fn staubli_tx40() -> Self {
            Parameters {
                a1: 0.000,
                a2: 0.000,
                b: 0.035,
                c1: 0.320,
                c2: 0.225,
                c3: 0.225,
                c4: 0.065,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                ..Self::new()
            }
        }

        pub fn staubli_rx160() -> Self {
            Parameters {
                a1: 0.15,
                a2: 0.0,
                b: 0.0,
                c1: 0.55,
                c2: 0.825,
                c3: 0.625,
                c4: 0.11,
                ..Self::new()
            }
        }

        pub fn irb2600_12_165() -> Self {
            Parameters {
                a1: 0.150,
                a2: -0.115,
                b: 0.000,
                c1: 0.445,
                c2: 0.700,
                c3: 0.795,
                c4: 0.085,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                ..Self::new()
            }
        }

        pub fn irb4600_60_205() -> Self {
            Parameters {
                a1: 0.175,
                a2: -0.175,
                b: 0.000,
                c1: 0.495,
                c2: 0.900,
                c3: 0.960,
                c4: 0.135,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                ..Self::new()
            }
        }

        /// Corrected ABB IRB 1600-10/1.45 parameters based on ROS Industrial
        /// Reference: https://github.com/ros-industrial/abb/blob/noetic-devel/abb_irb1600_support/config/opw_parameters_irb1600_10_145.yaml
        pub fn abb_1600() -> Self {
            Parameters {
                a1: 0.150,  // Distance from base to J1 axis
                a2: 0.0,    // Distance from J1 to J2 axis (parallel offset)
                b: 0.0,     // Distance from J2 to J3 axis (perpendicular offset)
                c1: 0.4865, // Distance from base to J2 axis (height)
                c2: 0.700,  // Distance from J2 to J3 axis (upper arm length)
                c3: 0.600,  // Distance from J3 to J4 axis (forearm length)
                c4: 0.065,  // Distance from J4 to J6 axis (wrist length)
                offsets: [0.0, 0.0, -std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0],
                sign_corrections: [1, 1, 1, 1, 1, 1],
                dof: 6,
            }
        }
    }
}

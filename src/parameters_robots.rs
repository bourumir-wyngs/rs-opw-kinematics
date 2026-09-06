//! Built-in OPW parameters for industrial robots.
//!
//! Lengths are in metres and joint offsets are in radians. Model documentation
//! links to reference definitions with their joint and tool-frame conventions.
//! ROS-Industrial presets map `base_link` to `tool0` using ROS joint coordinates.

pub mod opw_kinematics {
    use crate::parameters::opw_kinematics::Parameters;
    use std::f64::consts::PI;

    #[allow(dead_code)]
    impl Default for Parameters {
        fn default() -> Self {
            Self::new()
        }
    }

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

        /// ABB IRB 120-3/0.58, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Derived from the [ROS-Industrial URDF](https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb120_support/urdf/irb120_3_58_macro.xacro).
        pub fn irb120_3_58() -> Self {
            Parameters {
                a1: 0.000,
                a2: -0.070,
                b: 0.000,
                c1: 0.290,
                c2: 0.270,
                c3: 0.302,
                c4: 0.072,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                sign_corrections: [1; 6],
                ..Self::new()
            }
        }

        /// ABB IRB 1200-5/0.90, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Derived from the [ROS-Industrial URDF](https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb1200_support/urdf/irb1200_5_90_macro.xacro).
        pub fn irb1200_5_90() -> Self {
            Parameters {
                a1: 0.000,
                a2: -0.042,
                b: 0.000,
                c1: 0.3991,
                c2: 0.448,
                c3: 0.451,
                c4: 0.082,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                sign_corrections: [1; 6],
                ..Self::new()
            }
        }

        /// ABB IRB 1200-7/0.70, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Derived from the [ROS-Industrial URDF](https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb1200_support/urdf/irb1200_7_70_macro.xacro).
        pub fn irb1200_7_70() -> Self {
            Parameters {
                a1: 0.000,
                a2: -0.042,
                b: 0.000,
                c1: 0.3991,
                c2: 0.350,
                c3: 0.351,
                c4: 0.082,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                sign_corrections: [1; 6],
                ..Self::new()
            }
        }

        /// ABB IRB 1600-6/1.20, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb1600_support/config/opw_parameters_irb1600_6_120.yaml).
        pub fn irb1600_6_120() -> Self {
            Parameters {
                a1: 0.150,
                a2: 0.000,
                b: 0.000,
                c1: 0.4865,
                c2: 0.475,
                c3: 0.600,
                c4: 0.065,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                sign_corrections: [1; 6],
                ..Self::new()
            }
        }

        /// ABB IRB 4600-40/2.55, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Derived from the [ROS-Industrial URDF](https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb4600_support/urdf/irb4600_40_255_macro.xacro).
        pub fn irb4600_40_255() -> Self {
            Parameters {
                a1: 0.175,
                a2: -0.175,
                b: 0.000,
                c1: 0.495,
                c2: 1.095,
                c3: 1.270,
                c4: 0.135,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, 0.0],
                sign_corrections: [1; 6],
                ..Self::new()
            }
        }

        /// FANUC LR Mate 200iB, using ROS-Industrial joint coordinates and the `tool0` frame.
        /// The input angle `joints[2]` is `controller_j3 + controller_j2`, in radians.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_lrmate200ib_support/config/opw_parameters_lrmate200ib.yaml).
        pub fn fanuc_lrmate200ib() -> Self {
            Parameters {
                a1: 0.150,
                a2: -0.075,
                b: 0.000,
                c1: 0.350,
                c2: 0.250,
                c3: 0.290,
                c4: 0.080,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, PI],
                sign_corrections: [1, 1, -1, -1, -1, -1],
                ..Self::new()
            }
        }

        /// FANUC M-6iB, using ROS-Industrial joint coordinates and the `tool0` frame.
        /// The input angle `joints[2]` is `controller_j3 + controller_j2`, in radians.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_m6ib_support/config/opw_parameters_m6ib.yaml).
        pub fn fanuc_m6ib() -> Self {
            Parameters {
                a1: 0.150,
                a2: -0.100,
                b: 0.000,
                c1: 0.450,
                c2: 0.600,
                c3: 0.615,
                c4: 0.100,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, PI],
                sign_corrections: [1, 1, -1, -1, -1, -1],
                ..Self::new()
            }
        }

        /// FANUC M-10iA, using ROS-Industrial joint coordinates and the `tool0` frame.
        /// The input angle `joints[2]` is `controller_j3 + controller_j2`, in radians.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_m10ia_support/config/opw_parameters_m10ia.yaml).
        pub fn fanuc_m10ia() -> Self {
            Parameters {
                a1: 0.150,
                a2: -0.200,
                b: 0.000,
                c1: 0.450,
                c2: 0.600,
                c3: 0.640,
                c4: 0.100,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, PI],
                sign_corrections: [1, 1, -1, -1, -1, -1],
                ..Self::new()
            }
        }

        /// FANUC M-16iB/20, using ROS-Industrial joint coordinates and the `tool0` frame.
        /// The input angle `joints[2]` is `controller_j3 + controller_j2`, in radians.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_m16ib_support/config/opw_parameters_m16ib20.yaml).
        pub fn fanuc_m16ib20() -> Self {
            Parameters {
                a1: 0.150,
                a2: -0.100,
                b: 0.000,
                c1: 0.525,
                c2: 0.770,
                c3: 0.740,
                c4: 0.100,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, PI],
                sign_corrections: [1, 1, -1, -1, -1, -1],
                ..Self::new()
            }
        }

        /// FANUC M-20iA, using ROS-Industrial joint coordinates and the `tool0` frame.
        /// The input angle `joints[2]` is `controller_j3 + controller_j2`, in radians.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_m20ia_support/config/opw_parameters_m20ia.yaml).
        pub fn fanuc_m20ia() -> Self {
            Parameters {
                a1: 0.150,
                a2: -0.250,
                b: 0.000,
                c1: 0.525,
                c2: 0.790,
                c3: 0.835,
                c4: 0.100,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, PI],
                sign_corrections: [1, 1, -1, -1, -1, -1],
                ..Self::new()
            }
        }

        /// FANUC M-20iB/25, using ROS-Industrial joint coordinates and the `tool0` frame.
        /// The input angle `joints[2]` is `controller_j3 + controller_j2`, in radians.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_m20ib_support/config/opw_parameters_m20ib25.yaml).
        pub fn fanuc_m20ib25() -> Self {
            Parameters {
                a1: 0.075,
                a2: -0.120,
                b: 0.000,
                c1: 0.650,
                c2: 0.905,
                c3: 0.865,
                c4: 0.100,
                offsets: [0.0, 0.0, -PI / 2.0, 0.0, 0.0, PI],
                sign_corrections: [1, 1, -1, -1, -1, -1],
                ..Self::new()
            }
        }

        /// KUKA KR 6 R900-2, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/kuka_experimental/blob/8d9292b04a22628b1b78d989e2ddd3abb913bf92/kuka_kr6_support/config/opw_parameters_kr6r900_2.yaml).
        pub fn kuka_kr6_r900_2() -> Self {
            Parameters {
                a1: 0.025,
                a2: -0.025,
                b: 0.000,
                c1: 0.400,
                c2: 0.455,
                c3: 0.420,
                c4: 0.090,
                offsets: [0.0, -PI / 2.0, 0.0, 0.0, 0.0, 0.0],
                sign_corrections: [-1, 1, 1, -1, 1, -1],
                ..Self::new()
            }
        }

        /// KUKA KR 10 R1420, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/kuka_experimental/blob/8d9292b04a22628b1b78d989e2ddd3abb913bf92/kuka_kr10_support/config/opw_parameters_kr10r1420.yaml).
        pub fn kuka_kr10_r1420() -> Self {
            Parameters {
                a1: 0.150,
                a2: -0.020,
                b: 0.000,
                c1: 0.450,
                c2: 0.610,
                c3: 0.660,
                c4: 0.080,
                offsets: [0.0, -PI / 2.0, 0.0, 0.0, 0.0, 0.0],
                sign_corrections: [-1, 1, 1, -1, 1, -1],
                ..Self::new()
            }
        }

        /// KUKA KR 150 R3100-2, using ROS-Industrial joint coordinates and the `tool0` frame.
        ///
        /// Source: the [ROS-Industrial OPW parameters](https://github.com/ros-industrial/kuka_experimental/blob/8d9292b04a22628b1b78d989e2ddd3abb913bf92/kuka_kr150_support/config/opw_parameters_kr150r3100_2.yaml).
        pub fn kuka_kr150_r3100_2() -> Self {
            Parameters {
                a1: 0.330,
                a2: -0.115,
                b: 0.000,
                c1: 0.645,
                c2: 1.350,
                c3: 1.420,
                c4: 0.215,
                offsets: [0.0, -PI / 2.0, 0.0, 0.0, 0.0, 0.0],
                sign_corrections: [-1, 1, 1, -1, 1, -1],
                ..Self::new()
            }
        }

        /// Corrected ABB IRB 1600-10/1.45 parameters based on ROS Industrial
        /// Reference: <https://github.com/ros-industrial/abb/blob/noetic-devel/abb_irb1600_support/config/opw_parameters_irb1600_10_145.yaml>
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

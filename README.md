Rust implementation of inverse and forward kinematic solutions for six-axis industrial robots with a parallel base
and spherical wrist. Hardened against the J5 = 0&deg; or &plusmn; 180&deg; singularity and optimized for trajectory planning.

[![GitHub](https://img.shields.io/badge/GitHub-777777)](https://github.com/bourumir-wyngs/rs-opw-kinematics)
[![crates.io](https://img.shields.io/crates/v/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/bourumir-wyngs/rs-opw-kinematics/rust.yml)](https://github.com/bourumir-wyngs/rs-opw-kinematics/actions)
[![crates.io](https://img.shields.io/crates/l/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![crates.io](https://img.shields.io/crates/d/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![docs.rs](https://docs.rs/rs-opw-kinematics/badge.svg)](https://docs.rs/rs-opw-kinematics)

# Intro

This work is based on the 2014 year paper `An Analytical Solution of the Inverse Kinematics Problem
of Industrial Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist` by
Mathias Brandstötter, Arthur Angerer, and Michael Hofbaur. It is also inspired by the similar
C++ project [Jmeyer1292/opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics) (that was used as a reference
implementation to generate data for the test suite; also, this documentation uses the robot diagram from there).
This paper can be found
[here](https://www.researchgate.net/profile/Mathias-Brandstoetter/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist/links/53d2417e0cf2a7fbb2e98b09/An-Analytical-Solution-of-the-Inverse-Kinematics-Problem-of-Industrial-Serial-Manipulators-with-an-Ortho-parallel-Basis-and-a-Spherical-Wrist.pdf).

# Features
- rs-opw-kinematics is written entirely in Rust (not a C++ binding) and could be deployed using Cargo.
- all returned solutions are valid, normalized, and cross-checked with forward kinematics.
- to generate a trajectory of the robot (sequence of poses), it is possible to use "previous joint positions" as additional input.
- if the previous joint positions are provided, the solutions are sorted by proximity to them (closest first)
- for kinematic singularity at J5 = 0&deg; or J5 = &plusmn;180&deg; positions this solver provides reasonable J4 and J6
  values close to the previous positions of these joints (and not arbitrary that may result in a large jerk of the real robot)
- use zeros to get the possible solution of singularity case with J4 and J6 close to zero rotation.
- The solver currently uses 64-bit floats (Rust f64), providing the positional accuracy below 1&micro;m for
  the two robots tested.

# Parameters

This library uses seven kinematic parameters (a1, a2, b, c1, c2, c3, and c4). This solver assumes that the arm is
at zero when all joints stick straight up in the air, as seen in the image below. It also assumes that all
rotations are positive about the base axis of the robot. No other setup is required.

![OPW Diagram](documentation/opw.gif)

To use the library, fill out an `opw_kinematics::Parameters` data structure with the appropriate values for the 7
kinematic parameters and any joint offsets required to bring the paper's zero position (arm up in Z) to the
manufacturer's position. Additionally, there are 6 "sign correction" parameters (-1 or 1) that should be specified if
your robot's axes do not match the convention in the paper.

For example, the ABB IRB2400 has the following values:

```Rust
let parameters = Parameters {
    a1: 0.100,
    a2: - 0.135,
    b: 0.000,
    c1: 0.615,
    c2: 0.705,
    c3: 0.755,
    c4: 0.085,
    offsets: [0.0, 0.0, -std::f64::consts::PI / 2.0, 0.0, 0.0, 0.0],
    sign_corrections: [1; 6],
}
``` 

Note that the offset of the third joint is -90 degrees, bringing the joint from the upright position to parallel with
the ground at "zero."

# Example
Cargo.toml:
```toml
[dependencies]
rs-opw-kinematics = "1.0.2"
```

main.rs:

```Rust
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, JOINTS_AT_ZERO};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};

fn main() {
  let robot = OPWKinematics::new(Parameters::irb2400_10());
  let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; // Joints are alias of [f64; 6]
  println!("Initial joints with singularity J5 = 0: ");
  dump_joints(&joints);

  println!("Solutions (original angle set is lacking due singularity there: ");
  let pose: Pose = robot.forward(&joints); // Pose is alias of nalgebra::Isometry3<f64>

  let solutions = robot.inverse(&pose); // Solutions is alias of Vec<Joints>
  dump_solutions(&solutions);

  println!("Solutions assuming we continue from somewhere close. The 'lost solution' returns");
  let when_continuing_from: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5];
  let solutions = robot.inverse_continuing(&pose, &when_continuing_from);
  dump_solutions(&solutions);

  println!("Same pose, all J4+J6 rotation assumed to be previously concentrated on J4 only");
  let when_continuing_from_j6_0: [f64; 6] = [0.0, 0.11, 0.22, 0.8, 0.1, 0.0];
  let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
  dump_solutions(&solutions);

  println!("If we do not have the previous position, we can assume we want J4, J6 close to 0.0");
  println!("The solution appears and the needed rotation is now equally distributed between J4 and J6.");
  let solutions = robot.inverse_continuing(&pose, &JOINTS_AT_ZERO);
  dump_solutions(&solutions);
}
```

# Configuring the solver for your robot
The project contains built-in definitions for ABB IRB 2400/10, IRB 2600-12/1.65, RB 4600-60/2.05; KUKA KR 6 R700 sixx, 
FANUC R-2000iB/200R; Stäubli TX40, TX2-140, TX2-160 and TX2-160L with various levels of
testing. Robot manufacturers may provide such configurations for the robots they make.
For instance, FANUC M10IA is described [here](https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m10ia_support/config/opw_parameters_m10ia.yaml).
Many other robots are described in [ros-industrial/fanuc](https://github.com/ros-industrial/fanuc) repository.
This project contains the code for reading such configurations directly, including support for *deg(angle)*
function that sometimes occurs there.

Is it possible to read YAML parameter files directly, including parsing of the deg(angle)
function that sometimes occurs there.

```Rust
  let parameters = Parameters::from_yaml_file(filename).expect("Failed to load parameters from file");
  let robot = OPWKinematics::new(parameters);
```

# Testing
The code of this project is tested against the test set (cases.yaml, 2048 cases per robot) that is
believed to be correct for the two robots, KUKA KR 6 R700 sixx and ABB IRB 2400/10. It has been produced
using independent C++ implementation by [Jmeyer1292/opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics). The testing suite checks if the solutions
match.



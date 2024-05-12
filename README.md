Rust implementation of inverse and forward kinematic solutions for six-axis industrial robots with a parallel base
and spherical wrist. Hardened against the J5 = 0&deg; or &plusmn; 180&deg; singularity and optimized for trajectory
planning.

[![GitHub](https://img.shields.io/badge/GitHub-777777)](https://github.com/bourumir-wyngs/rs-opw-kinematics)
[![crates.io](https://img.shields.io/crates/v/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/bourumir-wyngs/rs-opw-kinematics/rust.yml)](https://github.com/bourumir-wyngs/rs-opw-kinematics/actions)
[![crates.io](https://img.shields.io/crates/l/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![crates.io](https://img.shields.io/crates/d/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![docs.rs](https://docs.rs/rs-opw-kinematics/badge.svg)](https://docs.rs/rs-opw-kinematics)

# Intro

This work builds upon the 2014 paper titled _An Analytical Solution of the Inverse Kinematics Problem of Industrial
Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist_, authored by Mathias Brandstötter, Arthur
Angerer, and Michael Hofbaur. The paper is [available in ResearchGate](https://www.researchgate.net/profile/Mathias-Brandstoetter/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist/links/53d2417e0cf2a7fbb2e98b09/An-Analytical-Solution-of-the-Inverse-Kinematics-Problem-of-Industrial-Serial-Manipulators-with-an-Ortho-parallel-Basis-and-a-Spherical-Wrist.pdf)
. Additionally, it draws inspiration from the similar C++
project, [Jmeyer1292/opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics), which served as a reference implementation for generating data for the test suite.
This documentation also incorporates the robot diagram from that project.

# Features

- rs-opw-kinematics is written entirely in Rust (not a C++ binding) and deployable via Cargo.
- All returned solutions are valid, normalized, and cross-checked with forward kinematics.
- Joint angles can be checked against constraints, ensuring only compliant solutions are returned.
- To generate a trajectory of the robot (sequence of poses), it is possible to use "previous joint positions" as
  additional input.
- If the previous joint positions are provided, the solutions are sorted by proximity to them (closest first).
  It is also possible to prioritize proximity to the center of constraints.
- For kinematic singularity at J5 = 0&deg; or J5 = &plusmn;180&deg; positions this solver provides reasonable J4 and J6
  values close to the previous positions of these joints (and not arbitrary that may result in a large jerk of the real
  robot)
- OPW parameters can be automatically exctracted form URDF or XACRO content.
 
The solver currently uses 64-bit floats (Rust f64), providing the positional accuracy below 1&micro;m for the two 
robots tested.

# Parameters

This library uses seven kinematic parameters (_a1, a2, b, c1, c2, c3_, and _c4_). This solver assumes that the arm is
at zero when all joints stick straight up in the air, as seen in the image below. It also assumes that all
rotations are positive about the base axis of the robot. No other setup is required.

![OPW Diagram](https://bourumir-wyngs.github.io/rs-opw-kinematics/documentation/opw.gif)
<!-- ![OPW Diagram](documentation/opw.gif) -->

To use the library, fill out an `opw_kinematics::Parameters` data structure with the appropriate values for the 7
kinematic parameters and any joint offsets required to bring the paper's zero position (arm up in Z) to the
manufacturer's position. Additionally, there are 6 "sign correction" parameters (-1 or 1) that should be specified if
your robot's axes do not match the convention in the paper.

For example, the ABB IRB2400 has the following values:

```Rust
let parameters = Parameters {
  a1: 0.100, a2: - 0.135, b: 0.000, c1: 0.615, c2: 0.705, c3: 0.755, c4: 0.085,
  offsets: [0.0, 0.0, -std::f64::consts::PI / 2.0, 0.0, 0.0, 0.0],
  sign_corrections: [1; 6],
}
``` 

Note that the offset of the third joint is -90&deg;, bringing the joint from the upright position to parallel with
the ground at "zero."

# Constraints

Since 1.1.0, it is possible to set constraints for the joints. Robot poses where any of the joints are outside
the specified constraint range are not included into returned list of solutions. It is also possible to
influence the sorting of the result list by giving some preference to the center of constraints.

Constraints are specified by providing two angles, _from_ and to, for every _joint_. If _from_ < _to_, the valid range
spans between from and to. If _from_ > _to_, the valid range spans over the 0&deg;, wrapping around. For instance,
if _from_ = 5&deg; and _to_ = 15&deg;, values 6&deg;, 8&deg;, and 11&deg; are valid, while values like 90&deg;, and 
180&deg; are not. If _from_ = 15&deg; and _to_ = 5&deg; (the opposite), values 16&deg;, 17&deg;, 100&deg;, 180&deg;,
359&deg;, 0&deg;, 1&deg;, 3&deg;, 4&deg; are valid, while 6&deg;, 8&deg;, and 11&deg; are not.

Constraints are tested for the range from -2&pi; to 2&pi;, but as angles repeat with period of 2&pi;, the
constraint from -&pi; to &pi; already permits free rotation, covering any angle.

# Example

Cargo.toml:

```toml
[dependencies]
rs-opw-kinematics = ">=1.1.1, <2.0.0" 
```

main.rs:

```Rust
use std::f64::consts::PI;
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, 
                                          JOINTS_AT_ZERO, CONSTRAINT_CENTERED};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};
use rs_opw_kinematics::constraints::{BY_CONSTRAINS, BY_PREV, Constraints};

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

  println!("If we do not have the previous position, we can assume we want J4, J6 close to 0.0 \
    The solution appears and the needed rotation is now equally distributed between J4 and J6.");
  let solutions = robot.inverse_continuing(&pose, &JOINTS_AT_ZERO);
  dump_solutions(&solutions);

  let robot = OPWKinematics::new_with_constraints(
    Parameters::irb2400_10(), Constraints::new(
      [-0.1, 0.0, 0.0, 0.0, -PI, -PI],
      [ PI, PI, 2.0*PI, PI, PI, PI],
      BY_PREV,
    ));

  println!("If we do not have the previous pose yet, we can now ask to prefer the pose \
    closer to the center of constraints.");
  let solutions = robot.inverse_continuing(&pose, &CONSTRAINT_CENTERED);
  dump_solutions(&solutions);

  println!("With constraints, sorted by proximity to the previous pose");
  let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
  dump_solutions(&solutions);

  let robot = OPWKinematics::new_with_constraints(
    Parameters::irb2400_10(), Constraints::new(
      [-0.1, 0.0, 0.0, 0.0, -PI, -PI],
      [ PI, PI, 2.0*PI, PI, PI, PI],
      BY_CONSTRAINS,
    ));
  println!("With constraints, sorted by proximity to the center of constraints");
  let solutions = robot.inverse_continuing(&pose, &when_continuing_from_j6_0);
  dump_solutions(&solutions);
}
```

The constants _BY_PREV_ ( = 0.0) and _BY_CONSTRAINTS_ ( = 1.0) are for convenience only. Intermediate values like
0.6 can also be specified and result in weighted sorting.

# Configuring the solver for your robot

The project contains built-in definitions for ABB IRB 2400/10, IRB 2600-12/1.65, IRB 4600-60/2.05; KUKA KR 6 R700 sixx,
FANUC R-2000iB/200R; Stäubli TX40, TX2-140, TX2-160 and TX2-160L with various levels of
testing. Robot manufacturers may provide such configurations for the robots they make.
For instance, FANUC M10IA is
described [here](https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m10ia_support/config/opw_parameters_m10ia.yaml).
Many other robots are described in [ros-industrial/fanuc](https://github.com/ros-industrial/fanuc) repository.
This project contains the code for reading such configurations directly, including support for *deg(angle)*
function that sometimes occurs there:

```Rust
  let parameters = Parameters::from_yaml_file(filename).expect("Failed to load parameters");
let robot = OPWKinematics::new(parameters);
```

Since version 1.1.2, parameters can also be directly extracted from URDF file or XACRO fragment:
```Rust
let robot = rs_opw_kinematics::urdf::from_urdf_file("/path/to/robot.urdf");
```

Both robot parameters and constraints are extracted. This example shows the "user friendly" version. See 
documentation for ```rs_opw_kinematics::urdf::from_urdf``` that takes URDF string rather than the file,
provides error handling and much more control over how the solver is constructed from the extracted values.

# Testing

The code of this project is tested against the test set (cases.yaml, 2048 cases per robot) that is
believed to be correct for the two robots, KUKA KR 6 R700 sixx and ABB IRB 2400/10. It has been produced
using independent C++ implementation by [Jmeyer1292/opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics). The
testing suite checks if the solutions
match.



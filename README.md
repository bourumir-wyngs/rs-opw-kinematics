Rust implementation of inverse and forward kinematic solutions for six-axis industrial robots with a parallel base
and spherical wrist. Hardened against the J5 = 0&deg; or &plusmn; 180&deg; singularity and optimized for trajectory
planning.

[![GitHub](https://img.shields.io/badge/GitHub-777777)](https://github.com/bourumir-wyngs/rs-opw-kinematics)
[![crates.io](https://img.shields.io/crates/v/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/bourumir-wyngs/rs-opw-kinematics/rust.yml)](https://github.com/bourumir-wyngs/rs-opw-kinematics/actions)
[![crates.io](https://img.shields.io/crates/l/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![crates.io](https://img.shields.io/crates/d/rs-opw-kinematics.svg)](https://crates.io/crates/rs-opw-kinematics)
[![docs.rs](https://docs.rs/rs-opw-kinematics/badge.svg)](https://docs.rs/rs-opw-kinematics)
[![Dependency Vulnerabilities](https://img.shields.io/endpoint?url=https%3A%2F%2Fapi-hooks.soos.io%2Fapi%2Fshieldsio-badges%3FbadgeType%3DDependencyVulnerabilities%26pid%3D3xxqf0020%26)](https://app.soos.io)

<img src="https://github.com/user-attachments/assets/64cf952a-93b0-4a69-ba6f-d3e98b1cee25" alt="screenshot" width="300"/>

See also [video produced with RViz under ROS2](https://www.youtube.com/watch?v=CVZ9CFt_QMM)

# Intro

This work builds upon the 2014 paper titled _An Analytical Solution of the Inverse Kinematics Problem of Industrial
Serial Manipulators with an Ortho-parallel Basis and a Spherical Wrist_, authored by Mathias Brandstötter, Arthur
Angerer, and Michael Hofbaur. The paper is [available in ResearchGate](https://www.researchgate.net/profile/Mathias-Brandstoetter/publication/264212870_An_Analytical_Solution_of_the_Inverse_Kinematics_Problem_of_Industrial_Serial_Manipulators_with_an_Ortho-parallel_Basis_and_a_Spherical_Wrist/links/53d2417e0cf2a7fbb2e98b09/An-Analytical-Solution-of-the-Inverse-Kinematics-Problem-of-Industrial-Serial-Manipulators-with-an-Ortho-parallel-Basis-and-a-Spherical-Wrist.pdf). Additionally, it draws inspiration from 
the similar C++ project, [Jmeyer1292/opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics), which served as a reference implementation for generating 
data for the test suite. This documentation also incorporates the robot diagram from that project.

# Features

- rs-opw-kinematics is written entirely in Rust (not a C++ binding) and deployable via Cargo.
- All returned solutions are valid, normalized, and cross-checked with forward kinematics.
- Joint angles can be checked against constraints, ensuring only compliant solutions are returned.
- Collision detection (with [Parry](https://parry.rs/)) allows excluding solutions where the robot would collide with
  itself or environment objects. It is possible to set guaranteed safety distances between surfaces rather than
  just checking if they touch.
- For kinematic singularity at J5 = 0&deg; or J5 = &plusmn;180&deg; positions this solver provides reasonable J4 and J6
  values close to the previous positions of these joints (and not arbitrary that may result in a large jerk of the real
  robot). Since 1.8.9, the "previous" rotation can be in a wide range well outside &plusmn;360&deg;
- The robot can be equipped with the tool and placed on the base, planning for the desired location and orientation
  of the tool center point (TCP) rather than any part of the robot. 
- Planning a Cartesian stroke composed of linear segments, ensuring configuration consistency (no abrupt jumps) and collision-free movement. Alternative methods for executing the stroke are being explored, transitioning from the specified "onboarding" robot configuration to the first waypoint before the linear stroke.
- Jacobian, torques and velocities
- 5 DOF inverse kinematics.
- Visualization (with [Bevy](https://bevyengine.org/)) allows quick check if the robot is properly configured.


The solver currently uses 64-bit floats (Rust f64), providing the positional accuracy below 1&micro;m for the two
robots tested.

# Quick example

Cargo.toml:

```toml
[dependencies]
rs-opw-kinematics = ">=1.8.9, <2.0.0"
```

Simple "hello world" demonstrating singularity evasion would look more or less like this:

```Rust
use rs_opw_kinematics::kinematic_traits::{Joints, Kinematics, Pose, JOINTS_AT_ZERO};
use rs_opw_kinematics::kinematics_impl::OPWKinematics;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::utils::{dump_joints, dump_solutions};

fn main() {
    // Create a robot with built-in parameter set. It is a simple Kinematics with no collision checks.
    let robot = OPWKinematics::new(Parameters::irb2400_10());
    let joints: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.5]; // Joints are alias of [f64; 6], given in radians here
    println!("\nInitial joints with singularity J5 = 0: ");
    dump_joints(&joints);

    println!("\nSolutions assuming we continue from somewhere close. No singularity effect.");
    // Some previous pose for picking the best solution when infinite number of these exist.
    let when_continuing_from: [f64; 6] = [0.0, 0.11, 0.22, 0.3, 0.1, 0.5]; 
    let solutions = robot.inverse_continuing(&pose, &when_continuing_from);
    dump_solutions(&solutions);
}
```
Since version 1.8.9, the "previous" angles can be very large (including negative values). Test cases cover angles up to 90,000 degrees.

The project rs-opw-kinematics has now evolved beyond being just a set of "useful building blocks." It now
enables the creation of a complete robot setup, which includes mounting the robot on a base, equipping it with a tool,
integrating collision checking and both joint-based and Cartesian path planning with collision avoidance. 
See example [complete_visible_robot](examples/constraints.rs).

## Parameters

This library uses seven kinematic parameters (_a1, a2, b, c1, c2, c3_, and _c4_). This solver assumes that the arm is
at zero when all joints stick straight up in the air, as seen in the image below. It also assumes that all
rotations are positive about the base axis of the robot. No other setup is required.

<img src="https://camo.githubusercontent.com/a60affbc3f6b93896f6e3c46e320ec0d36eb22b81c85cf8242dc0e315147c0ec/68747470733a2f2f626f7572756d69722d77796e67732e6769746875622e696f2f72732d6f70772d6b696e656d61746963732f646f63756d656e746174696f6e2f6f70772e676966" alt="OPW Kinematics GIF" width="300"/>

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

If you have the OPW robot and not sure how to configure it for this tool, contact
<a href="mailto:&#98;&#111;&#117;&#114;&#117;&#109;&#105;&#114;&#46;&#119;&#121;&#110;&#103;&#115;&#64;&#103;&#109;&#97;&#105;&#108;&#46;&#99;&#111;&#109;">&#98;&#111;&#117;&#114;&#117;&#109;&#105;&#114;&#46;&#119;&#121;&#110;&#103;&#115;&#64;&#103;&#109;&#97;&#105;&#108;&#46;&#99;&#111;&#109;</a> and we will help with integration.  

## Constraints

Since 1.1.0, it is possible to
set [constraints](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/constraints/index.html) for the joints.
Robot poses where any of the joints are outside
the specified constraint range are not included in the returned list of solutions. It is also possible to
influence the sorting of the result list by giving some preference to the center of constraints.

Constraints are specified by providing two angles, _from_ and to, for every _joint_. If _from_ < _to_, the valid range
spans between from and to. If _from_ > _to_, the valid range spans over the 0&deg;, wrapping around. For instance,
if _from_ = 5&deg; and _to_ = 15&deg;, values 6&deg;, 8&deg;, and 11&deg; are valid, while values like 90&deg;, and
180&deg; are not. If _from_ = 15&deg; and _to_ = 5&deg; (the opposite), values 16&deg;, 17&deg;, 100&deg;, 180&deg;,
359&deg;, 0&deg;, 1&deg;, 3&deg;, 4&deg; are valid, while 6&deg;, 8&deg;, and 11&deg; are not.

Constraints are tested for the range from -2&pi; to 2&pi;, but as angles repeat with the period of 2&pi;, the
constraint from -&pi; to &pi; already permits free rotation, covering any angle.

Since 1.8.2, convenience method exists to specify constraints as ranges in degrees.

Please see the [example](examples/constraints.rs).

## Jacobian: torques and velocities

Since 1.3.2, it is possible to obtain
the [Jacobian](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/jacobian/struct.Jacobian.html) that represents
the relationship between the joint velocities
and the end-effector velocities. The computed Jacobian object provides:

- Joint [velocities](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/jacobian/struct.Jacobian.html#method.velocities) required to achieve a desired end-effector velocity.
- Joint [torques](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/jacobian/struct.Jacobian.html#method.torques) required to achieve a desired end-effector force/torque.

The same Joints structure is reused, the six values now representing either angular velocities in radians per second
or torques in Newton meters. For the end effector, it is possible to use either
nalgebra::[Isometry3](https://docs.rs/nalgebra/latest/nalgebra/geometry/type.Isometry3.html)
or [Vector6](https://docs.rs/nalgebra/latest/nalgebra/base/type.Vector6.html), both defining velocities in m/s or
rotations in N m.

These values are useful when path planning for a robot that needs to move very swiftly, to prevent
overspeed or overtorque of individual joints.

Please see the [example](examples/jacobian.rs).

## The tool and the base

Since 1.3.2, robot can be equipped with
the [tool](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/tool/struct.Tool.html), defined as
nalgebra::[Isometry3](https://docs.rs/nalgebra/latest/nalgebra/geometry/type.Isometry3.html). The tool isometry defines
both
additional translation and additional rotation. The "pose" as defined in forward and inverse kinematics
now becomes the pose of the tool center point, not any part of the robot. The robot can also be placed
on a [base](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/tool/struct.Base.html), further supporting the conditions much closer to the real industrial environment.

"Robot with the tool" and "Robot on the base" can be constructed around
any [Kinematics](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/kinematic_traits/trait.Kinematics.html)
trait, and implement
this trait themselves. It is possible to cascade them, constructing a robot on a base and with the tool (or
two tools if the first is a receptacle of the tool changer).

Please see the [example](examples/tool_and_base.rs).

## The frame

Since 1.8.2 this package supports the frame transform that allows to transform the robot trajectory (in terms of joint
angles)
prepared for one location to make the same kind of movements in another location (translated and rotated).
Frame in robotics is most commonly defined by the 3 pairs of points (to and from) if the transform includes
also rotation, or just a single pair is enough if only shift (but not a rotation) is involved.

Once constructed by specifying original and transformed points, the Frame object can take "canonical" joint angles
and calculated joint angles for the transformed (shifted and rotated) trajectory. See the
[frame](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/frame/index.html) documentation and [example](examples/frame.rs) for details.

## Individual link positions

It is now possible to obtain positions of individual links in forward kinematics. This would be needed for
collision avoidance and graphical rendering of the robot.
See [forward_with_joint_poses](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/kinematic_traits/trait.Kinematics.html#tymethod.forward_with_joint_poses)
method.

## 5 DOF inverse kinematics

For tools that are not sensitive to axis rotation (such as welding torches or paint sprayers), inverse kinematics can be
requested where the value of joint 6 (which typically controls this rotation) is either inherited from the previous
position or explicitly specified.

The 5 DOF robot can still be represented with the same diagram, and has the same parameters. However, joint 6 is assumed
to be fixed. Such a robot still can bring the tool to the needed location, also following the generic orientation
but the rotation around the tool axis is not followed.

Support for 5 DOF robots is now included through an additional 'dof' field in the
parameter data structure. 5 DOF inverse kinematics can also be requested for 6 DOF
robots, particularly when the last joint is in constant motion (e.g., for drilling), or when maintaining precise tool
rotation would cause the robot to exceed its constraints. This method is also faster to compute. If the robot is
flagged as 5 DOF robot, the value of the joint 6 will normally be 0 and ignored.

## Parallelogram

The parallelogram mechanism maintains the orientation of the end-effector in
some robots. It introduces a geometric relationship between two joints,
typically referred to as the _driven joint_ (often J₂) and the _coupled joint_
(often J₃), to ensure the end-effector remains stable in its orientation during
motion.

In forward kinematics, J₃ is adjusted by subtracting the value of J₂ multiplied
by a scaling factor `s` (often 1.0). The relationship can be written as:

J₃' = J₃ − s * J₂

This adjustment maintains the correct orientation of the end-effector as the
robot moves through its workspace.

In inverse kinematics, the process is reversed. The value of J₂ is added back
to J₃, ensuring accurate joint angle calculations for achieving the desired
end-effector pose and orientation. This can be expressed as:

J₃' = J₃ + s * J₂

The scaling factor `s` determines how much influence J₂ has on J₃. A scaling
factor of 1.0 is the common, as this value ensures the end-effector’s
orientation remains unchanged if only J₃ and J₂ move.

See [Parallelogram](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/parallelogram/struct.Parallelogram.html) and [example](examples/parallelogram.rs).

## Collision avoidance
The new class 
[KinematicsWithShape](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/kinematics_with_shape/struct.KinematicsWithShape.html) 
combines kinematics and collision checking. It implements the 
[Kinematics](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/kinematic_traits/trait.Kinematics.html) trait, 
providing both forward and inverse kinematic solutions. During inverse kinematics, any colliding poses are excluded 
from the solution list.

For collision avoidance, you need to supply meshes for robot joints and, optionally, for the base, tool, 
and environment objects.

Starting with version 1.8.0, it is now possible to configure a safety distance — the minimum distance to a surface 
below which a collision is detected. In most real-world scenarios, a robot must maintain a guaranteed clearance from 
objects it might collide with (including its own parts) rather than simply avoiding surface contact.

If the safety distance is set to zero, collisions are still likely due to the inherent limitations in system accuracy. 
Setting a zero safety distance is only advisable if the meshes are "inflated" (made larger than their actual size), 
but achieving this reliably is challenging and requires careful validation. On the other hand, checking for contact 
rather than maintaining a safety distance is significantly faster.

Safety distances are controlled through [SafetyDistances](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/collisions/struct.SafetyDistances.html) structure as shown in the example below. 

Safety distances can be configured separately for robot-to-robot and robot-to-environment collisions. 
Shorter distances can be specified for joints that naturally operate in proximity.

The code below demonstrates how to create this structure, complete with tool,
base and constraints (see also [example](examples/complete_visible_robot.rs).:

```Rust
use std::ops::RangeInclusive;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_opw_kinematics::constraints::{Constraints, BY_PREV};
use rs_opw_kinematics::collisions::{BaseBody, CollisionBody, RobotBody};
use rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape;
use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
use rs_opw_kinematics::{utils, visualization};
use rs_read_trimesh::load_trimesh; // loads .stl, .ply, .obj and .dae. Supports scaling. 


/// Create the sample robot we will visualize. This function creates
/// Staubli RX160, using is parameter set.
/// It loads the joint meshes from .stl files bundles in the test folder
/// where they are shared under the rights of Apache license (ROS Industrial project).
/// Four environment objects and tool are also created.
pub fn create_rx160_robot() -> Result<KinematicsWithShape, String> {

  // Environment object to collide with.
  let monolith = load_trimesh("src/tests/data/object.stl", 1.0)?;

  Ok(KinematicsWithShape::with_safety(
    // OPW parameters for Staubli RX 160
    Parameters {
      a1: 0.15,
      a2: 0.0,
      b: 0.0,
      c1: 0.55,
      c2: 0.825,
      c3: 0.625,
      c4: 0.11,
      ..Parameters::new()
    },
    // Define constraints directly in degrees, converting internally to radians.
    Constraints::from_degrees(
      [
        -225.0..=225.0,
        -225.0..=225.0,
        -225.0..=225.0,
        -225.0..=225.0,
        -225.0..=225.0,
        -360.0..=360.0,
      ],
      BY_PREV, // Prioritize previous joint position
    ),
    // Joint meshes
    [
      // If your meshes, if offset in .stl file, use Trimesh::transform_vertices,
      // you may also need Trimesh::scale in some extreme cases.
      // If your joints or tool consist of multiple meshes, combine these
      // with Trimesh::append
      load_trimesh("src/tests/data/staubli/rx160/link_1.stl",1.0)?,
      load_trimesh("src/tests/data/staubli/rx160/link_2.stl",1.0)?,
      load_trimesh("src/tests/data/staubli/rx160/link_3.stl",1.0)?,
      load_trimesh("src/tests/data/staubli/rx160/link_4.stl",1.0)?,
      load_trimesh("src/tests/data/staubli/rx160/link_5.stl",1.0)?,
      load_trimesh("src/tests/data/staubli/rx160/link_6.stl",1.0)?,
    ],
    // Base link mesh
    load_trimesh("src/tests/data/staubli/rx160/base_link.stl",1.0)?,
    // Base transform, this is where the robot is standing
    Isometry3::from_parts(
      Translation3::new(0.4, 0.7, 0.0).into(),
      UnitQuaternion::identity(),
    ),
    // Tool mesh. Load it from .ply file for feature demonstration
    load_trimesh("src/tests/data/flag.ply",1.0)?,
    // Tool transform, tip (not base) of the tool. The point past this
    // transform is known as tool center point (TCP).
    Isometry3::from_parts(
      Translation3::new(0.0, 0.0, 0.5).into(),
      UnitQuaternion::identity(),
    ),
    // Objects around the robot, with global transforms for them.
    vec![
      CollisionBody {
        mesh: monolith.clone(),
        pose: Isometry3::translation(1., 0., 0.),
      },
      CollisionBody {
        mesh: monolith.clone(),
        pose: Isometry3::translation(-1., 0., 0.),
      },
      CollisionBody {
        mesh: monolith.clone(),
        pose: Isometry3::translation(0., 1., 0.),
      },
      CollisionBody {
        mesh: monolith.clone(),
        pose: Isometry3::translation(0., -1., 0.),
      },
    ],
    SafetyDistances {
      to_environment: 0.05,   // Robot should not come closer than 5 cm to pillars
      to_robot_default: 0.05, // No closer than 5 cm to itself.
      special_distances: SafetyDistances::distances(&[
        // Due construction of this robot, these joints are very close, so
        // special rules are needed for them.
        ((J2, J_BASE), NEVER_COLLIDES), // base and J2 cannot collide
        ((J3, J_BASE), NEVER_COLLIDES), // base and J3 cannot collide
        ((J2, J4), NEVER_COLLIDES),
        ((J3, J4), NEVER_COLLIDES),
        ((J4, J_TOOL), 0.02_f32), // reduce distance requirement to 2 cm.
        ((J4, J6), 0.02_f32),     // reduce distance requirement to 2 cm.
      ]),
      mode: CheckMode::AllCollsions, // we need to report all for visualization
      // mode: CheckMode::NoCheck, // this is very fast but no collision check
    },
  ))
}
```

## Path planning
There are currently few path planning libraries available in Rust. Instead of incorporating them directly into our project
and writing the code around, we decided to explore the complexity of integrating these libraries as external dependencies
(referenced only in examples). This approach allowed us to identify key "pain points" that complicate the integration of
external path planners.

We provide external support for two libraries, `rrt` and `pathfinding`. Although `rs-opw-kinematics` does not use either
internally, we include examples demonstrating their usage. These two libraries are listed as 
development dependencies in `Cargo.toml`.

### rrt
The Rapidly-Exploring Random Tree (RRT) library, [rrt](https://github.com/openrr/rrt), is available under the 
Apache 2.0 license by Takashi Ogura and Mitsuharu Kojima. It can be used the following way:

```Rust
use rrt::dual_rrt_connect;
fn plan_path(
  kinematics: &KinematicsWithShape,
  start: Joints, goal: Joints,
) -> Result<Vec<Vec<f64>>, String> {
  let collision_free = |joint_angles: &[f64]| -> bool {
    let joints = &<Joints>::try_from(joint_angles).expect("Cannot convert vector to array");
    !kinematics.collides(joints)
  };

  // Constraint compliant random joint configuration generator. 
  let random_joint_angles = || -> Vec<f64> {
    // RRT requires vector and we return array so convert
    return kinematics.constraints()
            .expect("Set joint ranges on kinematics").random_angles().to_vec();
  };

  // Plan the path with RRT
  dual_rrt_connect(
    &start, &goal, collision_free,
    random_joint_angles, 3_f64.to_radians(), // Step size in joint space
    2000,  // Max iterations
  )
}
```
This library required to produce random joint angles within constraints. We made constraints easily
accessible from the instance of Kinematics, and provided random_angles() methods for them.

See the [example](examples/path_planning_rrt.rs). for how to define the robot and other boilerplate code. The direct output
will be a vector of vectors (not vector of Joints), each representing a step in the trajectory.

## Cartesian stroke
Producing a robot's movement over the surface of an object performing a task (such as welding, painting, or washing)
involves more than simply converting a single pose into joint rotations. Such a task requires a series of poses where
the transitions between poses must often follow a straight-line trajectory. This cannot be assumed when joints undergo
significant rotations between poses. Additionally, the robot's configurations (defined by joint angles) for adjacent
poses must blend smoothly without abrupt changes. Abrupt jumps often result from alternative solutions in inverse
kinematics. While these alternative solutions may provide suitable positions for the tool center point, they can cause
large joint rotations (e.g., 110 degrees), significantly increasing the risk of collisions.

This means that although alternative solutions exist, the initial configuration at the start of a stroke often
determines how the stroke progresses. If the trajectory cannot be completed before finishing the stroke, it may still
be possible to execute the stroke by starting with a different initial configuration.

For this reason, the stroke planning in this library consists of the following steps:

- **Starting from the "home" position and moving to the "landing" position**:  
  The landing position should be close to the working surface and slightly elevated to allow the robot to move safely
  into this configuration without risky movements near the surface. This phase is planned using the Rapidly-exploring
  Random Tree (RRT) algorithm.

- **Executing the stroke**:  
  The robot transitions from the landing position to the first stroke position, moves between stroke positions, and
  finally returns to a "parking" position, lifting away from the surface.
  - All strokes in this phase are Cartesian, even if the steps between stroke points are large.
  - The planner generates sufficient intermediate poses to ensure the robot avoids collisions during long linear
    movements and prevents unexpected configuration changes.
  - These "intermediate" poses are flagged and can be included in the output (for simpler robots) or excluded (for
    advanced robots capable of executing Cartesian strokes using their built-in software).

You will find the complete code in cartesian_stroke.rs between examples. 

```Rust
    let planner = Cartesian {
        robot: &k, // The robot, instance of KinematicsWithShape
        check_step_m: 0.02, // Pose distance check accuracy in meters (for translation)
        check_step_rad: 3.0_f64.to_radians(), // Pose distance check accuracy in radians (for rotation)
        max_transition_cost: 3_f64.to_radians(), // Maximal transition costs (not tied to the parameter above)
        // (weighted sum of abs differences between 'from' and 'to' for all joints, radians).
        transition_coefficients: DEFAULT_TRANSITION_COSTS, // Joint weights to compute transition cost
        linear_recursion_depth: 8,

        // RRT planner that computes the non-Cartesian path from starting position to landing pose
        rrt: RRTPlanner {
            step_size_joint_space: 2.0_f64.to_radians(), // RRT planner step in joint space
            max_try: 1000,
            debug: true,
        },
        include_linear_interpolation: true, // If true, intermediate Cartesian poses are
        // included in the output. Otherwise, they are checked but not included in the output

        debug: true, // verbose output to console
    };

    // plan path
    let started = Instant::now();
    // start is Joints, starting position. land and park are landing and parking poses. steps is the vector of poses.
    let path = planner.plan(&start, &land, steps, &park);
    let elapsed = started.elapsed();

    match path {
        Ok(path) => {
            for joints in path {
                println!("{:?}", &joints);
            }
        }
        Err(message) => {
            println!("Failed: {}", message);
        }
    }
    println!("Took {:?}", elapsed);
```

It is important that while Parry3D can compute distances till collision objects and plan with safety margins,
it is much slower than simply checking for collisions. Example explains how to create the SafetyDistances
object that can be used for specifying how collisions should be checked. It is possible to specify the check with 
safety margin, or just check for collisions, or do not check for collisions at all if we concentrate on path and 
constraints to be sure everything is collision-free anyway.

Please see the [example](examples/cartesian_stroke.rs).

**Note**: versions 1.8.2 and below may produce large rotation while the tool center point is formally following Cartesian path. This is fixed since 1.8.3. Under these rare conditions (that occur only near, but not at the J5 = 0 singularity point), the rotation (not translation) of the generated pose may differ from requested, by no more than the value of Cartesian.check_step_rad parameter.
    

## Visualization
[KinematicsWithShape](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/kinematics_with_shape/struct.KinematicsWithShape.html)
is also straightforward to visualize, as it fully integrates both the kinematics and 3D meshes representing the robot.
To display it, simply pass this structure to the built-in function 
[visualize_robot](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/visualization/fn.visualize_robot.html):

```Rust
fn main() {
    // The robot itself (as per example above)
    let robot = create_rx160_robot();

    // In which position to show the robot on startup
    let intial_angles = [173., -8., -94., 6., 83., 207.];

    // Boundaries for XYZ drawbars in visualization GUI
    let tcp_box: [RangeInclusive<f64>; 3] = [-2.0..=2.0, -2.0..=2.0, 1.0..=2.0];

    visualization::visualize_robot(robot, intial_angles, tcp_box);
}
```

Visualization primarily serves to verify that your parameters, meshes, tool, and base setup are correct.
It is not intended as a production feature. Using Bevy, the visualization will display the robot 
(mounted on its base and equipped with the tool), various environment objects, and a selection of handles to 
manipulate the robot. 

In the visualization window, you can adjust joint positions for forward kinematics or set the tool center point using
Cartesian coordinates for inverse kinematics. Collision detection is active in both modes, but it functions differently:
in inverse kinematics, movement is restricted to prevent collisions entirely (the robot will not move if a collision
would occur). In forward kinematics, collisions are allowed, but colliding robot joints and environment objects will be
highlighted.

When using inverse kinematics, you may observe unexpected large "jumps," or in some cases, no viable solution within the
robot's reach, constraints, and collision boundaries. This simply reflects the inherent limitations and complexities of
real-world robotic movement.


# Configuring the solver for your robot

The project contains built-in definitions for Igus Rebel, ABB IRB 2400/10, IRB 2600-12/1.65, IRB 4600-60/2.05; KUKA KR 6 R700 sixx,
FANUC R-2000iB/200R; Stäubli TX40, TX2-140, TX2-160 and TX2-160L with various levels of
testing. Robot manufacturers may provide such configurations for the robots they make.
For instance, FANUC M10IA is
described [here](https://github.com/ros-industrial/fanuc/blob/3ea2842baca3184cc621071b785cbf0c588a4046/fanuc_m10ia_support/config/opw_parameters_m10ia.yaml).
Many other robots are described in [ros-industrial/fanuc](https://github.com/ros-industrial/fanuc) repository.
This project contains the code for reading such configurations directly, including support for *deg(angle)*
function that sometimes occurs there:

```Rust
  let parameters = Parameters::from_yaml_file(filename).expect("Failed to load parameters");
  println!("Reading:\n{}", &parameters.to_yaml());
  let robot = OPWKinematics::new(parameters);
```

Since version 1.2.0, parameters and constraints can also be directly extracted from URDF file:

```Rust
  let robot = rs_opw_kinematics::urdf::from_urdf_file("/path/to/robot.urdf");
  println!("Reading:\n{}", &parameters.to_yaml());
```

There is also more advanced
function [rs_opw_kinematics::urdf::from_urdf](https://docs.rs/rs-opw-kinematics/1.8.2/rs_opw_kinematics/urdf/fn.from_urdf.html)
that takes URDF string rather than the file, provides error handling and much more control over how the solver
is constructed from the extracted values.

YAML reader supports additional 'dof' field that can be set to 6 (default) or 5 (5DOF robot, tool rotation
not accounted for). The URDF reader has als been extended to support such robots, but joint names must always be
explicitly provided. Instead of specifying a name for joint 6, the name of the tool center point (TCP) must be given.
Both YAML and URDF readers still try to get the parameter c4 that is now distance from J5 axis till TCP.

**Important:** The URDF reader assumes a robot with a parallel base and spherical wrist and not an arbitrary robot.
You can easily check this in the robot documentation or simply looking into the drawing. If the robot appears OPW
compliant yet parameters are not extracted correctly, please submit a bug report, providing URDF file and expected
values. Use visualization as explained before feeding the output to the physical robot.

# Disabling filesystem

For security and performance, some users prefer smaller libraries with fewer dependencies. If YAML and URDF readers
are not in use and meshes for collision detection are obtained from somewhere else (
or collision detection is not used), the filesystem access can be completely disabled in your Cargo.toml, importing the
library like:

rs-opw-kinematics = { version = ">=1.8.0, <2.0.0", default-features = false }

In this case, import of URDF and YAML files will be inaccessible, visualization and
collision detection will not work either, and used dependencies
will be limited to the single _nalgebra_ crate.

# Testing

The code of this project is tested against the test set (cases.yaml, 2048 cases per robot) that is
believed to be correct for the two robots, KUKA KR 6 R700 sixx and ABB IRB 2400/10. It has been produced
using independent C++ implementation by [Jmeyer1292/opw_kinematics](https://github.com/Jmeyer1292/opw_kinematics). The
testing suite checks if the solutions
match.



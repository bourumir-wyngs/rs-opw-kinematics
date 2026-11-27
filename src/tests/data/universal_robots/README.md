# Universal Robots UR5e

`ur5e.urdf` uses the manufacturer's nominal UR5e configuration. The test
chain runs from `base_link` to `tool0`. Its forearm and wrist geometry
provides a rejection case for the spherical-wrist OPW model. Controller
calibration can change the measured joint transforms of a particular arm.

The fixture expands the package entry point with `ur_type:=ur5e`, an empty
frame prefix, and the upstream default kinematics and joint limits. Links
are empty. The file preserves all twelve fixed and revolute joint definitions,
including their origins, axes and limits. Graphical assets, inertial data,
joint dynamics and simulator extensions are omitted.

Upstream licenses the kinematic description under BSD-3-Clause. Its README
lists separate terms for graphical assets. Contributor attribution from the
robot macro is retained in the fixture. [Upstream license scope](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/0eb1a861f7772fceb38510d90fa2f98ec668070c/README.md#license) ·
[Upstream BSD-3-Clause text](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/0eb1a861f7772fceb38510d90fa2f98ec668070c/LICENSE) ·
[Local license](LICENSE)

| Upstream source | Purpose |
|---|---|
| [URDF entry point](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/0eb1a861f7772fceb38510d90fa2f98ec668070c/urdf/ur.urdf.xacro) | Instantiates the arm and defines the world mount. |
| [Arm macro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/0eb1a861f7772fceb38510d90fa2f98ec668070c/urdf/ur_macro.xacro) | Defines links, moving joints and fixed frames. |
| [UR5e geometry](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/0eb1a861f7772fceb38510d90fa2f98ec668070c/config/ur5e/default_kinematics.yaml) | Supplies nominal joint transforms. |
| [UR5e limits](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/0eb1a861f7772fceb38510d90fa2f98ec668070c/config/ur5e/joint_limits.yaml) | Supplies joint limits. |

# KUKA test models

These mesh-free URDFs were extracted from the KUKA robot description packages. They retain the original joint frames, rotary axes and limits. The Xacro prefix is empty. Use `base_link` to `tool0` for all three. Each model declares its joint axes in local frames that have their own rotations.

| Local fixture | Upstream description | Package |
|---|---|---|
| `kr16_r1610_2.urdf` | [KR 16 R1610-2 Xacro][kr16] | [CYBERTECH][pkg16] |
| `kr70_r2100.urdf` | [KR 70 R2100 Xacro][kr70] | [IONTEC][pkg70] |
| `kr360_r2830.urdf` | [KR 360 R2830 Xacro][kr360] | [FORTEC][pkg360] |

These packages declare Apache-2.0. Their upstream license includes the 2025 KUKA Hungaria Kft. copyright notice. [Local license](LICENSE-kuka-ros.txt) · [Upstream license][kuka-license]

The existing Xacro and OPW YAML fixtures came from the ROS-Industrial [KR 6][old-kr6], [KR 10][old-kr10] and [KR 150][old-kr150] support packages. Their Apache-2.0 terms are included in [LICENSE.txt](LICENSE.txt).

[kr16]: https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/kuka_cybertech_support/urdf/kr16_r1610_2_macro.xacro
[kr70]: https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/kuka_iontec_support/urdf/kr70_r2100_macro.xacro
[kr360]: https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/kuka_fortec_support/urdf/kr360_r2830_macro.xacro
[pkg16]: https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/kuka_cybertech_support/package.xml
[pkg70]: https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/kuka_iontec_support/package.xml
[pkg360]: https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/kuka_fortec_support/package.xml
[kuka-license]: https://github.com/kuka-ros/kuka_robot_descriptions/blob/5ddbc41ce8726fd680bcc19562d78bcabfbdf96d/LICENSE
[old-kr6]: https://github.com/ros-industrial/kuka_experimental/tree/8d9292b04a22628b1b78d989e2ddd3abb913bf92/kuka_kr6_support
[old-kr10]: https://github.com/ros-industrial/kuka_experimental/tree/8d9292b04a22628b1b78d989e2ddd3abb913bf92/kuka_kr10_support
[old-kr150]: https://github.com/ros-industrial/kuka_experimental/tree/8d9292b04a22628b1b78d989e2ddd3abb913bf92/kuka_kr150_support

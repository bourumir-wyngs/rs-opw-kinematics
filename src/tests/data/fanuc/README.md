# FANUC test models

The mesh-free URDFs below retain the upstream joint frames, axes and travel limits. The ROS-Industrial Xacro macros were expanded with an empty prefix. The MoveIt M-10iA source was already expanded. The base and tip for each fixture are `base_link` and `tool0`.

| Local fixture | Upstream description | Reference parameters |
|---|---|---|
| `r2000ic165f.urdf` | [R-2000iC/165F Xacro][r2000] | [opw_parameters_r2000ic165f.yaml][r2000-yaml] |
| `lrmate200ic.urdf` | [LR Mate 200iC Xacro][lrmate] | [opw_parameters_lrmate200ic.yaml][lrmate-yaml] |
| `lrmate200ic5h.urdf` | [LR Mate 200iC/5H Xacro][lrmate5h] | The five-axis variant omits the forearm roll. |
| `moveit_m10ia.urdf` | [MoveIt Resources M-10iA URDF][moveit] | This description connects joint 6 directly to `tool0`. [Upstream provenance][moveit-readme] |

Both upstream projects distribute these descriptions under the TU Delft Robotics Institute BSD-3-Clause license. The full copyright notice and terms accompany the fixtures in [LICENSE.txt](LICENSE.txt). [ROS-Industrial license][fanuc-license] · [MoveIt package license][moveit-license]

The existing LR Mate 200iB, M-6iB, M-10iA and M-20iA Xacro fixtures and their OPW YAML come from the corresponding packages in [ROS-Industrial FANUC, melodic-devel][fanuc-old]. The existing M-16iB/20 and M-20iB/25 numeric URDFs record their pinned Xacro sources in their file headers. The `fanuc_m16ib20*.yaml` files exercise this crate's parser with variations on the M-16iB/20 parameter set.

[r2000]: https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_r2000ic_support/urdf/r2000ic165f_macro.xacro
[r2000-yaml]: https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_r2000ic_support/config/opw_parameters_r2000ic165f.yaml
[lrmate]: https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_lrmate200ic_support/urdf/lrmate200ic_macro.xacro
[lrmate-yaml]: https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_lrmate200ic_support/config/opw_parameters_lrmate200ic.yaml
[lrmate5h]: https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/fanuc_lrmate200ic_support/urdf/lrmate200ic5h_macro.xacro
[fanuc-license]: https://github.com/ros-industrial/fanuc/blob/d8f42bd73584b255df87098395512538882caea1/LICENSE
[moveit]: https://github.com/moveit/moveit_resources/blob/e1657c128ea61621777a041e3783c9c7c93db4ca/fanuc_description/urdf/fanuc.urdf
[moveit-readme]: https://github.com/moveit/moveit_resources/blob/e1657c128ea61621777a041e3783c9c7c93db4ca/fanuc_description/README.md
[moveit-license]: https://github.com/moveit/moveit_resources/blob/e1657c128ea61621777a041e3783c9c7c93db4ca/fanuc_description/LICENSE
[fanuc-old]: https://github.com/ros-industrial/fanuc/tree/786fb173a2facf4fdcaa1124fcc8af8c6ecb8b3e

# ABB robot descriptions

These kinematic fixtures use ROS-Industrial's ABB models at revision
[`45f4769`](https://github.com/ros-industrial/abb/tree/45f4769d826cf3ac62a65495f2db67b78b0c81df).
The base-to-tip path runs from `base_link` to `tool0`. Distances are in
metres and joint angles are in radians.

| Fixture | Geometry used in the tests | Upstream description and license |
|---|---|---|
| [IRB2400-12/1.55](irb2400_12_155.urdf) | Two levers mimic J3 on a side branch. A fixed transform supplies the 85 mm tool offset. | [Xacro][2400] · [Apache-2.0][2400-license] |
| [IRB4400L-30/2.43](irb4400l_30_243.urdf) | The translation into J4 has both horizontal and vertical components. The J5 and J6 origins have zero translation. | [Xacro][4400] · [Apache-2.0][4400-license] |
| [IRB5400](irb5400.urdf) | Both J5 and J5b lie on the tool path. J5b mimics J5 with multiplier -1 about a different axis. | [Xacro][5400] · [Apache-2.0][5400-license] |

These three files retain every physical joint, including its origin, axis,
limits and coupling declarations where present. The Xacro prefix is empty,
numeric expressions are expanded, and links contain only their names. The
modification and attribution notices are in the XML headers. The three
upstream packages share the [included license](LICENSE).

The smaller IRB120, IRB1200, IRB1600 and IRB4600 fixtures also identify
their upstream sources in their file headers. Their extracts omit joint
limits. The [IRB1600 reference parameters](opw_parameters_irb1600_6_120.yaml)
come from the corresponding [upstream OPW configuration][1600-opw].

[2400]: https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb2400_support/urdf/irb2400_12_155_macro.xacro
[2400-license]: https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb2400_support/LICENSE
[4400]: https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb4400_support/urdf/irb4400l_30_243_macro.xacro
[4400-license]: https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb4400_support/LICENSE
[5400]: https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb5400_support/urdf/irb5400_macro.xacro
[5400-license]: https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb5400_support/LICENSE
[1600-opw]: https://github.com/ros-industrial/abb/blob/45f4769d826cf3ac62a65495f2db67b78b0c81df/abb_irb1600_support/config/opw_parameters_irb1600_6_120.yaml

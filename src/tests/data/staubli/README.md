# Stäubli robot descriptions

The [RX160L](rx160l.urdf) and [TX2-60](tx2_60.urdf) fixtures follow the
path from `base_link` to `tool0`. Both wrists have vertical roll axes at
joint zero. The RX160L has a 0.925 m separation between the J4 and J5
origins. The TX2-60 has a 0.02 m lateral offset in its independently
published [OPW parameters](opw_parameters_tx2_60.yaml).

| Fixture | Upstream source | Included license |
|---|---|---|
| RX160L | ROS-Industrial [URDF][rx-model] · [package manifest][rx-package] · [upstream license][rx-license] | [Apache-2.0](LICENSE) |
| TX2-60 | ROS-Industrial [Xacro][tx-model] · [OPW parameters][tx-parameters] · [package manifest][tx-package] · [upstream license][tx-license] | [Apache-2.0](LICENSE) |

The URDF extracts preserve link names and physical joints, including their
axes, origins and limits. Meshes, link inertias and joint dynamics are
omitted. The TX2-60 Xacro uses an empty prefix and expanded numeric
expressions. Its OPW YAML is copied unchanged from upstream. URDF lengths
are in metres and angles are in radians.

[rx-model]: https://github.com/ros-industrial/staubli/blob/367d92ec7432867d9ba87c3399d49a940c4afc67/staubli_rx160_support/urdf/rx160l.urdf
[rx-package]: https://github.com/ros-industrial/staubli/blob/367d92ec7432867d9ba87c3399d49a940c4afc67/staubli_rx160_support/package.xml
[rx-license]: https://github.com/ros-industrial/staubli/blob/367d92ec7432867d9ba87c3399d49a940c4afc67/LICENSE
[tx-model]: https://github.com/ros-industrial/staubli_experimental/blob/256a71acea2161b64e12af1a6b925a7f26695007/staubli_tx2_60_support/urdf/tx2_60_macro.xacro
[tx-parameters]: https://github.com/ros-industrial/staubli_experimental/blob/256a71acea2161b64e12af1a6b925a7f26695007/staubli_tx2_60_support/config/opw_parameters_tx2_60.yaml
[tx-package]: https://github.com/ros-industrial/staubli_experimental/blob/256a71acea2161b64e12af1a6b925a7f26695007/staubli_tx2_60_support/package.xml
[tx-license]: https://github.com/ros-industrial/staubli_experimental/blob/256a71acea2161b64e12af1a6b925a7f26695007/LICENSE

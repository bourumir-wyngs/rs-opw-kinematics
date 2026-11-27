# DENSO VS060

The [VS060 kinematic fixture](vs060.urdf) comes from DENSO WAVE's
`denso_robot_descriptions` package. Its arm chain runs from `base_link` to
`J6`. Both wrist roll axes are vertical at joint zero. The six transmission
entries reference the same joint names as the physical joints. [Upstream URDF][model]

The extract retains all link names, joint transforms, axes, limits, and
transmission records. The original visual geometry, inertias and simulator
settings are omitted. Translation units are metres and rotation units are
radians.

The description package declares the MIT license. The [included license](LICENSE)
reproduces the upstream repository's complete notice, including the DENSO WAVE
copyright. [Package manifest][package] · [Upstream license][license]

[model]: https://github.com/DENSORobot/denso_robot_ros/blob/0bd656cde5ec2701872fb05d5870e682e27ff069/denso_robot_descriptions/vs060_description/vs060.urdf
[package]: https://github.com/DENSORobot/denso_robot_ros/blob/0bd656cde5ec2701872fb05d5870e682e27ff069/denso_robot_descriptions/package.xml
[license]: https://github.com/DENSORobot/denso_robot_ros/blob/0bd656cde5ec2701872fb05d5870e682e27ff069/LICENSE

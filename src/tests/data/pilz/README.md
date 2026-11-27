# Pilz PRBT 6

`pilz-prbt.urdf` contains the six-axis arm from the manufacturer's
`prbt_support` package. The test chain runs from `prbt_base_link` to
`prbt_tool0`. At joint zero, the wrist roll axes are vertical and the pitch
axis intersects them. The model retains six transmission elements that refer
to the physical joints.

The fixture expands the `prbt` macro with `prefix="prbt_"`. Links are empty.
Joint origins, axes, limits, fixed frames and transmission declarations are
preserved. Visual and collision meshes, inertial data, joint dynamics and
simulator extensions are omitted.

Copyright (c) 2018 Pilz GmbH & Co. KG. The source file carries an Apache-2.0
notice. [Original Xacro](https://github.com/PilzDE/pilz_robots/blob/ff607dbdf949a548dd14d43fac9ca36d6c886fbf/prbt_support/urdf/prbt_macro.xacro) ·
[Upstream package license](https://github.com/PilzDE/pilz_robots/blob/ff607dbdf949a548dd14d43fac9ca36d6c886fbf/prbt_support/LICENSE) ·
[Local license](LICENSE)

//! Provides implementation of inverse and direct kinematics.

use crate::constraints::{BY_CONSTRAINS, BY_PREV, Constraints};
use crate::kinematic_traits::{J4, J6};
use crate::kinematic_traits::{JOINTS_AT_ZERO, Joints, Kinematics, Pose, Solutions};
use crate::parameters::opw_kinematics::Parameters;
use crate::pose::PoseError;
use crate::singularity::wrist_pole::{self, POLE_PHASE_ROUNDOFF_THR};
use crate::singularity::{j1free, j1j2free, j2free};
use glam::{DMat3, DQuat, DVec3};
use std::f64::consts::PI;
use std::ops::Index;

const DEBUG: bool = false;

#[derive(Debug, Copy, Clone)]
pub struct OPWKinematics {
    /// The parameters that were used to construct this solver.
    pub(crate) parameters: Parameters,
    constraints: Option<Constraints>,
    /// Geometry permits a free J1 at a wrist center on the base axis.
    pub(crate) j1free: bool,
    /// Equal effective arm lengths permit a free J2 when fully folded.
    pub(crate) j2free: bool,
    /// Folding onto a shoulder on the base axis permits both J1 and J2 to be free.
    pub(crate) j1j2free: bool,
    /// Linear tolerance scaled to the total size of the robot geometry.
    pub(crate) distance_tolerance: f64,
}

impl OPWKinematics {
    /// Creates a new `OPWKinematics` instance with the given parameters.
    ///
    /// # Panics
    /// Panics if any joint offset is non-finite or outside ±2π radians (±360°).
    #[allow(dead_code)]
    pub fn new(parameters: Parameters) -> Self {
        Self::from_parameters(parameters, None)
    }

    /// Create a new instance that takes also Constraints.
    /// If constraints are set, all solutions returned by this solver are constraint compliant.
    ///
    /// # Panics
    /// Panics if any joint offset is non-finite or outside ±2π radians (±360°).
    pub fn new_with_constraints(parameters: Parameters, constraints: Constraints) -> Self {
        Self::from_parameters(parameters, Some(constraints))
    }

    fn from_parameters(parameters: Parameters, constraints: Option<Constraints>) -> Self {
        for (joint, offset) in parameters.offsets.iter().enumerate() {
            assert!(
                (-2.0 * PI..=2.0 * PI).contains(offset),
                "joint {} offset must be finite and within ±2π radians (±360°), got {offset}",
                joint + 1,
            );
        }
        let geometry_length = parameters.a1.abs()
            + parameters.a2.abs()
            + parameters.b.abs()
            + parameters.c1.abs()
            + parameters.c2.abs()
            + parameters.c3.abs()
            + parameters.c4.abs();

        let kappa = (parameters.a2 * parameters.a2 + parameters.c3 * parameters.c3).sqrt();
        let arm_length = parameters.c2.abs() + kappa;
        // Conservative geometry gates retain roundoff from link cancellation
        // and wrist-center/TCP subtraction; per-pose checks remain tighter.
        let transverse_roundoff =
            16.0 * ARM_ROUNDOFF * (parameters.a1.abs() + arm_length + parameters.c4.abs());
        let j1free = parameters.b == 0.0 && parameters.a1.abs() <= arm_length + transverse_roundoff;
        let j2free = parameters.c2 > 0.0
            && kappa > 0.0
            && (parameters.c2 - kappa).abs() <= ARM_ROUNDOFF * (parameters.c2 + kappa);
        let j1j2free = j1free && j2free && parameters.a1.abs() <= transverse_roundoff;

        Self {
            parameters,
            constraints,
            j1free,
            j2free,
            j1j2free,
            distance_tolerance: geometry_length * RELATIVE_DISTANCE_TOLERANCE,
        }
    }
}

/// Linear errors up to one part per million of the total robot geometry are accepted.
pub(crate) const RELATIVE_DISTANCE_TOLERANCE: f64 = 1E-6;
pub(crate) const ANGULAR_TOLERANCE: f64 = 1E-6;

// Relative allowance for the arithmetic and rotation operations in arm recovery.
// Unlike the FK tolerance, this only permits domain errors on the roundoff scale.
const ARM_ROUNDOFF: f64 = 64.0 * f64::EPSILON;

// Below this floor the regular wrist atan2 pairs cannot be reliably resolved.
const WRIST_ROUNDOFF_THR: f64 = 64.0 * f64::EPSILON;
// Merge roundoff duplicates without losing resolvable nonzero wrist bends.
pub(crate) const JOINT_DUPLICATE_THR: f64 = 64.0 * f64::EPSILON;

#[derive(Clone, Copy)]
pub(crate) struct RotationMatrix {
    pub(crate) matrix: DMat3,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ArmBranch {
    pub(crate) q1: f64,
    pub(crate) q2: f64,
    pub(crate) q3: f64,
}

/// Discrete branches and position equations that leave an arm angle free.
struct ArmRecovery {
    branches: [ArmBranch; 4],
    free_j1: bool,
    free_j2: [bool; 2],
    folded_q3: f64,
}

/// Preferred J4 and J6 in model coordinates, after sign corrections and offsets.
#[derive(Clone, Copy, Debug)]
pub(crate) struct J4J6Near {
    pub(crate) j4: f64,
    pub(crate) j6: f64,
}

impl J4J6Near {
    pub(crate) fn from_joints(joints: &Joints, parameters: &Parameters) -> Self {
        Self {
            j4: joints[J4] * parameters.sign_corrections[J4] as f64 - parameters.offsets[J4],
            j6: joints[J6] * parameters.sign_corrections[J6] as f64 - parameters.offsets[J6],
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct WristBranch {
    pub(crate) q4: f64,
    pub(crate) q5: f64,
    pub(crate) q6: f64,
}

impl RotationMatrix {
    pub(crate) fn from_quat(rotation: DQuat) -> Self {
        Self {
            matrix: DMat3::from_quat(rotation),
        }
    }
}

impl Index<(usize, usize)> for RotationMatrix {
    type Output = f64;

    fn index(&self, (row, col): (usize, usize)) -> &Self::Output {
        match (row, col) {
            (0, 0) => &self.matrix.x_axis.x,
            (1, 0) => &self.matrix.x_axis.y,
            (2, 0) => &self.matrix.x_axis.z,
            (0, 1) => &self.matrix.y_axis.x,
            (1, 1) => &self.matrix.y_axis.y,
            (2, 1) => &self.matrix.y_axis.z,
            (0, 2) => &self.matrix.z_axis.x,
            (1, 2) => &self.matrix.z_axis.y,
            (2, 2) => &self.matrix.z_axis.z,
            _ => panic!("rotation matrix index out of bounds: ({row}, {col})"),
        }
    }
}

fn mat3_from_rows(rows: [[f64; 3]; 3]) -> DMat3 {
    DMat3::from_cols(
        DVec3::new(rows[0][0], rows[1][0], rows[2][0]),
        DVec3::new(rows[0][1], rows[1][1], rows[2][1]),
        DVec3::new(rows[0][2], rows[1][2], rows[2][2]),
    )
}

impl Kinematics for OPWKinematics {
    /// Return the solution that is constraint compliant anv values are valid
    /// (no NaNs, etc) but otherwise not sorted.
    /// If this is 5 degree of freedom robot only, the 6 joint is set to 0.0
    /// The rotation of pose in this case is only approximate.
    fn inverse(&self, pose: &Pose) -> Solutions {
        if self.parameters.dof == 5 {
            return self.inverse_5dof(pose, 0.0);
        }

        self.filter_constraints_compliant(self.inverse_intern(pose, self.constraint_centers()))
    }

    // Resolves wrist singularities near the previous joint values in wrist_branch.
    // If this is 5 degree of freedom robot only, the 6 joint is set to as it was previous.
    // The rotation of pose in this case is only approximate.
    fn inverse_continuing(&self, pose: &Pose, prev: &Joints) -> Solutions {
        if self.parameters.dof == 5 {
            return self.inverse_continuing_5dof(pose, prev);
        }

        let previous = if prev[0].is_nan() {
            // Special value CONSTRAINT_CENTERED has been used
            self.constraint_centers()
        } else {
            prev
        };
        if !previous.iter().all(|joint| joint.is_finite()) {
            return Vec::new();
        }
        let mut solutions = self.inverse_intern(pose, previous);
        self.normalize_and_validate(&mut solutions, pose, previous, false);
        self.sort_by_closeness(&mut solutions, previous);
        self.filter_constraints_compliant(solutions)
    }

    fn forward(&self, joints: &Joints) -> Pose {
        self.try_forward(joints).expect("pose parts must be valid")
    }

    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        let p = &self.parameters;

        let q1 = joints[0] * p.sign_corrections[0] as f64 - p.offsets[0];
        let q2 = joints[1] * p.sign_corrections[1] as f64 - p.offsets[1];
        let q3 = joints[2] * p.sign_corrections[2] as f64 - p.offsets[2];
        let q4 = joints[3] * p.sign_corrections[3] as f64 - p.offsets[3];
        let q5 = joints[4] * p.sign_corrections[4] as f64 - p.offsets[4];
        let q6 = joints[5] * p.sign_corrections[5] as f64 - p.offsets[5];

        // Pose 1 is lifted by c1 as per URDF concepts (there is the base link that sits at 0,0,0)
        let pose1 = Pose::from_parts(
            DVec3::new(0.0, 0.0, p.c1),
            DQuat::from_axis_angle(DVec3::Z, q1),
        );

        // Pose 2: The c2 - spanning arm is by c1 up, by a1 along x, and rotated around z by
        let pose2 = pose1
            * Pose::from_parts(
                DVec3::new(p.a1, p.b, 0.0),
                DQuat::from_axis_angle(DVec3::Y, q2),
            );

        // Pose 3: The c3 - spanning arm goes starts further away by the length of c2.
        let pose3 = pose2
            * Pose::from_parts(
                DVec3::new(0.0, 0.0, p.c2),
                DQuat::from_axis_angle(DVec3::Y, q3),
            );

        // Pose 4: this part uses pose3 as a base and just rotates around z.
        let pose4 = pose3
            * Pose::from_parts(
                DVec3::new(p.a2, 0.0, 0.0),
                DQuat::from_axis_angle(DVec3::Z, q4),
            );

        // Pose 5 is the movable "nose" close to the tool center point.
        let pose5 = pose4
            * Pose::from_parts(
                DVec3::new(0.0, 0.0, p.c3),
                DQuat::from_axis_angle(DVec3::Y, q5),
            );

        // Pose 6 is pose of the tool-accepting joint that is often round and the
        // rotation not visible in rendering without tool
        let pose6 = pose5
            * Pose::from_parts(
                DVec3::new(0.0, 0.0, p.c4),
                DQuat::from_axis_angle(DVec3::Z, q6),
            );

        [pose1, pose2, pose3, pose4, pose5, pose6]
    }

    fn inverse_5dof(&self, pose: &Pose, j6: f64) -> Solutions {
        if !j6.is_finite() {
            return Vec::new();
        }
        let mut preferred = *self.constraint_centers();
        preferred[J6] = j6;
        self.filter_constraints_compliant(self.inverse_intern_5_dof(pose, j6, &preferred))
    }

    fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
        let previous = if prev[0].is_nan() {
            // Special value CONSTRAINT_CENTERED has been used
            self.constraint_centers()
        } else {
            prev
        };
        if !previous.iter().all(|joint| joint.is_finite()) {
            return Vec::new();
        }

        let mut solutions = self.inverse_intern_5_dof(pose, previous[5], previous);
        self.normalize_and_validate(&mut solutions, pose, previous, true);
        self.sort_by_closeness(&mut solutions, previous);
        self.filter_constraints_compliant(solutions)
    }

    fn constraints(&self) -> &Option<Constraints> {
        &self.constraints
    }
}

impl OPWKinematics {
    /// Computes the TCP pose from joint angles in radians, returning an error if
    /// the computed pose is invalid.
    ///
    /// This is the fallible counterpart of [`Kinematics::forward`]. On success,
    /// the translation is finite and the rotation is normalized. Joint limits
    /// are not checked, and success does not guarantee accuracy for extremely
    /// large angles that have lost precision.
    ///
    /// # Errors
    ///
    /// Returns the errors from [`Pose::try_from_parts`]:
    ///
    /// - [`PoseError::NonFiniteTranslation`] if the computed translation contains
    ///   NaN or infinity.
    /// - [`PoseError::NonFiniteRotation`] if the computed quaternion or its norm
    ///   is non-finite.
    /// - [`PoseError::ZeroRotation`] if the computed quaternion has zero norm.
    ///
    /// Non-finite joint angles or geometry parameters, and arithmetic overflow
    /// during forward kinematics, can cause these errors. Even finite inputs
    /// can overflow: for example, J2 and J3 both equal to `1e308` radians with
    /// positive sign corrections overflow when added, yielding
    /// [`PoseError::NonFiniteTranslation`].
    ///
    /// # Examples
    ///
    /// ```
    /// use rs_opw_kinematics::kinematics_impl::OPWKinematics;
    /// use rs_opw_kinematics::parameters::opw_kinematics::Parameters;
    /// use rs_opw_kinematics::pose::PoseError;
    ///
    /// let robot = OPWKinematics::new(Parameters::irb2400_10());
    /// let pose = robot.try_forward(&[0.0; 6])?;
    /// assert!(pose.translation.is_finite());
    ///
    /// let overflowing = [0.0, 1e308, 1e308, 0.0, 0.0, 0.0];
    /// assert_eq!(
    ///     robot.try_forward(&overflowing),
    ///     Err(PoseError::NonFiniteTranslation),
    /// );
    /// # Ok::<(), PoseError>(())
    /// ```
    pub fn try_forward(&self, joints: &Joints) -> Result<Pose, PoseError> {
        let p = &self.parameters;

        // Apply sign corrections and offsets
        let q1 = joints[0] * p.sign_corrections[0] as f64 - p.offsets[0];
        let q2 = joints[1] * p.sign_corrections[1] as f64 - p.offsets[1];
        let q3 = joints[2] * p.sign_corrections[2] as f64 - p.offsets[2];
        let q4 = joints[3] * p.sign_corrections[3] as f64 - p.offsets[3];
        let q5 = joints[4] * p.sign_corrections[4] as f64 - p.offsets[4];
        let q6 = joints[5] * p.sign_corrections[5] as f64 - p.offsets[5];

        let psi3 = f64::atan2(p.a2, p.c3);
        let k = f64::sqrt(p.a2 * p.a2 + p.c3 * p.c3);

        // Precompute q23_psi3 for better readability and reuse
        let q23_psi3 = q2 + q3 + psi3;
        let sin_q23_psi3 = q23_psi3.sin();
        let cos_q23_psi3 = q23_psi3.cos();

        let cx1 = p.c2 * f64::sin(q2) + k * sin_q23_psi3 + p.a1;
        let cy1 = p.b;
        let cz1 = p.c2 * f64::cos(q2) + k * cos_q23_psi3;

        let cx0 = cx1 * f64::cos(q1) - cy1 * f64::sin(q1);
        let cy0 = cx1 * f64::sin(q1) + cy1 * f64::cos(q1);
        let cz0 = cz1 + p.c1;

        // Precompute sines and cosines for efficiency
        let (s1, c1) = q1.sin_cos();
        let (s2, c2) = q2.sin_cos();
        let (s3, c3) = q3.sin_cos();
        let (s4, c4) = q4.sin_cos();
        let (s5, c5) = q5.sin_cos();
        let (s6, c6) = q6.sin_cos();

        // Compute rotation matrix r_0c
        let r_0c = mat3_from_rows([
            [
                c1 * c2 * c3 - c1 * s2 * s3,
                -s1,
                c1 * c2 * s3 + c1 * s2 * c3,
            ],
            [s1 * c2 * c3 - s1 * s2 * s3, c1, s1 * c2 * s3 + s1 * s2 * c3],
            [-s2 * c3 - c2 * s3, 0.0, -s2 * s3 + c2 * c3],
        ]);

        // Compute rotation matrix r_ce
        let r_ce = mat3_from_rows([
            [c4 * c5 * c6 - s4 * s6, -c4 * c5 * s6 - s4 * c6, c4 * s5],
            [s4 * c5 * c6 + c4 * s6, -s4 * c5 * s6 + c4 * c6, s4 * s5],
            [-s5 * c6, s5 * s6, c5],
        ]);

        // Compute the final rotation matrix r_oe
        let r_oe = r_0c * r_ce;

        // Calculate the final translation
        let translation = DVec3::new(cx0, cy0, cz0) + p.c4 * (r_oe * DVec3::Z);
        let rotation = DQuat::from_mat3(&r_oe);

        Pose::try_from_parts(translation, rotation)
    }

    /// Computes discrete arm branches and detects free angles on the same geometry.
    fn arm_branches(&self, pose: &Pose) -> ArmRecovery {
        let params = &self.parameters;

        // Adjust to wrist center
        let translation_vector = pose.translation;
        let scaled_z_axis = params.c4 * (pose.rotation * DVec3::Z);

        let c = translation_vector - scaled_z_axis;

        // Recovering the wrist center includes rotation and TCP subtraction
        // error. Keep component-wise length bounds so a small shoulder radius
        // does not inherit an allowance proportional to the whole robot squared.
        let c_error = ARM_ROUNDOFF * (translation_vector.abs() + DVec3::splat(params.c4.abs()));
        let radial_squared = c.x * c.x + c.y * c.y - params.b * params.b;
        let radial_squared_error = squared_norm_roundoff(c.x, c.y, c_error.x, c_error.y)
            + ARM_ROUNDOFF * params.b * params.b;
        let (radial, radial_error) = sqrt_with_roundoff(radial_squared, radial_squared_error);
        let nx1 = radial - params.a1;

        let tmp1 = c.y.atan2(c.x); // Rust's method call syntax for atan2(y, x)
        let tmp2 = params.b.atan2(radial);

        let theta1_i = tmp1 - tmp2;
        let theta1_ii = tmp1 + tmp2 - PI;

        let tmp3 = c.z - params.c1;
        let s1_2 = nx1 * nx1 + tmp3 * tmp3;

        let tmp4 = radial + params.a1;
        let s2_2 = tmp4 * tmp4 + tmp3 * tmp3;
        let kappa_2 = params.a2 * params.a2 + params.c3 * params.c3;

        let c2_2 = params.c2 * params.c2;

        // The sqrt uncertainty matters when both a1 and b are nonzero: close
        // to the shoulder cylinder it can amplify errors in either arm triangle.
        let planar_error = radial_error + ARM_ROUNDOFF * (radial + params.a1.abs());
        let height_error = c_error.z + ARM_ROUNDOFF * (c.z.abs() + params.c1.abs());
        let s1_2_error = squared_norm_roundoff(nx1, tmp3, planar_error, height_error);
        let s2_2_error = squared_norm_roundoff(tmp4, tmp3, planar_error, height_error);
        let triangle1_error = s1_2_error + ARM_ROUNDOFF * (s1_2 + c2_2 + kappa_2);
        let triangle2_error = s2_2_error + ARM_ROUNDOFF * (s2_2 + c2_2 + kappa_2);

        let tmp5 = s1_2 + c2_2 - kappa_2;

        let (s1, s1_error) = sqrt_with_roundoff(s1_2, s1_2_error);
        let (s2, s2_error) = sqrt_with_roundoff(s2_2, s2_2_error);

        let tmp13 = acos_with_roundoff(
            tmp5,
            2.0 * s1 * params.c2,
            triangle1_error + 2.0 * params.c2.abs() * s1_error,
        );
        let tmp14 = f64::atan2(nx1, tmp3);
        let theta2_i = -tmp13 + tmp14;
        let theta2_ii = tmp13 + tmp14;

        let tmp6 = s2_2 + c2_2 - kappa_2;

        let tmp15 = acos_with_roundoff(
            tmp6,
            2.0 * s2 * params.c2,
            triangle2_error + 2.0 * params.c2.abs() * s2_error,
        );
        let tmp16 = f64::atan2(tmp4, tmp3);
        let theta2_iii = -tmp15 - tmp16;
        let theta2_iv = tmp15 - tmp16;

        // theta3
        let tmp7 = s1_2 - c2_2 - kappa_2;
        let tmp8 = s2_2 - c2_2 - kappa_2;
        let tmp9 = 2.0 * params.c2 * f64::sqrt(kappa_2);
        let tmp10 = f64::atan2(params.a2, params.c3);

        let tmp11 = acos_with_roundoff(tmp7, tmp9, triangle1_error);
        let theta3_i = tmp11 - tmp10;
        let theta3_ii = -tmp11 - tmp10;

        let tmp12 = acos_with_roundoff(tmp8, tmp9, triangle2_error);
        let theta3_iii = tmp12 - tmp10;
        let theta3_iv = -tmp12 - tmp10;

        let kappa = kappa_2.sqrt();
        // Cancelling arm-link contributions can leave a residual even with
        // no flange or base height. Account for their length scale when
        // classifying free joints, separately from workspace-domain bounds.
        let link_error = ARM_ROUNDOFF * (params.c2.abs() + kappa);
        let radial_link_error = link_error + ARM_ROUNDOFF * params.a1.abs();
        let free_j1 = self.j1free && radial <= radial_error + radial_link_error;
        let folded = |x: f64| {
            self.j2free
                && x.abs() <= planar_error + radial_link_error
                && tmp3.abs() <= height_error + link_error
        };

        ArmRecovery {
            free_j1,
            free_j2: [folded(nx1), folded(tmp4)],
            folded_q3: PI - tmp10,
            branches: [
                ArmBranch {
                    q1: theta1_i,
                    q2: theta2_i,
                    q3: theta3_i,
                },
                ArmBranch {
                    q1: theta1_i,
                    q2: theta2_ii,
                    q3: theta3_ii,
                },
                ArmBranch {
                    q1: theta1_ii,
                    q2: theta2_iii,
                    q3: theta3_iii,
                },
                ArmBranch {
                    q1: theta1_ii,
                    q2: theta2_iv,
                    q3: theta3_iv,
                },
            ],
        }
    }

    /// Computes model-space J4-J6 candidates for one arm branch.
    /// Near a wrist pole, also resolves the coupled J4/J6 angles near the reference.
    pub(crate) fn wrist_branch(
        matrix: &RotationMatrix,
        arm: ArmBranch,
        near: &J4J6Near,
    ) -> impl Iterator<Item = WristBranch> + use<> {
        let (sin1, cos1) = arm.q1.sin_cos();
        let (sin23, cos23) = (arm.q2 + arm.q3).sin_cos();

        let m =
            matrix[(0, 2)] * sin23 * cos1 + matrix[(1, 2)] * sin23 * sin1 + matrix[(2, 2)] * cos23;
        let q4_y = matrix[(1, 2)] * cos1 - matrix[(0, 2)] * sin1;
        let q4_x =
            matrix[(0, 2)] * cos23 * cos1 + matrix[(1, 2)] * cos23 * sin1 - matrix[(2, 2)] * sin23;

        let q6_y =
            matrix[(0, 1)] * sin23 * cos1 + matrix[(1, 1)] * sin23 * sin1 + matrix[(2, 1)] * cos23;
        let q6_x =
            -matrix[(0, 0)] * sin23 * cos1 - matrix[(1, 0)] * sin23 * sin1 - matrix[(2, 0)] * cos23;
        // Two estimates of |sin(q5)|. Unlike sqrt(1 - m*m), these norms
        // preserve small, nonzero wrist bends near either pole.
        let sin5_e1 = q4_x.hypot(q4_y);
        let sin5_e2 = q6_x.hypot(q6_y);
        let q5 = sin5_e1.atan2(m.clamp(-1.0, 1.0));

        let pole = wrist_pole::recover(
            matrix,
            (sin1, cos1),
            (sin23, cos23),
            m,
            [sin5_e1, sin5_e2],
            near,
        );

        // Normal wrist IK: solve J4 and J6 individually using the nonzero J5 bend.
        let regular = (pole.is_none()
            || sin5_e1 > WRIST_ROUNDOFF_THR
            || sin5_e2 > WRIST_ROUNDOFF_THR)
            .then(|| WristBranch {
                q4: q4_y.atan2(q4_x),
                q5,
                q6: q6_y.atan2(q6_x),
            });

        // Return two candidates when only one method applies, or four when
        // pole recovery and regular solutions overlap. The iterator skips absent options
        // without allocating a Vec for each arm branch.
        [pole, regular].into_iter().flatten().flat_map(|wrist| {
            // Each method contributes the original wrist and its flipped orientation.
            [
                wrist,
                WristBranch {
                    q4: wrist.q4 + PI,
                    q5: -wrist.q5,
                    q6: wrist.q6 - PI,
                },
            ]
        })
    }

    /// Convert a wrist solution using the actual user reference, including pole limits.
    fn wrist_to_joints(
        &self,
        arm: ArmBranch,
        wrist: WristBranch,
        reference: &Joints,
        fixed_j6: Option<f64>,
    ) -> Option<Joints> {
        let params = &self.parameters;
        let theta = [arm.q1, arm.q2, arm.q3, wrist.q4, wrist.q5, wrist.q6];
        let mut joints = std::array::from_fn(|i| {
            (theta[i] + params.offsets[i]) * params.sign_corrections[i] as f64
        });
        wrist_pole::adjust_joints(
            &mut joints,
            wrist.q5,
            params,
            reference,
            self.constraints.as_ref(),
            fixed_j6,
        )?;
        if let Some(j6) = fixed_j6 {
            joints[J6] = j6;
        }
        Some(joints)
    }

    /// Validate a point on a singular arm family against its coupled wrist limits.
    /// Search modules use this same check when testing boundary intersections.
    pub(crate) fn arm_wrist_candidates(
        &self,
        pose: &Pose,
        arm: ArmBranch,
        reference: &Joints,
        fixed_j6: Option<f64>,
    ) -> Vec<Joints> {
        let near = J4J6Near::from_joints(reference, &self.parameters);
        let matrix = RotationMatrix::from_quat(pose.rotation);
        let mut result = Vec::new();
        for wrist in Self::wrist_branch(&matrix, arm, &near) {
            let Some(mut joints) = self.wrist_to_joints(arm, wrist, reference, fixed_j6) else {
                continue;
            };
            if !joints.iter().all(|joint| joint.is_finite()) {
                continue;
            }
            for i in 0..6 {
                if i == J6 && fixed_j6.is_some() {
                    continue;
                }
                joints[i] = wrapped_angle(joints[i]);
                if reference[i].is_finite() {
                    normalize_near(&mut joints[i], reference[i]);
                }
            }
            if let Some(constraints) = self.constraints {
                // Analytic interval endpoints can recover just outside an
                // inclusive limit. Move only roundoff-sized violations inside
                // before the strict constraints and FK checks below.
                for (i, joint) in joints.iter_mut().enumerate() {
                    if i == J6 && fixed_j6.is_some() {
                        continue;
                    }
                    let tolerance = constraints.tolerances[i];
                    let delta = wrapped_angle(*joint - constraints.centers[i]);
                    let excess = delta.abs() - tolerance;
                    if excess > 0.0 && excess <= POLE_PHASE_ROUNDOFF_THR {
                        *joint -=
                            delta.signum() * (excess + POLE_PHASE_ROUNDOFF_THR.min(tolerance));
                    }
                }
                if !constraints.compliant(&joints) {
                    continue;
                }
                // The public inverse paths also use a bounded representation.
                let bounded = joints.map(wrapped_angle);
                if !constraints.compliant(&bounded) {
                    continue;
                }
            }
            let Ok(actual) = self.try_forward(&joints) else {
                continue;
            };
            let orientation_error = if fixed_j6.is_some() {
                (actual.rotation * DVec3::Z - pose.rotation * DVec3::Z).length()
            } else {
                actual.angular_distance(*pose)
            };
            if (actual.translation - pose.translation).length() <= self.distance_tolerance
                && orientation_error <= ANGULAR_TOLERANCE
            {
                push_unique(&mut result, joints);
            }
        }
        result
    }

    /// Search free arm angles before FK validation and final constraint filtering.
    fn singular_arm_candidates(
        &self,
        pose: &Pose,
        recovery: &ArmRecovery,
        reference: &Joints,
        fixed_j6: Option<f64>,
    ) -> Vec<Joints> {
        let mut arms = Vec::new();
        if self.j1j2free && recovery.free_j1 && recovery.free_j2.iter().any(|free| *free) {
            arms.extend(j1j2free::search(
                self,
                pose,
                recovery.folded_q3,
                reference,
                fixed_j6,
            ));
        } else {
            for shoulder in 0..2 {
                if self.j2free && recovery.free_j2[shoulder] {
                    let arm = ArmBranch {
                        q1: recovery.branches[2 * shoulder].q1,
                        q2: 0.0,
                        q3: recovery.folded_q3,
                    };
                    arms.extend(j2free::search(self, pose, arm, reference, fixed_j6));
                } else if self.j1free && recovery.free_j1 {
                    for arm in recovery.branches[2 * shoulder..2 * shoulder + 2].iter() {
                        if arm.q2.is_finite() && arm.q3.is_finite() {
                            arms.extend(j1free::search(self, pose, *arm, reference, fixed_j6));
                        }
                    }
                }
            }
        }
        let mut result = Vec::new();
        for arm in arms {
            for joints in self.arm_wrist_candidates(pose, arm, reference, fixed_j6) {
                push_unique(&mut result, joints);
            }
        }
        self.sort_by_closeness(&mut result, reference);
        result
    }

    /// Keeps discrete solutions in the overlap with singular arm families.
    pub(crate) fn inverse_candidates(
        &self,
        pose: &Pose,
        reference: &Joints,
        fixed_j6: Option<f64>,
    ) -> impl Iterator<Item = Joints> + use<> {
        let matrix = RotationMatrix::from_quat(pose.rotation);
        let near = J4J6Near::from_joints(reference, &self.parameters);
        let recovery = self.arm_branches(pose);
        let singular = self.singular_arm_candidates(pose, &recovery, reference, fixed_j6);
        let robot = *self;
        let reference = *reference;
        recovery
            .branches
            .into_iter()
            .flat_map(move |arm| {
                Self::wrist_branch(&matrix, arm, &near).filter_map(move |wrist| {
                    robot.wrist_to_joints(arm, wrist, &reference, fixed_j6)
                })
            })
            // Search ranks near the user's reference, which may contain many
            // turns. Pass bounded angles into validation just as the discrete
            // formulas do; continuation restores the selected turns afterward.
            .chain(singular.into_iter().map(|joints| joints.map(wrapped_angle)))
    }

    fn inverse_intern(&self, pose: &Pose, reference: &Joints) -> Solutions {
        let mut result: Solutions = Vec::with_capacity(8);

        // Debug check. Solution failing cross-verification is flagged
        // as invalid. This loop also normalizes valid solutions to 0
        for (si, mut solution) in self.inverse_candidates(pose, reference, None).enumerate() {
            let mut valid = true;
            for angle in solution.iter_mut() {
                let mut current = *angle;
                if current.is_finite() {
                    while current > PI {
                        current -= 2.0 * PI;
                    }
                    while current < -PI {
                        current += 2.0 * PI;
                    }
                    *angle = current;
                } else {
                    valid = false;
                    break;
                }
            }
            if valid {
                let Ok(check_pose) = self.try_forward(&solution) else {
                    continue;
                };
                if compare_poses(
                    pose,
                    &check_pose,
                    self.distance_tolerance,
                    ANGULAR_TOLERANCE,
                ) {
                    push_unique(&mut result, solution);
                } else {
                    if DEBUG {
                        println!("********** Pose Failure sol {} *********", si);
                    }
                }
            }
        }

        result
    }

    fn inverse_intern_5_dof(&self, pose: &Pose, j6: f64, reference: &Joints) -> Solutions {
        let mut result: Solutions = Vec::with_capacity(8);

        // Debug check. Solution failing cross-verification is flagged
        // as invalid. This loop also normalizes valid solutions to 0
        // J6 is fixed below and its orientation is ignored; do not constrain
        // the provisional six-axis wrist phase in this path.
        for (si, mut solution) in self
            .inverse_candidates(pose, reference, Some(j6))
            .enumerate()
        {
            solution[J6] = j6; // J6 goes directly to response and is not more adjusted
            let mut valid = true;
            for angle in solution.iter_mut().take(5) {
                let mut current = *angle;
                if current.is_finite() {
                    while current > PI {
                        current -= 2.0 * PI;
                    }
                    while current < -PI {
                        current += 2.0 * PI;
                    }
                    *angle = current;
                } else {
                    valid = false;
                    break;
                }
            }
            if valid {
                let Ok(check_pose) = self.try_forward(&solution) else {
                    continue;
                };
                if Self::compare_xyz_only(
                    &pose.translation,
                    &check_pose.translation,
                    self.distance_tolerance,
                ) {
                    push_unique(&mut result, solution);
                } else {
                    if DEBUG {
                        println!("********** Pose Failure 5DOF sol {} *********", si);
                    }
                }
            }
        }

        result
    }

    fn compare_xyz_only(pose_translation: &DVec3, check_xyz: &DVec3, tolerance: f64) -> bool {
        (*pose_translation - *check_xyz).length() <= tolerance
    }

    fn filter_constraints_compliant(&self, solutions: Solutions) -> Solutions {
        match &self.constraints {
            Some(constraints) => constraints.filter(&solutions),
            None => solutions,
        }
    }

    /// Restore previous turns, then validate the actual angles being returned.
    /// Large finite references can lose pose-defining bits during normalization.
    fn normalize_and_validate(
        &self,
        solutions: &mut Solutions,
        pose: &Pose,
        previous: &Joints,
        five_dof: bool,
    ) {
        solutions.retain_mut(|solution| {
            // Five-axis J6 is fixed by the caller, not chosen by normalization.
            let moving_joints = if five_dof { 5 } else { 6 };
            for i in 0..moving_joints {
                normalize_near(&mut solution[i], previous[i]);
            }
            if !solution.iter().all(|joint| joint.is_finite()) {
                return false;
            }
            let Ok(actual) = self.try_forward(solution) else {
                return false;
            };
            if !pose.translation.is_finite() || !pose.rotation.is_finite() {
                return false;
            }
            let orientation_error = if five_dof {
                // Tool roll is ignored, but tool direction must still match,
                // including when c4 is zero and direction cannot affect XYZ.
                (actual.rotation * DVec3::Z - pose.rotation * DVec3::Z).length()
            } else {
                actual.angular_distance(*pose)
            };
            (actual.translation - pose.translation).length() <= self.distance_tolerance
                && orientation_error <= ANGULAR_TOLERANCE
        });
    }

    /// Sorts the solutions vector by closeness to the `previous` joint.
    /// Joints must be pre-normalized to be as close as possible, not away by 360 degrees
    pub(crate) fn sort_by_closeness(&self, solutions: &mut Solutions, previous: &Joints) {
        let sorting_weight = self
            .constraints
            .as_ref()
            .map_or(BY_PREV, |c| c.sorting_weight);
        let centers = self.constraint_centers();
        solutions.sort_by(|a, b| {
            weighted_distance(a, previous, centers, sorting_weight)
                .partial_cmp(&weighted_distance(b, previous, centers, sorting_weight))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
    }

    /// Get constraint centers in case we have the already constructed instance of the
    fn constraint_centers(&self) -> &Joints {
        self.constraints
            .as_ref()
            .map_or(&JOINTS_AT_ZERO, |c| &c.centers)
    }
}

pub(crate) fn wrapped_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

/// Allowed displacements from a reference, bounded to the nearest full turn.
/// A circular range may cross the +/-pi cut, so include its neighboring copies.
pub(crate) fn wrist_limit_intervals(
    center: f64,
    tolerance: f64,
    reference: f64,
    sign: f64,
) -> impl Iterator<Item = (f64, f64)> + Clone {
    let (center, tolerance) = if tolerance >= PI {
        // Includes the infinite tolerance used for an unconstrained joint.
        (0.0, PI)
    } else {
        (
            wrapped_angle(sign * (wrapped_angle(center) - reference)),
            tolerance,
        )
    };
    [-2.0 * PI, 0.0, 2.0 * PI]
        .into_iter()
        .filter_map(move |shift| {
            let lower = (center + shift - tolerance).max(-PI);
            let upper = (center + shift + tolerance).min(PI);
            (center.is_finite() && tolerance >= 0.0 && lower <= upper).then_some((lower, upper))
        })
}

/// Normalizes the angle `now` to be as close as possible to `must_be_near`
///
/// # Arguments
///
/// * `now` - A mutable reference to the angle to be normalized, radians
/// * `must_be_near` - The reference angle, radians
pub(crate) fn normalize_near(now: &mut f64, must_be_near: f64) {
    let two_pi = 2.0 * PI;
    // Smallest signed difference in (-π, π]
    let diff = (*now - must_be_near + PI).rem_euclid(two_pi) - PI;
    *now = must_be_near + diff;
}

pub(crate) fn calculate_distance(joint1: &[f64], joint2: &[f64]) -> f64 {
    joint1
        .iter()
        .zip(joint2.iter())
        .map(|(a, b)| (a - b).abs())
        .sum()
}

/// Shared ranking for whole solutions and the joints free at a wrist pole.
/// Callers normalize candidates near previous before comparing raw coordinates.
pub(crate) fn weighted_distance(
    joints: &[f64],
    previous: &[f64],
    centers: &[f64],
    sorting_weight: f64,
) -> f64 {
    let previous_distance = if sorting_weight == BY_CONSTRAINS {
        0.0
    } else {
        calculate_distance(joints, previous)
    };
    if sorting_weight == BY_PREV {
        return previous_distance;
    }
    previous_distance * (1.0 - sorting_weight)
        + calculate_distance(joints, centers) * sorting_weight
}

fn push_unique(solutions: &mut Solutions, candidate: Joints) {
    let duplicate = solutions.iter().any(|solution| {
        solution.iter().zip(candidate).all(|(existing, angle)| {
            let difference = (existing - angle + PI).rem_euclid(2.0 * PI) - PI;
            difference.abs() <= JOINT_DUPLICATE_THR
        })
    });
    if !duplicate {
        solutions.push(candidate);
    }
}

pub(crate) fn compare_poses(
    ta: &Pose,
    tb: &Pose,
    distance_tolerance: f64,
    angular_tolerance: f64,
) -> bool {
    let translation_distance = (ta.translation - tb.translation).length();
    let angular_distance = ta.angular_distance(*tb);

    if translation_distance.abs() > distance_tolerance {
        if DEBUG {
            println!("Positioning error: {}", translation_distance);
        }
        return false;
    }

    if angular_distance.abs() > angular_tolerance {
        if DEBUG {
            println!("Orientation errors: {}", angular_distance);
        }
        return false;
    }
    true
}

/// Square root with a bounded input error, returning the root and its error.
/// Near zero, propagate the square-root amplification instead of assuming that
/// a squared-length error stays small after taking the root.
fn sqrt_with_roundoff(value: f64, error: f64) -> (f64, f64) {
    if !value.is_finite() || !error.is_finite() || error < 0.0 || value < -error {
        return (f64::NAN, f64::NAN);
    }

    let root = value.max(0.0).sqrt();
    let root_error = if value > error {
        // Rationalized sqrt(value) - sqrt(value - error), avoiding cancellation.
        error / (root + (value - error).sqrt())
    } else {
        error.sqrt()
    };
    (root, root_error + ARM_ROUNDOFF * root)
}

/// Error in x² + y², including uncertainty already present in each coordinate.
fn squared_norm_roundoff(x: f64, y: f64, x_error: f64, y_error: f64) -> f64 {
    2.0 * x.abs() * x_error
        + x_error * x_error
        + 2.0 * y.abs() * y_error
        + y_error * y_error
        + ARM_ROUNDOFF * (x * x + y * y)
}

/// Clamp only roundoff-sized cosine domain violations. Compare before division:
/// a small denominator near inner reach can greatly amplify a tiny input error.
fn acos_with_roundoff(numerator: f64, denominator: f64, error: f64) -> f64 {
    if !numerator.is_finite()
        || !denominator.is_finite()
        || denominator == 0.0
        || !error.is_finite()
        || error < 0.0
        || numerator.abs() - denominator.abs() > error
    {
        // Unreachable or degenerate triangles must not become arbitrary angles.
        return f64::NAN;
    }
    (numerator.clamp(-denominator.abs(), denominator.abs()) / denominator).acos()
}

//! Provides implementation of inverse and direct kinematics.

use crate::constraints::{BY_CONSTRAINS, BY_PREV, Constraints};
use crate::kinematic_traits::{J4, J6};
use crate::kinematic_traits::{JOINTS_AT_ZERO, Joints, Kinematics, Pose, Solutions};
use crate::parameters::opw_kinematics::Parameters;
use glam::{DMat3, DQuat, DVec3};
use std::f64::consts::PI;
use std::ops::Index;

const DEBUG: bool = false;

#[derive(Debug, Copy, Clone)]
pub struct OPWKinematics {
    /// The parameters that were used to construct this solver.
    parameters: Parameters,
    constraints: Option<Constraints>,
    /// Linear tolerance scaled to the total size of the robot geometry.
    distance_tolerance: f64,
}

impl OPWKinematics {
    /// Creates a new `OPWKinematics` instance with the given parameters.
    #[allow(dead_code)]
    pub fn new(parameters: Parameters) -> Self {
        Self::from_parameters(parameters, None)
    }

    /// Create a new instance that takes also Constraints.
    /// If constraints are set, all solutions returned by this solver are constraint compliant.
    pub fn new_with_constraints(parameters: Parameters, constraints: Constraints) -> Self {
        Self::from_parameters(parameters, Some(constraints))
    }

    fn from_parameters(parameters: Parameters, constraints: Option<Constraints>) -> Self {
        let geometry_length = parameters.a1.abs()
            + parameters.a2.abs()
            + parameters.b.abs()
            + parameters.c1.abs()
            + parameters.c2.abs()
            + parameters.c3.abs()
            + parameters.c4.abs();

        Self {
            parameters,
            constraints,
            distance_tolerance: geometry_length * RELATIVE_DISTANCE_TOLERANCE,
        }
    }
}

/// Linear errors up to one part per million of the total robot geometry are accepted.
const RELATIVE_DISTANCE_TOLERANCE: f64 = 1E-6;
const ANGULAR_TOLERANCE: f64 = 1E-6;

// Below this floor the regular wrist atan2 pairs cannot be reliably resolved.
const WRIST_ROUNDOFF_THR: f64 = 64.0 * f64::EPSILON;
// Arm recovery can amplify roundoff beyond the matrix-only floor. Try pole
// candidates throughout the pose tolerance, alongside resolvable regular bends.
// Forward kinematics still validates every candidate.
const WRIST_POLE_OVERLAP_THR: f64 = ANGULAR_TOLERANCE;
// Merge roundoff duplicates without losing resolvable nonzero wrist bends.
const JOINT_DUPLICATE_THR: f64 = 64.0 * f64::EPSILON;

#[derive(Clone, Copy)]
struct RotationMatrix {
    matrix: DMat3,
}

#[derive(Clone, Copy, Debug)]
struct ArmBranch {
    q1: f64,
    q2: f64,
    q3: f64,
}

/// Preferred J4 and J6 in model coordinates, after sign corrections and offsets.
#[derive(Clone, Copy, Debug)]
struct J4J6Near {
    j4: f64,
    j6: f64,
}

impl J4J6Near {
    fn from_joints(joints: &Joints, parameters: &Parameters) -> Self {
        Self {
            j4: joints[J4] * parameters.sign_corrections[J4] as f64 - parameters.offsets[J4],
            j6: joints[J6] * parameters.sign_corrections[J6] as f64 - parameters.offsets[J6],
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct WristBranch {
    q4: f64,
    q5: f64,
    q6: f64,
}

impl RotationMatrix {
    fn from_quat(rotation: DQuat) -> Self {
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

        let near = J4J6Near::from_joints(self.constraint_centers(), &self.parameters);
        self.filter_constraints_compliant(self.inverse_intern(pose, &near))
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
        let near = J4J6Near::from_joints(previous, &self.parameters);

        let mut solutions = self.inverse_intern(pose, &near);
        // Before any sorting, normalize all angles to be close to
        // 'previous'
        for solution in &mut solutions {
            for (joint, previous_joint) in solution.iter_mut().zip(previous.iter()) {
                normalize_near(joint, *previous_joint);
            }
        }
        self.sort_by_closeness(&mut solutions, previous);
        self.filter_constraints_compliant(solutions)
    }

    fn forward(&self, joints: &Joints) -> Pose {
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

        Pose::from_parts(translation, rotation)
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
        let mut preferred = *self.constraint_centers();
        if j6.is_finite() {
            preferred[J6] = j6;
        }
        let near = J4J6Near::from_joints(&preferred, &self.parameters);
        self.filter_constraints_compliant(self.inverse_intern_5_dof(pose, j6, &near))
    }

    fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
        let previous = if prev[0].is_nan() {
            // Special value CONSTRAINT_CENTERED has been used
            self.constraint_centers()
        } else {
            prev
        };

        let near = J4J6Near::from_joints(previous, &self.parameters);
        let mut solutions = self.inverse_intern_5_dof(pose, prev[5], &near);

        // Before any sorting, normalize all angles to be close to
        // 'previous'
        for solution in &mut solutions {
            for (joint, previous_joint) in solution.iter_mut().zip(previous.iter()) {
                normalize_near(joint, *previous_joint);
            }
        }
        self.sort_by_closeness(&mut solutions, previous);
        self.filter_constraints_compliant(solutions)
    }

    fn constraints(&self) -> &Option<Constraints> {
        &self.constraints
    }
}

impl OPWKinematics {
    /// Computes the four model-space J1-J3 branches that place the wrist center.
    fn arm_branches(&self, pose: &Pose) -> [ArmBranch; 4] {
        let params = &self.parameters;

        // Adjust to wrist center
        let translation_vector = pose.translation;
        let scaled_z_axis = params.c4 * (pose.rotation * DVec3::Z);

        let c = translation_vector - scaled_z_axis;

        let nx1 = ((c.x * c.x + c.y * c.y) - params.b * params.b).sqrt() - params.a1;

        let tmp1 = c.y.atan2(c.x); // Rust's method call syntax for atan2(y, x)
        let tmp2 = params.b.atan2(nx1 + params.a1);

        let theta1_i = tmp1 - tmp2;
        let theta1_ii = tmp1 + tmp2 - PI;

        let tmp3 = c.z - params.c1;
        let s1_2 = nx1 * nx1 + tmp3 * tmp3;

        let tmp4 = nx1 + 2.0 * params.a1;
        let s2_2 = tmp4 * tmp4 + tmp3 * tmp3;
        let kappa_2 = params.a2 * params.a2 + params.c3 * params.c3;

        let c2_2 = params.c2 * params.c2;

        let tmp5 = s1_2 + c2_2 - kappa_2;

        let s1 = f64::sqrt(s1_2);
        let s2 = f64::sqrt(s2_2);

        let tmp13 = f64::acos(tmp5 / (2.0 * s1 * params.c2));
        let tmp14 = f64::atan2(nx1, c.z - params.c1);
        let theta2_i = -tmp13 + tmp14;
        let theta2_ii = tmp13 + tmp14;

        let tmp6 = s2_2 + c2_2 - kappa_2;

        let tmp15 = f64::acos(tmp6 / (2.0 * s2 * params.c2));
        let tmp16 = f64::atan2(nx1 + 2.0 * params.a1, c.z - params.c1);
        let theta2_iii = -tmp15 - tmp16;
        let theta2_iv = tmp15 - tmp16;

        // theta3
        let tmp7 = s1_2 - c2_2 - kappa_2;
        let tmp8 = s2_2 - c2_2 - kappa_2;
        let tmp9 = 2.0 * params.c2 * f64::sqrt(kappa_2);
        let tmp10 = f64::atan2(params.a2, params.c3);

        let tmp11 = f64::acos(tmp7 / tmp9);
        let theta3_i = tmp11 - tmp10;
        let theta3_ii = -tmp11 - tmp10;

        let tmp12 = f64::acos(tmp8 / tmp9);
        let theta3_iii = tmp12 - tmp10;
        let theta3_iv = -tmp12 - tmp10;

        [
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
        ]
    }

    /// Computes model-space J4-J6 candidates for one arm branch.
    /// Near a wrist pole, also resolves the coupled J4/J6 angles near the reference.
    fn wrist_branch(
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

        let pole = if sin5_e1 <= WRIST_POLE_OVERLAP_THR
            && sin5_e2 <= WRIST_POLE_OVERLAP_THR
            && near.j4.is_finite()
            && near.j6.is_finite()
        {
            // Upper-left block of R_arm^T * R_target. Its sum/difference
            // terms remain well-conditioned when the usual atan2 pairs vanish.
            let r00 = matrix[(0, 0)] * cos23 * cos1 + matrix[(1, 0)] * cos23 * sin1
                - matrix[(2, 0)] * sin23;
            let r01 = matrix[(0, 1)] * cos23 * cos1 + matrix[(1, 1)] * cos23 * sin1
                - matrix[(2, 1)] * sin23;
            let r10 = matrix[(1, 0)] * cos1 - matrix[(0, 0)] * sin1;
            let r11 = matrix[(1, 1)] * cos1 - matrix[(0, 1)] * sin1;

            // At q5=0 the observable phase is q4+q6; at q5=pi it is q4-q6.
            let sign = if m >= 0.0 { 1.0 } else { -1.0 };
            let phase = (sign * r10 - r01).atan2(r11 + sign * r00);
            // Work with bounded representatives so large references cannot
            // swallow the correction or enter the caller's normalization loops.
            // inverse_continuing restores the turn count near the reference.
            let near4 = near.j4.rem_euclid(2.0 * PI);
            let near6 = near.j6.rem_euclid(2.0 * PI);
            let previous_phase = near4 + sign * near6;
            let correction = (phase - previous_phase + PI).rem_euclid(2.0 * PI) - PI;

            // Equal splitting minimizes squared wrist motion. The existing
            // L1 solution sorter can tie on other splits of the same correction.
            // Offer an exact-pole candidate; the caller still checks it with FK.
            Some(WristBranch {
                q4: near4 + correction / 2.0,
                q5: if m >= 0.0 { 0.0 } else { PI },
                q6: near6 + sign * correction / 2.0,
            })
        } else {
            // Outside the overlap, or without a finite reference, use only regular solutions.
            None
        };

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

    /// Converts all arm/wrist candidates to joint coordinates before validation.
    fn inverse_candidates(
        &self,
        pose: &Pose,
        near: &J4J6Near,
    ) -> impl Iterator<Item = Joints> + use<> {
        let params = self.parameters;
        let matrix = RotationMatrix::from_quat(pose.rotation);
        let arm_branches = self.arm_branches(pose);
        let near = *near;
        arm_branches.into_iter().flat_map(move |arm| {
            Self::wrist_branch(&matrix, arm, &near).map(move |wrist| {
                let theta = [arm.q1, arm.q2, arm.q3, wrist.q4, wrist.q5, wrist.q6];
                std::array::from_fn(|i| {
                    (theta[i] + params.offsets[i]) * params.sign_corrections[i] as f64
                })
            })
        })
    }

    fn inverse_intern(&self, pose: &Pose, near: &J4J6Near) -> Solutions {
        let mut result: Solutions = Vec::with_capacity(8);

        // Debug check. Solution failing cross-verification is flagged
        // as invalid. This loop also normalizes valid solutions to 0
        for (si, mut solution) in self.inverse_candidates(pose, near).enumerate() {
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
                let check_pose = self.forward(&solution);
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

    fn inverse_intern_5_dof(&self, pose: &Pose, j6: f64, near: &J4J6Near) -> Solutions {
        let mut result: Solutions = Vec::with_capacity(8);

        // Debug check. Solution failing cross-verification is flagged
        // as invalid. This loop also normalizes valid solutions to 0
        for (si, mut solution) in self.inverse_candidates(pose, near).enumerate() {
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
                let check_xyz = self.forward(&solution).translation;
                if Self::compare_xyz_only(&pose.translation, &check_xyz, self.distance_tolerance) {
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

    /// Sorts the solutions vector by closeness to the `previous` joint.
    /// Joints must be pre-normalized to be as close as possible, not away by 360 degrees
    fn sort_by_closeness(&self, solutions: &mut Solutions, previous: &Joints) {
        let sorting_weight = self
            .constraints
            .as_ref()
            .map_or(BY_PREV, |c| c.sorting_weight);
        if sorting_weight == BY_PREV {
            // If no constraints or they weight is zero, use simpler version
            solutions.sort_by(|a, b| {
                let distance_a = calculate_distance(a, previous);
                let distance_b = calculate_distance(b, previous);
                distance_a
                    .partial_cmp(&distance_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        } else {
            let constraints = self.constraints.as_ref().unwrap();
            solutions.sort_by(|a, b| {
                let prev_a;
                let prev_b;
                if sorting_weight != BY_CONSTRAINS {
                    prev_a = calculate_distance(a, previous);
                    prev_b = calculate_distance(b, previous);
                } else {
                    // Do not calculate unneeded distances if these values are to be ignored.
                    prev_a = 0.0;
                    prev_b = 0.0;
                }

                let constr_a = calculate_distance(a, &constraints.centers);
                let constr_b = calculate_distance(b, &constraints.centers);

                let distance_a = prev_a * (1.0 - sorting_weight) + constr_a * sorting_weight;
                let distance_b = prev_b * (1.0 - sorting_weight) + constr_b * sorting_weight;
                distance_a
                    .partial_cmp(&distance_b)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });
        }
    }

    /// Get constraint centers in case we have the already constructed instance of the
    fn constraint_centers(&self) -> &Joints {
        self.constraints
            .as_ref()
            .map_or(&JOINTS_AT_ZERO, |c| &c.centers)
    }
}

/// Normalizes the angle `now` to be as close as possible to `must_be_near`
///
/// # Arguments
///
/// * `now` - A mutable reference to the angle to be normalized, radians
/// * `must_be_near` - The reference angle, radians
fn normalize_near(now: &mut f64, must_be_near: f64) {
    let two_pi = 2.0 * PI;
    // Smallest signed difference in (-π, π]
    let diff = (*now - must_be_near + PI).rem_euclid(two_pi) - PI;
    *now = must_be_near + diff;
}

fn calculate_distance(joint1: &Joints, joint2: &Joints) -> f64 {
    joint1
        .iter()
        .zip(joint2.iter())
        .map(|(a, b)| (a - b).abs())
        .sum()
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

fn compare_poses(ta: &Pose, tb: &Pose, distance_tolerance: f64, angular_tolerance: f64) -> bool {
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

#[cfg(test)]
#[path = "wrist_singularity_tests.rs"]
mod wrist_singularity_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematic_traits::{J5, Joints, Kinematics};
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;

    fn scale_geometry(mut parameters: Parameters, scale: f64) -> Parameters {
        parameters.a1 *= scale;
        parameters.a2 *= scale;
        parameters.b *= scale;
        parameters.c1 *= scale;
        parameters.c2 *= scale;
        parameters.c3 *= scale;
        parameters.c4 *= scale;
        parameters
    }

    #[test]
    fn distance_tolerance_scales_with_robot_geometry() {
        let parameters = Parameters::irb2400_10();
        let robot = OPWKinematics::new(parameters);
        let expected = 2.395 * RELATIVE_DISTANCE_TOLERANCE;
        assert!(
            (robot.distance_tolerance - expected).abs() <= expected * f64::EPSILON,
            "distance tolerance {} does not match expected {}",
            robot.distance_tolerance,
            expected
        );

        let scale = 1_000_000.0;
        let scaled_parameters = scale_geometry(parameters, scale);
        let scaled_robot = OPWKinematics::new(scaled_parameters);
        let expected_scaled = robot.distance_tolerance * scale;
        assert!(
            (scaled_robot.distance_tolerance - expected_scaled).abs()
                <= expected_scaled * f64::EPSILON,
            "scaled distance tolerance {} does not match expected {}",
            scaled_robot.distance_tolerance,
            expected_scaled
        );

        let constraints = Constraints::new([0.0; 6], [0.0; 6], BY_PREV);
        let constrained_robot = OPWKinematics::new_with_constraints(scaled_parameters, constraints);
        assert_eq!(
            constrained_robot.distance_tolerance,
            scaled_robot.distance_tolerance
        );
    }

    #[test]
    fn inverse_continuing_scales_singularity_recovery_with_geometry() {
        let parameters = scale_geometry(Parameters::irb2400_10(), 1_000.0);
        let robot = OPWKinematics::new(parameters);
        let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.4];
        let target: Joints = [0.0, 0.1, 0.2, 0.5, 0.0, 0.6];
        let pose = robot.forward(&target);

        let base = robot.inverse(&pose);
        let continuing = robot.inverse_continuing(&pose, &previous);
        assert_eq!(
            continuing.len(),
            base.len(),
            "singularity recovery should use existing wrist branches for scaled geometry"
        );

        let recovered = continuing
            .iter()
            .find(|solution| solution[J5].abs() < ANGULAR_TOLERANCE)
            .expect("expected a recovered J5≈0 solution for scaled geometry");
        let resolved_pose = robot.forward(recovered);
        let translation_error = (resolved_pose.translation - pose.translation).length();
        let angular_error = resolved_pose.angular_distance(pose);
        assert!(
            translation_error <= robot.distance_tolerance,
            "resolved FK translation error {} exceeds scaled tolerance {}",
            translation_error,
            robot.distance_tolerance
        );
        assert!(
            angular_error <= ANGULAR_TOLERANCE,
            "resolved FK angular error {} exceeds tolerance {}",
            angular_error,
            ANGULAR_TOLERANCE
        );
    }

    #[test]
    fn inverse_cross_validation_scales_with_robot_geometry() {
        let parameters = Parameters::irb2400_10();
        let scaled_parameters = scale_geometry(parameters, 1E12);
        let robot = OPWKinematics::new(parameters);
        let scaled_robot = OPWKinematics::new(scaled_parameters);
        let joints: Joints = [0.37, -0.61, 0.83, -1.11, 0.72, 1.39];

        let solutions = robot.inverse(&robot.forward(&joints));
        let scaled_pose = scaled_robot.forward(&joints);
        let scaled_solutions = scaled_robot.inverse(&scaled_pose);
        assert!(
            !solutions.is_empty(),
            "baseline inverse returned no solutions"
        );
        assert_eq!(
            scaled_solutions.len(),
            solutions.len(),
            "inverse solution count changed when the robot geometry was scaled"
        );

        for solution in scaled_solutions {
            let resolved_pose = scaled_robot.forward(&solution);
            let translation_error = (resolved_pose.translation - scaled_pose.translation).length();
            assert!(
                translation_error <= scaled_robot.distance_tolerance,
                "resolved FK translation error {} exceeds scaled tolerance {}",
                translation_error,
                scaled_robot.distance_tolerance
            );
        }
    }

    #[test]
    fn inverse_5dof_cross_validation_scales_with_robot_geometry() {
        let parameters = Parameters::irb2400_10();
        let scaled_parameters = scale_geometry(parameters, 1E12);
        let robot = OPWKinematics::new(parameters);
        let scaled_robot = OPWKinematics::new(scaled_parameters);
        let joints: Joints = [0.37, -0.61, 0.83, -1.11, 0.72, 1.39];

        let solutions = robot.inverse_5dof(&robot.forward(&joints), joints[J6]);
        let scaled_pose = scaled_robot.forward(&joints);
        let scaled_solutions = scaled_robot.inverse_5dof(&scaled_pose, joints[J6]);
        assert!(
            !solutions.is_empty(),
            "baseline 5-DOF inverse returned no solutions"
        );
        assert_eq!(
            scaled_solutions.len(),
            solutions.len(),
            "5-DOF inverse solution count changed when the robot geometry was scaled"
        );

        for solution in scaled_solutions {
            let resolved_translation = scaled_robot.forward(&solution).translation;
            let translation_error = (resolved_translation - scaled_pose.translation).length();
            assert!(
                translation_error <= scaled_robot.distance_tolerance,
                "resolved 5-DOF FK translation error {} exceeds scaled tolerance {}",
                translation_error,
                scaled_robot.distance_tolerance
            );
        }
    }

    #[test]
    fn wrist_branch_handles_cosine_roundoff_at_both_poles() {
        let arm = ArmBranch {
            q1: 0.0,
            q2: 0.0,
            q3: 0.0,
        };
        let near = J4J6Near { j4: 0.0, j6: 0.0 };
        for (rotation, cosine, expected_q5) in [
            (DMat3::IDENTITY, 1.0 + f64::EPSILON, 0.0),
            (DMat3::from_rotation_y(PI), -1.0 - f64::EPSILON, PI),
        ] {
            let mut matrix = RotationMatrix { matrix: rotation };
            matrix.matrix.z_axis.z = cosine;
            for branch in OPWKinematics::wrist_branch(&matrix, arm, &near) {
                assert!(branch.q4.is_finite() && branch.q5.is_finite() && branch.q6.is_finite());
                assert_eq!(branch.q5.abs(), expected_q5);
            }
        }
    }

    #[test]
    fn test_inverse_continuing_large_j6_angles() {
        let robot = OPWKinematics::new(Parameters::irb2400_10());

        let angles_deg: [f64; 10] = [
            -90000.0, -9000.0, -900.0, -90.0, -9.0, 9.0, 90.0, 900.0, 9000.0, 90000.0,
        ];

        for &angle_deg in &angles_deg {
            let j6_rad = angle_deg.to_radians();

            let pose = robot.forward(&[0.0, 0.1, 0.2, 0.3, 0.1, j6_rad]);

            let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.1, j6_rad];
            let solutions = robot.inverse_continuing(&pose, &previous);

            assert!(
                !solutions.is_empty(),
                "No solutions found for angle {} degrees",
                angle_deg
            );

            let solution_j6 = solutions[0][J6];

            // Normalize near previous angle
            let mut normalized_solution_j6 = solution_j6;
            normalize_near(&mut normalized_solution_j6, previous[J6]);

            let diff = (normalized_solution_j6 - previous[J6]).abs();

            // Allow small epsilon due to floating-point errors
            assert!(
                diff < 1e-6,
                "J6 mismatch for angle {} degrees: difference was {} radians",
                angle_deg,
                diff
            );
        }
    }

    #[test]
    fn test_inverse_continuing_recovers_wrist_at_j5_pi() {
        use crate::kinematic_traits::{J4, J5, J6, Joints, Kinematics};
        use std::f64::consts::PI;

        // Use a known-good robot model; adjust if you prefer a different preset.
        let robot = OPWKinematics::new(Parameters::irb2400_10());

        // Previous configuration with the wrist at the π singularity.
        // For J5 ≈ π, the *orientation* depends on (J4 - J6),
        // and the continuity-preserving update is δ4 = -δ6.
        let previous: Joints = [0.0, 0.1, 0.2, 0.3, PI, -0.8];

        // Create a target pose by moving J4 and J6 in OPPOSITE directions by ±Δ
        // while keeping J5 at π. This changes (J4 - J6) by 2Δ.
        let delta = 0.20_f64;
        let target: Joints = [
            previous[0],
            previous[1],
            previous[2],
            previous[J4] + delta,
            previous[J5], // keep J5 at π
            previous[J6] - delta,
        ];

        // Pose generated from the "target" configuration
        let pose = robot.forward(&target);

        // Baseline: plain IK solutions (no continuity logic)
        let base = robot.inverse(&pose);
        assert!(
            !base.is_empty(),
            "baseline IK returned no solutions for the target pose"
        );

        // Continuation recovers the existing wrist branches near `previous`.
        let cont = robot.inverse_continuing(&pose, &previous);
        assert!(!cont.is_empty(), "inverse_continuing returned no solutions");
        assert_eq!(
            cont.len(),
            base.len(),
            "recovery at J5≈π should preserve the wrist branch count"
        );

        // The recovered wrist solution should sort closest to `previous`.
        let mut best = cont[0];
        for j in 0..6 {
            normalize_near(&mut best[j], previous[j]);
        }

        // Opposite-direction motion relative to previous: δ4 + δ6 ≈ 0
        let d4 = best[J4] - previous[J4];
        let d6 = best[J6] - previous[J6];
        assert!(
            (d4 + d6).abs() < 1e-6,
            "expected opposite-direction update at J5≈π, got δ4={} δ6={}",
            d4,
            d6
        );

        // And (J4 - J6) must match the pose-implied (target) value (mod 2π)
        let mut best_diff = best[J4] - best[J6];
        let target_diff = target[J4] - target[J6];
        normalize_near(&mut best_diff, target_diff);
        assert!(
            (best_diff - target_diff).abs() < 1e-6,
            "q4 - q6 mismatch: got {}, want {}",
            best_diff,
            target_diff
        );
    }

    #[test]
    fn test_inverse_continuing_recovers_wrist_at_j5_zero() {
        use crate::kinematic_traits::{J4, J5, J6, Joints, Kinematics};

        let robot = OPWKinematics::new(Parameters::irb2400_10());

        // At J5 = 0, the orientation depends on J4 + J6. The target changes
        // that sum while `previous` supplies the preferred wrist distribution.
        let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.0, 0.4];
        let delta = 0.20_f64;
        let target: Joints = [
            previous[0],
            previous[1],
            previous[2],
            previous[J4] + delta,
            previous[J5],
            previous[J6] + delta,
        ];
        let pose = robot.forward(&target);

        let base = robot.inverse(&pose);
        assert!(
            !base.is_empty(),
            "baseline IK returned no solutions for the target pose"
        );

        let cont = robot.inverse_continuing(&pose, &previous);
        assert_eq!(
            cont.len(),
            base.len(),
            "recovery at J5≈0 should preserve the wrist branch count"
        );

        // The continuity solution is the minimum joint-space change from
        // `previous`, so sorting should place it first.
        let mut best = cont[0];
        for j in 0..6 {
            normalize_near(&mut best[j], previous[j]);
        }

        assert!(
            best[J5].abs() < 1e-6,
            "expected a J5≈0 continuity solution, got J5={}",
            best[J5]
        );

        // The remap must move J4 and J6 in the same direction by equal
        // amounts. An opposite-sign update preserves the old sum and fails
        // the final forward-pose validation.
        let d4 = best[J4] - previous[J4];
        let d6 = best[J6] - previous[J6];
        assert!(
            (d4 - d6).abs() < 1e-6,
            "expected same-direction update at J5≈0, got δ4={} δ6={}",
            d4,
            d6
        );

        let mut best_sum = best[J4] + best[J6];
        let target_sum = target[J4] + target[J6];
        normalize_near(&mut best_sum, target_sum);
        assert!(
            (best_sum - target_sum).abs() < 1e-6,
            "q4 + q6 mismatch: got {}, want {}",
            best_sum,
            target_sum
        );

        let resolved_pose = robot.forward(&best);
        let translation_error = (resolved_pose.translation - pose.translation).length();
        let angular_error = resolved_pose.angular_distance(pose);
        assert!(
            translation_error <= robot.distance_tolerance,
            "resolved FK translation error {} exceeds tolerance {}",
            translation_error,
            robot.distance_tolerance
        );
        assert!(
            angular_error <= ANGULAR_TOLERANCE,
            "resolved FK angular error {} exceeds tolerance {}",
            angular_error,
            ANGULAR_TOLERANCE
        );
    }

    #[test]
    fn test_inverse_continuing_handles_non_finite_previous_joint() {
        let robot = OPWKinematics::new(Parameters::irb2400_10());
        let pose = robot.forward(&[0.0, 0.1, 0.2, 0.3, 0.1, 0.2]);
        let previous: Joints = [0.0, 0.1, 0.2, 0.3, 0.1, f64::INFINITY];

        let _ = robot.inverse_continuing(&pose, &previous);
    }
}

//! Provides implementation of inverse and direct kinematics.

use std::f64::{consts::PI};
use crate::kinematic_traits::{Kinematics, Solutions, Pose, Singularity, Joints, JOINTS_AT_ZERO};
use crate::kinematic_traits::{J4, J5, J6};
use crate::parameters::opw_kinematics::{Parameters};
use crate::utils::opw_kinematics::{is_valid};
use nalgebra::{Isometry3, Matrix3, OVector, Rotation3, Translation3, U3, Unit, UnitQuaternion,
               Vector3};
use crate::constraints::{BY_CONSTRAINS, BY_PREV, Constraints};

const DEBUG: bool = false;

#[derive(Debug, Copy, Clone)]
pub struct OPWKinematics {
    /// The parameters that were used to construct this solver.
    parameters: Parameters,
    constraints: Option<Constraints>,

    unit_z: Unit<OVector<f64, U3>>,
}

impl OPWKinematics {
    /// Creates a new `OPWKinematics` instance with the given parameters.
    #[allow(dead_code)]
    pub fn new(parameters: Parameters) -> Self {
        OPWKinematics {
            parameters,
            unit_z: Unit::new_normalize(Vector3::z_axis().into_inner()),
            constraints: None,
        }
    }

    /// Create a new instance that takes also Constraints.
    /// If constraints are set, all solutions returned by this solver are constraint compliant. 
    pub fn new_with_constraints(parameters: Parameters, constraints: Constraints) -> Self {
        OPWKinematics {
            parameters,
            unit_z: Unit::new_normalize(Vector3::z_axis().into_inner()),
            constraints: Some(constraints),
        }
    }
}


const MM: f64 = 0.001;
const DISTANCE_TOLERANCE: f64 = 0.001 * MM;
const ANGULAR_TOLERANCE: f64 = 1E-6;

// Use for singularity checks.
const SINGULARITY_ANGLE_THR: f64 = 0.01 * PI / 180.0;

impl Kinematics for OPWKinematics {
    /// Return the solution that is constraint compliant anv values are valid
    /// (no NaNs, etc) but otherwise not sorted.
    /// If this is 5 degree of freedom robot only, the 6 joint is set to 0.0
    /// The rotation of pose in this case is only approximate.
    fn inverse(&self, pose: &Pose) -> Solutions {
        if self.parameters.dof == 5 {
            // For 5 DOF robot, we can only do 5 DOF approximate inverse. 
            self.inverse_intern_5_dof(pose, f64::NAN)
        } else {
            self.filter_constraints_compliant(self.inverse_intern(&pose))
        }
    }

    // Replaces singularity with correct solution
    // If this is 5 degree of freedom robot only, the 6 joint is set to as it was previous.
    // The rotation of pose in this case is only approximate.    
    fn inverse_continuing(&self, pose: &Pose, prev: &Joints) -> Solutions {
        if self.parameters.dof == 5 {
            return self.inverse_intern_5_dof(pose, prev[5]);
        }

        let previous;
        if prev[0].is_nan() {
            // Special value CONSTRAINT_CENTERED has been used
            previous = self.constraint_centers();
        } else {
            previous = prev;
        }

        const SINGULARITY_SHIFT: f64 = DISTANCE_TOLERANCE / 8.;
        const SINGULARITY_SHIFTS: [[f64; 3]; 4] =
            [[0., 0., 0., ], [SINGULARITY_SHIFT, 0., 0.],
                [0., SINGULARITY_SHIFT, 0.], [0., 0., SINGULARITY_SHIFT]];

        let mut solutions: Vec<Joints> = Vec::with_capacity(9);
        let pt = pose.translation;

        let rotation = pose.rotation;
        'shifts: for d in SINGULARITY_SHIFTS {
            let shifted = Pose::from_parts(
                Translation3::new(pt.x + d[0], pt.y + d[1], pt.z + d[2]), rotation);
            let ik = self.inverse_intern(&shifted);
            // Self::dump_shifted_solutions(d, &ik);
            if solutions.is_empty() {
                // Unshifted version that comes first is always included into results
                solutions.extend(&ik);
            }

            for s_idx in 0..ik.len() {
                let singularity =
                    self.kinematic_singularity(&ik[s_idx]);
                if singularity.is_some() && is_valid(&ik[s_idx]) {
                    let s;
                    let s_n;
                    if let Some(Singularity::A) = singularity {
                        let mut now = ik[s_idx];
                        if are_angles_close(now[J5], 0.) {
                            // J5 = 0 singlularity, J4 and J6 rotate same direction
                            s = previous[J4] + previous[J6];
                            s_n = now[J4] + now[J6];
                        } else {
                            // J5 = -180 or 180 singularity, even if the robot would need
                            // specific design to rotate J5 to this angle without self-colliding.
                            // J4 and J6 rotate in opposite directions
                            s = previous[J4] - previous[J6];
                            s_n = now[J4] - now[J6];

                            // Fix J5 sign to match the previous
                            normalize_near(&mut now[J5], previous[J5]);
                        }

                        let mut angle = s_n - s;
                        while angle > PI {
                            angle -= 2.0 * PI;
                        }
                        while angle < -PI {
                            angle += 2.0 * PI;
                        }
                        let j_d = angle / 2.0;

                        now[J4] = previous[J4] + j_d;
                        now[J6] = previous[J6] + j_d;

                        // Check last time if the pose is ok
                        let check_pose = self.forward(&now);
                        if compare_poses(&pose, &check_pose, DISTANCE_TOLERANCE, ANGULAR_TOLERANCE) &&
                            self.constraints_compliant(now) {
                            // Guard against the case our solution is out of constraints.
                            solutions.push(now);
                            // We only expect one singularity case hence once we found, we can end
                            break 'shifts;
                        }
                    }

                    break;
                }
            }
        }
        // Before any sorting, normalize all angles to be close to
        // 'previous'
        for s_idx in 0..solutions.len() {
            for joint_idx in 0..6 {
                normalize_near(&mut solutions[s_idx][joint_idx], previous[joint_idx]);
            }
        }
        self.sort_by_closeness(&mut solutions, &previous);
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
        let (s1, c1) = (q1.sin(), q1.cos());
        let (s2, c2) = (q2.sin(), q2.cos());
        let (s3, c3) = (q3.sin(), q3.cos());
        let (s4, c4) = (q4.sin(), q4.cos());
        let (s5, c5) = (q5.sin(), q5.cos());
        let (s6, c6) = (q6.sin(), q6.cos());

        // Compute rotation matrix r_0c
        let r_0c = Matrix3::new(
            c1 * c2 * c3 - c1 * s2 * s3, -s1, c1 * c2 * s3 + c1 * s2 * c3,
            s1 * c2 * c3 - s1 * s2 * s3, c1, s1 * c2 * s3 + s1 * s2 * c3,
            -s2 * c3 - c2 * s3, 0.0, -s2 * s3 + c2 * c3,
        );

        // Compute rotation matrix r_ce
        let r_ce = Matrix3::new(
            c4 * c5 * c6 - s4 * s6, -c4 * c5 * s6 - s4 * c6, c4 * s5,
            s4 * c5 * c6 + c4 * s6, -s4 * c5 * s6 + c4 * c6, s4 * s5,
            -s5 * c6, s5 * s6, c5,
        );

        // Compute the final rotation matrix r_oe
        let r_oe = r_0c * r_ce;

        // Calculate the final translation
        let translation = Vector3::new(cx0, cy0, cz0) + p.c4 * r_oe * *self.unit_z;

        // Convert the rotation matrix to a quaternion
        let rotation = Rotation3::from_matrix_unchecked(r_oe);

        // Return the pose combining translation and rotation
        Pose::from_parts(
            Translation3::from(translation),
            UnitQuaternion::from_rotation_matrix(&rotation),
        )
    }

    fn forward_with_joint_poses(&self, joints: &Joints) -> [Pose; 6] {
        let p = &self.parameters;

        // Use joint angles directly as radians (no conversion needed)
        let q1 = joints[0] * p.sign_corrections[0] as f64 - p.offsets[0];
        let q2 = joints[1] * p.sign_corrections[1] as f64 - p.offsets[1];
        let q3 = joints[2] * p.sign_corrections[2] as f64 - p.offsets[2];
        let q4 = joints[3] * p.sign_corrections[3] as f64 - p.offsets[3];
        let q5 = joints[4] * p.sign_corrections[4] as f64 - p.offsets[4];
        let q6 = joints[5] * p.sign_corrections[5] as f64 - p.offsets[5];

        // Precompute sines and cosines for efficiency
        let (s1, c1) = (q1.sin(), q1.cos());
        let (s2, c2) = (q2.sin(), q2.cos());
        let (s3, c3) = (q3.sin(), q3.cos());
        let (s4, c4) = (q4.sin(), q4.cos());
        let (s5, c5) = (q5.sin(), q5.cos());
        let (s6, c6) = (q6.sin(), q6.cos());

        // Pose 1: Base link position
        let joint1_pos = Vector3::new(0.0, 0.0, p.c1); // Z-offset from the base height
        let joint1_rot = Rotation3::identity(); // No rotation for the base frame
        let pose1 = Pose::from_parts(Translation3::from(joint1_pos), UnitQuaternion::from_rotation_matrix(&joint1_rot));

        // Pose 2: Link 1 position (translation along x = a1, rotation around q1)
        let joint2_pos = Vector3::new(p.a1 * c1, p.a1 * s1, p.c1); // a1 affects X, base height remains Z
        let joint2_rot = Rotation3::from_euler_angles(q1, 0.0, 0.0); // Rotation around Z-axis (q1)
        let pose2 = Pose::from_parts(Translation3::from(joint2_pos), UnitQuaternion::from_rotation_matrix(&joint2_rot));

        // Pose 3: Link 2 position
        let (sin_q2, cos_q2)  = (q2.sin(), q2.cos());
        
        let cx1 = p.c2 * sin_q2 + p.a1;
        let cy1 = p.b; // Typically 0 for most robots
        let cz1 = p.c2 * cos_q2; // Move in Z by link length
        let joint3_pos = Vector3::new(
            cx1 * c1 - cy1 * s1,
            cx1 * s1 + cy1 * c1,
            cz1 + p.c1 // Add the base height
        );
        let joint3_rot = Rotation3::from_euler_angles(q1, q2, 0.0); // Rotation around Z and Y
        let pose3 = Pose::from_parts(Translation3::from(joint3_pos), UnitQuaternion::from_rotation_matrix(&joint3_rot));

        // Pose 4: Link 3 position (corrected calculation)
        let cx2 = p.a1 + p.c2 * sin_q2 + p.a2 * (q2 + q3).sin();
        let cz2 = p.c1 + p.c2 * cos_q2 + p.c3;
        let joint4_pos = Vector3::new(
            cx2 * c1 - cy1 * s1,
            cx2 * s1 + cy1 * c1,
            cz2 // Corrected Z position without excessive offset
        );
        let joint4_rot = Rotation3::from_euler_angles(q1, q2, q3); // Rotation around Z, Y, and additional Y (q3)
        let pose4 = Pose::from_parts(Translation3::from(joint4_pos - Vector3::new(0.0, 0.0, p.c3)), UnitQuaternion::from_rotation_matrix(&joint4_rot));

        // Pose 5: Link 4 position (No c4 applied here, just joint4_pos)
        let joint5_pos = joint4_pos; // Do not apply c4 here
        let joint5_rot = Rotation3::from_euler_angles(q1, q2, q3 + q4); // Adding q4 for rotation around X-axis
        let pose5 = Pose::from_parts(Translation3::from(joint5_pos), UnitQuaternion::from_rotation_matrix(&joint5_rot));

        // Pose 6: End-effector position (including c4 offset)
        let r_0c = Matrix3::new(
            c1 * c2 * c3 - c1 * s2 * s3, -s1, c1 * c2 * s3 + c1 * s2 * c3,
            s1 * c2 * c3 - s1 * s2 * s3, c1, s1 * c2 * s3 + s1 * s2 * c3,
            -s2 * c3 - c2 * s3, 0.0, -s2 * s3 + c2 * c3,
        );
        let r_ce = Matrix3::new(
            c4 * c5 * c6 - s4 * s6, -c4 * c5 * s6 - s4 * c6, c4 * s5,
            s4 * c5 * c6 + c4 * s6, -s4 * c5 * s6 + c4 * c6, s4 * s5,
            -s5 * c6, s5 * s6, c5,
        );
        let r_oe = r_0c * r_ce;

        // Apply the c4 offset for the final end-effector position
        let end_effector_pos = joint5_pos + r_oe * (p.c4 * *self.unit_z); // Apply c4 offset here for the wrist
        let end_effector_rot = Rotation3::from_matrix_unchecked(r_oe); // Final rotation
        let pose6 = Pose::from_parts(Translation3::from(end_effector_pos), UnitQuaternion::from_rotation_matrix(&end_effector_rot));

        // Return all 6 Poses, with Pose 6 including c4 offset
        [pose1, pose2, pose3, pose4, pose5, pose6]
    }
    fn inverse_5dof(&self, pose: &Pose, j6: f64) -> Solutions {
        self.filter_constraints_compliant(self.inverse_intern_5_dof(&pose, j6))
    }

    fn inverse_continuing_5dof(&self, pose: &Pose, prev: &Joints) -> Solutions {
        let previous;
        if prev[0].is_nan() {
            // Special value CONSTRAINT_CENTERED has been used
            previous = self.constraint_centers();
        } else {
            previous = prev;
        }

        let mut solutions = self.inverse_intern_5_dof(pose, prev[5]);

        // Before any sorting, normalize all angles to be close to
        // 'previous'
        for s_idx in 0..solutions.len() {
            for joint_idx in 0..6 {
                normalize_near(&mut solutions[s_idx][joint_idx], previous[joint_idx]);
            }
        }
        self.sort_by_closeness(&mut solutions, &previous);
        self.filter_constraints_compliant(solutions)
    }

    fn kinematic_singularity(&self, joints: &Joints) -> Option<Singularity> {
        if is_close_to_multiple_of_pi(joints[J5], SINGULARITY_ANGLE_THR) {
            Some(Singularity::A)
        } else {
            None
        }
    }
}

impl OPWKinematics {
    fn inverse_intern(&self, pose: &Pose) -> Solutions {
        let params = &self.parameters;

        // Adjust to wrist center
        let matrix = pose.rotation.to_rotation_matrix();
        let translation_vector = &pose.translation.vector; // Get the translation vector component
        let scaled_z_axis = params.c4 * matrix.transform_vector(&Vector3::z_axis()); // Scale and rotate the z-axis vector

        let c = translation_vector - scaled_z_axis;

        let nx1 = ((c.x * c.x + c.y * c.y) - params.b * params.b).sqrt() - params.a1;

        let tmp1 = c.y.atan2(c.x); // Rust's method call syntax for atan2(y, x)
        let tmp2 = params.b.atan2(nx1 + params.a1);

        let theta1_i = tmp1 - tmp2;
        let theta1_ii = tmp1 + tmp2 - PI;

        let tmp3 = c.z - params.c1; // Access z directly for nalgebra's Vector3
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

        let theta1_i_sin = theta1_i.sin();
        let theta1_i_cos = theta1_i.cos();
        let theta1_ii_sin = theta1_ii.sin();
        let theta1_ii_cos = theta1_ii.cos();

        // orientation part
        let sin1: [f64; 4] = [
            theta1_i_sin, theta1_i_sin, theta1_ii_sin, theta1_ii_sin,
        ];

        let cos1: [f64; 4] = [
            theta1_i_cos, theta1_i_cos, theta1_ii_cos, theta1_ii_cos
        ];

        let s23: [f64; 4] = [
            (theta2_i + theta3_i).sin(),
            (theta2_ii + theta3_ii).sin(),
            (theta2_iii + theta3_iii).sin(),
            (theta2_iv + theta3_iv).sin(),
        ];

        let c23: [f64; 4] = [
            (theta2_i + theta3_i).cos(),
            (theta2_ii + theta3_ii).cos(),
            (theta2_iii + theta3_iii).cos(),
            (theta2_iv + theta3_iv).cos(),
        ];

        let m: [f64; 4] = [
            matrix[(0, 2)] * s23[0] * cos1[0] + matrix[(1, 2)] * s23[0] * sin1[0] + matrix[(2, 2)] * c23[0],
            matrix[(0, 2)] * s23[1] * cos1[1] + matrix[(1, 2)] * s23[1] * sin1[1] + matrix[(2, 2)] * c23[1],
            matrix[(0, 2)] * s23[2] * cos1[2] + matrix[(1, 2)] * s23[2] * sin1[2] + matrix[(2, 2)] * c23[2],
            matrix[(0, 2)] * s23[3] * cos1[3] + matrix[(1, 2)] * s23[3] * sin1[3] + matrix[(2, 2)] * c23[3],
        ];

        let theta5_i = f64::atan2((1.0 - m[0] * m[0]).sqrt(), m[0]);
        let theta5_ii = f64::atan2((1.0 - m[1] * m[1]).sqrt(), m[1]);
        let theta5_iii = f64::atan2((1.0 - m[2] * m[2]).sqrt(), m[2]);
        let theta5_iv = f64::atan2((1.0 - m[3] * m[3]).sqrt(), m[3]);

        let theta5_v = -theta5_i;
        let theta5_vi = -theta5_ii;
        let theta5_vii = -theta5_iii;
        let theta5_viii = -theta5_iv;

        let theta4_i;
        let theta6_i;

        let theta4_iy = matrix[(1, 2)] * cos1[0] - matrix[(0, 2)] * sin1[0];
        let theta4_ix = matrix[(0, 2)] * c23[0] * cos1[0] + matrix[(1, 2)] * c23[0] * sin1[0] - matrix[(2, 2)] * s23[0];
        theta4_i = theta4_iy.atan2(theta4_ix);

        let theta6_iy = matrix[(0, 1)] * s23[0] * cos1[0] + matrix[(1, 1)] * s23[0] * sin1[0] + matrix[(2, 1)] * c23[0];
        let theta6_ix = -matrix[(0, 0)] * s23[0] * cos1[0] - matrix[(1, 0)] * s23[0] * sin1[0] - matrix[(2, 0)] * c23[0];
        theta6_i = theta6_iy.atan2(theta6_ix);

        let theta4_ii;
        let theta6_ii;

        let theta4_iiy = matrix[(1, 2)] * cos1[1] - matrix[(0, 2)] * sin1[1];
        let theta4_iix = matrix[(0, 2)] * c23[1] * cos1[1] + matrix[(1, 2)] * c23[1] * sin1[1] - matrix[(2, 2)] * s23[1];
        theta4_ii = theta4_iiy.atan2(theta4_iix);

        let theta6_iiy = matrix[(0, 1)] * s23[1] * cos1[1] + matrix[(1, 1)] * s23[1] * sin1[1] + matrix[(2, 1)] * c23[1];
        let theta6_iix = -matrix[(0, 0)] * s23[1] * cos1[1] - matrix[(1, 0)] * s23[1] * sin1[1] - matrix[(2, 0)] * c23[1];
        theta6_ii = theta6_iiy.atan2(theta6_iix);

        let theta4_iii;
        let theta6_iii;

        let theta4_iiiy = matrix[(1, 2)] * cos1[2] - matrix[(0, 2)] * sin1[2];
        let theta4_iiix = matrix[(0, 2)] * c23[2] * cos1[2] + matrix[(1, 2)] * c23[2] * sin1[2] - matrix[(2, 2)] * s23[2];
        theta4_iii = theta4_iiiy.atan2(theta4_iiix);

        let theta6_iiiy = matrix[(0, 1)] * s23[2] * cos1[2] + matrix[(1, 1)] * s23[2] * sin1[2] + matrix[(2, 1)] * c23[2];
        let theta6_iiix = -matrix[(0, 0)] * s23[2] * cos1[2] - matrix[(1, 0)] * s23[2] * sin1[2] - matrix[(2, 0)] * c23[2];
        theta6_iii = theta6_iiiy.atan2(theta6_iiix);

        let theta4_iv;
        let theta6_iv;

        let theta4_ivy = matrix[(1, 2)] * cos1[3] - matrix[(0, 2)] * sin1[3];
        let theta4_ivx = matrix[(0, 2)] * c23[3] * cos1[3] + matrix[(1, 2)] * c23[3] * sin1[3] - matrix[(2, 2)] * s23[3];
        theta4_iv = theta4_ivy.atan2(theta4_ivx);

        let theta6_ivy = matrix[(0, 1)] * s23[3] * cos1[3] + matrix[(1, 1)] * s23[3] * sin1[3] + matrix[(2, 1)] * c23[3];
        let theta6_ivx = -matrix[(0, 0)] * s23[3] * cos1[3] - matrix[(1, 0)] * s23[3] * sin1[3] - matrix[(2, 0)] * c23[3];
        theta6_iv = theta6_ivy.atan2(theta6_ivx);

        let theta4_v = theta4_i + PI;
        let theta4_vi = theta4_ii + PI;
        let theta4_vii = theta4_iii + PI;
        let theta4_viii = theta4_iv + PI;

        let theta6_v = theta6_i - PI;
        let theta6_vi = theta6_ii - PI;
        let theta6_vii = theta6_iii - PI;
        let theta6_viii = theta6_iv - PI;

        let theta: [[f64; 6]; 8] = [
            [theta1_i, theta2_i, theta3_i, theta4_i, theta5_i, theta6_i],
            [theta1_i, theta2_ii, theta3_ii, theta4_ii, theta5_ii, theta6_ii],
            [theta1_ii, theta2_iii, theta3_iii, theta4_iii, theta5_iii, theta6_iii],
            [theta1_ii, theta2_iv, theta3_iv, theta4_iv, theta5_iv, theta6_iv],
            [theta1_i, theta2_i, theta3_i, theta4_v, theta5_v, theta6_v],
            [theta1_i, theta2_ii, theta3_ii, theta4_vi, theta5_vi, theta6_vi],
            [theta1_ii, theta2_iii, theta3_iii, theta4_vii, theta5_vii, theta6_vii],
            [theta1_ii, theta2_iv, theta3_iv, theta4_viii, theta5_viii, theta6_viii],
        ];

        let mut sols: [[f64; 6]; 8] = [[f64::NAN; 6]; 8];
        for si in 0..sols.len() {
            for ji in 0..6 {
                sols[si][ji] = (theta[si][ji] + params.offsets[ji]) *
                    params.sign_corrections[ji] as f64;
            }
        }

        let mut result: Solutions = Vec::with_capacity(8);

        // Debug check. Solution failing cross-verification is flagged
        // as invalid. This loop also normalizes valid solutions to 0
        for si in 0..sols.len() {
            let mut valid = true;
            for ji in 0..6 {
                let mut angle = sols[si][ji];
                if angle.is_finite() {
                    while angle > PI {
                        angle -= 2.0 * PI;
                    }
                    while angle < -PI {
                        angle += 2.0 * PI;
                    }
                    sols[si][ji] = angle;
                } else {
                    valid = false;
                    break;
                }
            };
            if valid {
                let check_pose = self.forward(&sols[si]);
                if compare_poses(&pose, &check_pose, DISTANCE_TOLERANCE, ANGULAR_TOLERANCE) {
                    result.push(sols[si]);
                } else {
                    if DEBUG {
                        println!("********** Pose Failure sol {} *********", si);
                    }
                }
            }
        }

        result
    }

    fn inverse_intern_5_dof(&self, pose: &Pose, j6: f64) -> Solutions {
        let params = &self.parameters;

        // Adjust to wrist center
        let matrix = pose.rotation.to_rotation_matrix();
        let translation_vector = &pose.translation.vector; // Get the translation vector component
        let scaled_z_axis = params.c4 * matrix.transform_vector(&Vector3::z_axis()); // Scale and rotate the z-axis vector

        let c = translation_vector - scaled_z_axis;

        let nx1 = ((c.x * c.x + c.y * c.y) - params.b * params.b).sqrt() - params.a1;

        let tmp1 = c.y.atan2(c.x); // Rust's method call syntax for atan2(y, x)
        let tmp2 = params.b.atan2(nx1 + params.a1);

        let theta1_i = tmp1 - tmp2;
        let theta1_ii = tmp1 + tmp2 - PI;

        let tmp3 = c.z - params.c1; // Access z directly for nalgebra's Vector3
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

        let theta1_i_sin = theta1_i.sin();
        let theta1_i_cos = theta1_i.cos();
        let theta1_ii_sin = theta1_ii.sin();
        let theta1_ii_cos = theta1_ii.cos();

        // orientation part
        let sin1: [f64; 4] = [
            theta1_i_sin, theta1_i_sin, theta1_ii_sin, theta1_ii_sin,
        ];

        let cos1: [f64; 4] = [
            theta1_i_cos, theta1_i_cos, theta1_ii_cos, theta1_ii_cos
        ];

        let s23: [f64; 4] = [
            (theta2_i + theta3_i).sin(),
            (theta2_ii + theta3_ii).sin(),
            (theta2_iii + theta3_iii).sin(),
            (theta2_iv + theta3_iv).sin(),
        ];

        let c23: [f64; 4] = [
            (theta2_i + theta3_i).cos(),
            (theta2_ii + theta3_ii).cos(),
            (theta2_iii + theta3_iii).cos(),
            (theta2_iv + theta3_iv).cos(),
        ];

        let m: [f64; 4] = [
            matrix[(0, 2)] * s23[0] * cos1[0] + matrix[(1, 2)] * s23[0] * sin1[0] + matrix[(2, 2)] * c23[0],
            matrix[(0, 2)] * s23[1] * cos1[1] + matrix[(1, 2)] * s23[1] * sin1[1] + matrix[(2, 2)] * c23[1],
            matrix[(0, 2)] * s23[2] * cos1[2] + matrix[(1, 2)] * s23[2] * sin1[2] + matrix[(2, 2)] * c23[2],
            matrix[(0, 2)] * s23[3] * cos1[3] + matrix[(1, 2)] * s23[3] * sin1[3] + matrix[(2, 2)] * c23[3],
        ];

        let theta5_i = f64::atan2((1.0 - m[0] * m[0]).sqrt(), m[0]);
        let theta5_ii = f64::atan2((1.0 - m[1] * m[1]).sqrt(), m[1]);
        let theta5_iii = f64::atan2((1.0 - m[2] * m[2]).sqrt(), m[2]);
        let theta5_iv = f64::atan2((1.0 - m[3] * m[3]).sqrt(), m[3]);

        let theta5_v = -theta5_i;
        let theta5_vi = -theta5_ii;
        let theta5_vii = -theta5_iii;
        let theta5_viii = -theta5_iv;

        let theta4_i;

        let theta4_iy = matrix[(1, 2)] * cos1[0] - matrix[(0, 2)] * sin1[0];
        let theta4_ix = matrix[(0, 2)] * c23[0] * cos1[0] + matrix[(1, 2)] * c23[0] * sin1[0] - matrix[(2, 2)] * s23[0];
        theta4_i = theta4_iy.atan2(theta4_ix);

        let theta4_ii;

        let theta4_iiy = matrix[(1, 2)] * cos1[1] - matrix[(0, 2)] * sin1[1];
        let theta4_iix = matrix[(0, 2)] * c23[1] * cos1[1] + matrix[(1, 2)] * c23[1] * sin1[1] - matrix[(2, 2)] * s23[1];
        theta4_ii = theta4_iiy.atan2(theta4_iix);

        let theta4_iii;

        let theta4_iiiy = matrix[(1, 2)] * cos1[2] - matrix[(0, 2)] * sin1[2];
        let theta4_iiix = matrix[(0, 2)] * c23[2] * cos1[2] + matrix[(1, 2)] * c23[2] * sin1[2] - matrix[(2, 2)] * s23[2];
        theta4_iii = theta4_iiiy.atan2(theta4_iiix);

        let theta4_iv;

        let theta4_ivy = matrix[(1, 2)] * cos1[3] - matrix[(0, 2)] * sin1[3];
        let theta4_ivx = matrix[(0, 2)] * c23[3] * cos1[3] + matrix[(1, 2)] * c23[3] * sin1[3] - matrix[(2, 2)] * s23[3];
        theta4_iv = theta4_ivy.atan2(theta4_ivx);

        let theta4_v = theta4_i + PI;
        let theta4_vi = theta4_ii + PI;
        let theta4_vii = theta4_iii + PI;
        let theta4_viii = theta4_iv + PI;

        let theta: [[f64; 5]; 8] = [
            [theta1_i, theta2_i, theta3_i, theta4_i, theta5_i],
            [theta1_i, theta2_ii, theta3_ii, theta4_ii, theta5_ii],
            [theta1_ii, theta2_iii, theta3_iii, theta4_iii, theta5_iii],
            [theta1_ii, theta2_iv, theta3_iv, theta4_iv, theta5_iv],
            [theta1_i, theta2_i, theta3_i, theta4_v, theta5_v],
            [theta1_i, theta2_ii, theta3_ii, theta4_vi, theta5_vi],
            [theta1_ii, theta2_iii, theta3_iii, theta4_vii, theta5_vii],
            [theta1_ii, theta2_iv, theta3_iv, theta4_viii, theta5_viii],
        ];

        let mut sols: [[f64; 6]; 8] = [[f64::NAN; 6]; 8];
        for si in 0..sols.len() {
            for ji in 0..5 {
                sols[si][ji] = (theta[si][ji] + params.offsets[ji]) *
                    params.sign_corrections[ji] as f64;
            }
            sols[si][5] = j6 // J6 goes directly to response and is not more adjusted
        }

        let mut result: Solutions = Vec::with_capacity(8);

        // Debug check. Solution failing cross-verification is flagged
        // as invalid. This loop also normalizes valid solutions to 0
        for si in 0..sols.len() {
            let mut valid = true;
            for ji in 0..5 { // J6 is not processed in this loop
                let mut angle = sols[si][ji];
                if angle.is_finite() {
                    while angle > PI {
                        angle -= 2.0 * PI;
                    }
                    while angle < -PI {
                        angle += 2.0 * PI;
                    }
                    sols[si][ji] = angle;
                } else {
                    valid = false;
                    break;
                }
            };
            if valid {
                let check_xyz = self.forward(&sols[si]).translation;
                if Self::compare_xyz_only(&pose.translation, &check_xyz, DISTANCE_TOLERANCE) {
                    result.push(sols[si]);
                } else {
                    if DEBUG {
                        println!("********** Pose Failure 5DOF sol {} *********", si);
                    }
                }
            }
        }

        result
    }

    fn compare_xyz_only(pose_translation: &Translation3<f64>, check_xyz: &Translation3<f64>, tolerance: f64) -> bool {
        (pose_translation.vector - check_xyz.vector).norm() <= tolerance
    }

    fn filter_constraints_compliant(&self, solutions: Solutions) -> Solutions {
        match &self.constraints {
            Some(constraints) => constraints.filter(&solutions),
            None => solutions
        }
    }

    fn constraints_compliant(&self, solution: Joints) -> bool {
        match &self.constraints {
            Some(constraints) => constraints.compliant(&solution),
            None => true
        }
    }

    /// Sorts the solutions vector by closeness to the `previous` joint.
    /// Joints must be pre-normalized to be as close as possible, not away by 360 degrees
    fn sort_by_closeness(&self, solutions: &mut Solutions, previous: &Joints) {
        let sorting_weight = self.constraints.as_ref()
            .map_or(BY_PREV, |c| c.sorting_weight);
        if sorting_weight == BY_PREV {
            // If no constraints or they weight is zero, use simpler version
            solutions.sort_by(|a, b| {
                let distance_a = calculate_distance(a, previous);
                let distance_b = calculate_distance(b, previous);
                distance_a.partial_cmp(&distance_b).unwrap_or(std::cmp::Ordering::Equal)
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
                distance_a.partial_cmp(&distance_b).unwrap_or(std::cmp::Ordering::Equal)
            });
        }
    }

    /// Get constraint centers in case we have the already constructed instance of the
    fn constraint_centers(&self) -> &Joints {
        self.constraints.as_ref()
            .map_or(&JOINTS_AT_ZERO, |c| &c.centers)
    }
}

// Adjusted helper function to check for n*pi where n is any integer
fn is_close_to_multiple_of_pi(joint_value: f64, threshold: f64) -> bool {

    // Normalize angle within [0, 2*PI)
    let normalized_angle = joint_value.rem_euclid(2.0 * PI);
    // Check if the normalized angle is close to 0 or PI
    normalized_angle < threshold ||
        (PI - normalized_angle).abs() < threshold
}

fn are_angles_close(angle1: f64, angle2: f64) -> bool {
    let mut diff = (angle1 - angle2).abs();
    diff = diff % (2.0 * PI);
    while diff > PI {
        diff = (2.0 * PI) - diff;
    }
    diff < SINGULARITY_ANGLE_THR
}

/// Normalizes the angle `now` to be as close as possible to `prev`
///
/// # Arguments
///
/// * `now` - A mutable reference to the angle to be normalized, radians
/// * `prev` - The reference angle, radians
fn normalize_near(now: &mut f64, must_be_near: f64) {
    let two_pi = 2.0 * PI;

    fn adjust(now: &mut f64, prev: f64, two_pi: f64) {
        if (*now - prev).abs() > ((*now - two_pi) - prev).abs() {
            *now -= two_pi;
        }
        if (*now - prev).abs() > ((*now + two_pi) - prev).abs() {
            *now += two_pi;
        }
        // Handle case -pi and pi that are identical angles
        if (*now).abs() == PI && (prev.signum() != (*now).signum()) {
            *now = -*now;
        }
    }

    // Perform the adjustment potentially twice to ensure minimum difference
    adjust(now, must_be_near, two_pi);
    adjust(now, must_be_near, two_pi);
}


fn calculate_distance(joint1: &Joints, joint2: &Joints) -> f64 {
    joint1.iter()
        .zip(joint2.iter())
        .map(|(a, b)| (a - b).abs())
        .sum()
}

// Compare two poses with the given tolerance.
fn compare_poses(ta: &Isometry3<f64>, tb: &Isometry3<f64>,
                 distance_tolerance: f64, angular_tolerance: f64) -> bool {
    let translation_distance = (ta.translation.vector - tb.translation.vector).norm();
    let angular_distance = ta.rotation.angle_to(&tb.rotation);

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

#[allow(dead_code)]
fn dump_shifted_solutions(d: [f64; 3], ik: &Solutions) {
    println!("Shifted solutions {} {} {}", d[0], d[1], d[2]);
    for sol_idx in 0..ik.len() {
        let mut row_str = String::new();
        for joint_idx in 0..6 {
            let computed = ik[sol_idx][joint_idx];
            row_str.push_str(&format!("{:5.2} ", computed.to_degrees()));
        }
        println!("[{}]", row_str.trim_end()); // Trim trailing space for aesthetics
    }
}

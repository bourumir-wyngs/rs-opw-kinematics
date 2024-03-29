use crate::kinematic_traits::kinematics_traits::{Kinematics, Solutions, Pose};
use crate::parameters::opw_kinematics::Parameters;
use nalgebra::{Isometry3, Matrix3, Quaternion, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};

struct OPWKinematics {
    parameters: Parameters,
}

impl OPWKinematics {
    /// Creates a new `OPWKinematics` instance with the given parameters.
    pub fn new(parameters: Parameters) -> Self {
        OPWKinematics { parameters }
    }
}

// Compare two poses with the given tolerance.
fn compare_poses(ta: &Isometry3<f64>, tb: &Isometry3<f64>, tolerance: f64) -> bool {
    let translation_distance = (ta.translation.vector - tb.translation.vector).norm();
    let angular_distance = ta.rotation.angle_to(&tb.rotation);

    if translation_distance.abs() > tolerance {
        println!("Translation Error: {}", translation_distance);
        return false;
    }

    if angular_distance.abs() > tolerance {
        println!("Angular Error: {}", angular_distance);
        return false;
    }
    true
}


impl Kinematics for OPWKinematics {
    fn inverse(&self, pose: &Pose) -> Solutions {
        let mut solutions: Solutions = Solutions::from_element(f64::NAN);
        unimplemented!();
        solutions
    }

    fn forward(&self, joints: &[f64; 6]) -> Pose {
        let mut q = [0.0; 6];
        let p = &self.parameters;

        for i in 0..6 {
            q[i] = joints[i] * p.sign_corrections[i] as f64 - p.offsets[i];
        }

        let psi3 = f64::atan2(p.a2, p.c3);
        let k = f64::sqrt(p.a2 * p.a2 + p.c3 * p.c3);

        let cx1 = p.c2 * f64::sin(q[1]) + k * f64::sin(q[1] + q[2] + psi3) + p.a1;
        let cy1 = p.b;
        let cz1 = p.c2 * f64::cos(q[1]) + k * f64::cos(q[1] + q[2] + psi3);

        let cx0 = cx1 * f64::cos(q[0]) - cy1 * f64::sin(q[0]);
        let cy0 = cx1 * f64::sin(q[0]) + cy1 * f64::cos(q[0]);
        let cz0 = cz1 + p.c1;

        let s1 = f64::sin(q[0]);
        let s2 = f64::sin(q[1]);
        let s3 = f64::sin(q[2]);
        let s4 = f64::sin(q[3]);
        let s5 = f64::sin(q[4]);
        let s6 = f64::sin(q[5]);

        let c1 = f64::cos(q[0]);
        let c2 = f64::cos(q[1]);
        let c3 = f64::cos(q[2]);
        let c4 = f64::cos(q[3]);
        let c5 = f64::cos(q[4]);
        let c6 = f64::cos(q[5]);

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

        let unit_z = Unit::new_normalize(Vector3::z_axis().into_inner());
        let translation = Vector3::new(cx0, cy0, cz0) + p.c4 * r_oe * *unit_z;
        let rotation = Rotation3::from_matrix_unchecked(r_oe);

        Pose::from_parts(Translation3::from(translation),
                         UnitQuaternion::from_rotation_matrix(&rotation))
    }
}

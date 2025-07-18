#[cfg(test)]
mod tests {
    use std::f64::consts::PI;
    use crate::constraints::{BY_CONSTRAINS, Constraints};
    use crate::kinematic_traits::{Kinematics, Solutions};
    use crate::kinematics_impl::OPWKinematics;
    use crate::parameters::opw_kinematics::Parameters;
    use crate::utils::{as_radians, dump_solutions, dump_solutions_degrees};

    #[test]
    fn test_constraints() {
        // This robot has both A and B type singularity
        // B type singularity two angles, maestro
        let parameters = Parameters::staubli_tx2_160l();
        
        let not_below = as_radians([9,  18, 28, 38, -5, 55]);        
        let joints = as_radians(   [10, 20, 30, 40,  0, 60]);        
        let not_above = as_radians([11, 22, 33, 44,  5, 65]);


        //let robot = OPWKinematics::new_with_constraints(parameters,
        //  Constraints::new(not_below, not_above, BY_CONSTRAINS));
        let robot = OPWKinematics::new(parameters.clone());
        let pose = robot.forward(&joints);

        let solutions = robot.inverse_continuing(&pose, &joints);
        verify(&solutions, vec!(
            [10.00, 20.00, 30.00, 40.00, 0.00, 60.00],
            [10.00, 51.75, -30.00, -0.00, 28.25, 100.00],
            [10.00, 51.75, -30.00, 180.00, -28.25, -80.00]
        ));

        let constraints = Constraints::new(not_below, not_above, BY_CONSTRAINS);
        assert!(constraints.compliant(&joints));
        let robot = OPWKinematics::new_with_constraints(
            parameters.clone(), constraints);
        let solutions = robot.inverse_continuing(&pose, &joints);
        verify(&solutions, vec!(
            [10.00, 20.00, 30.00, 40.00, 0.00, 60.00]
        ));
    }

    fn verify(actual: &Solutions, expected: Solutions) {
        let tolerance = 1.0;
        let mut solution_matches = true;

        if actual.len() != expected.len() {
            solution_matches = false;
        } else {
            for sol_idx in 0..expected.len() {
                for joint_idx in 0..6 {
                    let computed = actual[sol_idx][joint_idx].to_degrees();
                    let asserted = expected[sol_idx][joint_idx];

                    let diff = (computed - asserted).abs();
                    if diff >= tolerance && (diff - 2. * PI).abs() > tolerance {
                        // For angles, 360 degree difference means the same angle.
                        solution_matches = false;
                        break;
                    }
                }
            }
        }

        if !solution_matches {
            println!("Solutions do not match");
            println!("Expected:");
            dump_solutions_degrees(&expected);
            println!("Actual");
            dump_solutions(&actual);
            panic!("Solutions do not match");
        }
    }

    #[test]
    fn test_no_limits_accept_all_angles() {
        // from == to for each joint means no limits (tolerance is infinity)
        let from = [0.0; 6];
        let to = [0.0; 6];
        let constraints = Constraints::new(from, to, BY_CONSTRAINS);

        let samples = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [std::f64::consts::PI, -std::f64::consts::PI, 2.0 * std::f64::consts::PI,
             -2.0 * std::f64::consts::PI, 1.5 * std::f64::consts::PI, -1.5 * std::f64::consts::PI],
        ];

        for angles in samples.iter() {
            assert!(constraints.compliant(angles));
        }
    }
}
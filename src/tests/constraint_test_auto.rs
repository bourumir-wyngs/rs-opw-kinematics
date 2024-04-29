#[cfg(test)]
mod tests {
    extern crate rand;

    use rand::{Rng, SeedableRng};
    use rand::rngs::StdRng;
    use crate::constraints::{BY_CONSTRAINS, Constraints};
    use crate::utils::as_radians;

    // Define the data structure for a test case
    struct TestCase {
        id: usize,
        from_angles: [i32; 6],
        to_angles: [i32; 6],
        check_angles: [i32; 6],
        expected_results: [bool; 6],
        passing: bool,
    }

    impl TestCase {
        pub fn generate(id: usize, rng: &mut StdRng) -> Self {
            let mut from_angles = [0; 6];
            let mut to_angles = [0; 6];
            let mut check_angles = [0; 6];
            let mut expected_results = [false; 6];

            let mut all_passing = true;
            for i in 0..6 {
                let inside_range = rng.gen::<bool>();
                let a = rng.gen_range(1..359);
                let b = rng.gen_range(1..359);
                if inside_range {
                    from_angles[i] = a.min(b);
                    to_angles[i] = a.max(b);
                } else {
                    from_angles[i] = a.max(b);
                    to_angles[i] = a.min(b);
                }

                expected_results[i] = rng.gen_bool(0.95);
                if expected_results[i] {
                    if from_angles[i] <= to_angles[i] {
                        check_angles[i] = rng.gen_range(from_angles[i]..=to_angles[i]);
                    } else {
                        check_angles[i] = if rng.gen::<bool>() {
                            rng.gen_range(from_angles[i]..360)
                        } else {
                            rng.gen_range(0..=to_angles[i])
                        };
                    }
                } else {
                    if from_angles[i] <= to_angles[i] {
                        check_angles[i] = if rng.gen::<bool>() {
                            rng.gen_range(0..from_angles[i])
                        } else {
                            rng.gen_range(to_angles[i] + 1..360)
                        };
                    } else {
                        // Handle wrap-around failure properly
                        if to_angles[i] != 359 {
                            check_angles[i] = rng.gen_range(to_angles[i] + 1..360);
                        } else {
                            check_angles[i] = rng.gen_range(0..from_angles[i]);
                        }
                        all_passing = false;
                    }
                }
            }

            TestCase {
                id,
                from_angles,
                to_angles,
                check_angles,
                expected_results,
                passing: all_passing
            }
        }
    }

    #[test]
    fn test_generated_constraints() {
        let seed = [0u8; 32];
        let mut rng = StdRng::from_seed(seed);

        // Generate multiple test cases
        for id in 0..2048 {
            let case = TestCase::generate(id, &mut rng);
            let constraints = Constraints::new(
                as_radians(case.from_angles),
                as_radians(case.to_angles),
                BY_CONSTRAINS,
            );
            let actual = constraints.compliant(&as_radians(case.check_angles));
            if actual != case.passing {
                println!("Case mimatch: expected {}, actual {}", case.passing, actual);
                println!("ID: {}, From: {:?}, To: {:?}, Check: {:?}, Result: {:?} Passing {:?}",
                         case.id, case.from_angles, case.to_angles, case.check_angles,
                         case.expected_results, case.passing);
                panic!("Test case {} failed", case.id);
            }
        }
    }
}
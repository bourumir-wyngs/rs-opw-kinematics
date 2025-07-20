#[cfg(test)]
mod tests {
    extern crate rand;

    use std::mem;
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

            for i in 0..6 {
                // Generate two angles that are not equal
                let mut a: i32 = rng.gen_range(0..360);
                let mut b: i32 = rng.gen_range(0..360);

                while (a - b).abs() < 2 { // Leave at least on value in between
                    b = rng.gen_range(0..360); // generate again
                }

                // Make sure b > a
                if b < a {
                    mem::swap(&mut a, &mut b);
                }
                let c; // the value to check

                // Decide if the case should pass
                let pass = rng.gen_bool(0.95);

                // ordinary case, a < c < b to pass
                if pass {
                    // Generate c within the range from a to b boundaries exclusive
                    c = rng.gen_range(a + 1..b);
                } else {
                    c = loop {
                        // Generate outside the range, either below a or above b:
                        if rng.gen_bool(0.5) && a > 0 {
                            // below a
                            break rng.gen_range(0..a);
                        } else if b < 360 - 2 { // 360 and 359 would not generate as expected
                            // above b
                            break rng.gen_range(b + 1..360);
                        };
                    }
                }

                // Decide if we are doing the "wrap around 360 or 0 case" or ordinary case
                if rng.gen_bool(0.5) {
                    expected_results[i] = pass;
                    from_angles[i] = a;
                    to_angles[i] = b;
                    check_angles[i] = c;
                } else {
                    expected_results[i] = !pass;
                    from_angles[i] = b;
                    to_angles[i] = a;
                    check_angles[i] = c;
                }
            }

            TestCase {
                id,
                from_angles,
                to_angles,
                check_angles,
                expected_results,
                passing: expected_results.iter().all(|&val| val),
            }
        }
    }

    #[test]
    fn test_generated_constraints_0_360() {
        test_generated_constraints_shifted_by(0);
    }

    #[test]
    /// This tests checks the "shifted" case where both angles and constraints
    /// are in the range from -180 to 180
    fn test_generated_constraints_180_180() {
        test_generated_constraints_shifted_by(-180);
    }

    #[test]
    fn test_special_cases() {
        let cases: Vec<TestCase> = vec![
            TestCase {
                id: 0,
                from_angles: [-20; 6],
                to_angles: [190; 6],
                check_angles: [-19, -5, 0, 20, 100, 189],
                expected_results: [true; 6],
                passing: true,
            },
            TestCase {
                id: 1,
                from_angles: [-50; 6],
                to_angles: [260; 6],
                check_angles: [-49, -5, 0, 20, 200, 259],
                expected_results: [true; 6],
                passing: true,
            },
            TestCase {
                id: 2,
                from_angles: [-50; 6],
                to_angles: [260; 6],
                check_angles: [-51, -5, 0, 20, 200, 259],
                expected_results: [false, true, true, true, true, true],
                passing: false,
            },
            TestCase {
                id: 2,
                from_angles: [-50; 6],
                to_angles: [260; 6],
                check_angles: [261, -5, 0, 20, 200, 259],
                expected_results: [false, true, true, true, true, true],
                passing: false,
            },
            TestCase {
                id: 3,
                from_angles: [260; 6],
                to_angles: [-50; 6],
                check_angles: [-49, -5, 0, 20, 200, 259],
                expected_results: [false; 6],
                passing: false,
            },
        ];

        for case in cases {
            test_case(&case, 0);
        }
    }

    #[test]
    fn test_boundary_cases() {
        let cases: Vec<TestCase> = vec![
            TestCase {
                id: 0,
                from_angles: [-60; 6],
                to_angles: [60; 6],
                check_angles: [-60, 60, -59, 59, 0, 1],
                expected_results: [true; 6],
                passing: true,
            },
            TestCase {
                id: 1,
                from_angles: [-60; 6],
                to_angles: [60; 6],
                check_angles: [-61, 61, -69, 69, 180, 181],
                expected_results: [false; 6],
                passing: false,
            },
            
        ];

        for case in cases {
            test_case(&case, 0);
        }
    }

    #[test]
    fn test_over_360() {
        let cases: Vec<TestCase> = vec![
            TestCase {
                id: 0,
                from_angles: [-60; 6],
                to_angles: [60; 6],
                check_angles: [-60 + 360, 60, -59 + 360, 59, 0, 0],
                expected_results: [true; 6],
                passing: true,
            },
        ];

        for case in cases {
            test_case(&case, 0);
        }
    }


    fn test_generated_constraints_shifted_by(offset: i32) {
        let seed = [0u8; 32];
        let mut rng = StdRng::from_seed(seed);

        // Generate multiple test cases
        for id in 0..2048 {
            let case = TestCase::generate(id, &mut rng);
            test_case(&case, offset);
        }
    }

    fn test_case(case: &TestCase, offset: i32) {
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
            println!("Deep check");

            // To make analysis of the glitch easier, we set contraints and all angles
            // as one
            for p in 0..6 {
                let from = case.to_angles[p];
                let to = case.from_angles[p];
                let focused_constraints =
                    Constraints::new(
                        as_radians([to + offset; 6]),
                        as_radians([from + offset; 6]),
                        BY_CONSTRAINS);
                let angle = case.check_angles[p];
                let joints = as_radians([angle + offset; 6]);
                println!("{}: {} .. {} : {} ? = {}", p,
                         case.from_angles[p], case.to_angles[p], angle + offset,
                         focused_constraints.compliant(&joints));
            }
            panic!("Test case {} failed", case.id);
        }
    }
}

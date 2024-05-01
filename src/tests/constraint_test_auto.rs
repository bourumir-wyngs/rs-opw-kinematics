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

                // Decide if we are doing the "wrap arround 360 or 0 case" or ordinary case
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
                println!("Deep check");

                // To make analysis of the glitch easier, we set contraints and all angles
                // as one
                for p in 0..6 {
                    let focused_constraints =
                        Constraints::new(
                            as_radians([case.from_angles[p]; 6]),
                            as_radians([case.to_angles[p]; 6]),
                            BY_CONSTRAINS);
                    let joints = as_radians([case.check_angles[p]; 6]);
                    println!("{}: {} .. {} : {} ? = {}", p,
                             case.from_angles[p], case.to_angles[p], case.check_angles[p],
                             focused_constraints.compliant(&joints));
                }
                panic!("Test case {} failed", case.id);
            }
        }
    }

    #[test]
    /// This tests checks the "shifted" case where both angles and constraints
    /// are in the range from -180 to 180
    fn test_generated_constraints_shifted() {
        fn shift(angle: i32) -> i32 {
            // 0 .. 360 to - 180 .. 180 
            angle - 180
        }

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
                println!("Deep check");

                // To make analysis of the glitch easier, we set contraints and all angles
                // as one
                for p in 0..6 {
                    let focused_constraints =
                        Constraints::new(
                            as_radians([shift(case.from_angles[p]); 6]),
                            as_radians([shift(case.to_angles[p]); 6]),
                            BY_CONSTRAINS);
                    let joints = as_radians([shift(case.check_angles[p]); 6]);
                    println!("{}: {} .. {} : {} ? = {}", p,
                             case.from_angles[p], case.to_angles[p], shift(case.check_angles[p]),
                             focused_constraints.compliant(&joints));
                }
                panic!("Test case {} failed", case.id);
            }
        }
    }
}

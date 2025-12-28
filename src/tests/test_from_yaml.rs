
#[cfg(test)]
mod tests {
    use crate::parameters::opw_kinematics::Parameters;
    
    const READ_ERROR: &'static str = "Failed to load parameters from file";

    #[test]
    fn test_parameters_from_yaml() {
        let filename = "src/tests/data/fanuc/fanuc_m16ib20.yaml";
        let loaded =
            Parameters::from_yaml_file(filename).expect(READ_ERROR);

        let expected = Parameters {
            a1: 0.15,
            a2: -0.10,
            b: 0.0,
            c1: 0.525,
            c2: 0.77,
            c3: 0.74,
            c4: 0.10,
            offsets: [0.0, 0.0, -90.0_f64.to_radians(), 0.0, 0.0, 180.0_f64.to_radians()],
            sign_corrections: [1, 1, -1, -1, -1, -1],
            dof: 6,
        };


        assert_eq!(expected.a1, loaded.a1);
        assert_eq!(expected.a2, loaded.a2);
        assert_eq!(expected.b, loaded.b);
        assert_eq!(expected.c1, loaded.c1);
        assert_eq!(expected.c2, loaded.c2);
        assert_eq!(expected.c3, loaded.c3);
        assert_eq!(expected.c4, loaded.c4);
        assert_eq!(expected.offsets, loaded.offsets);
        assert_eq!(expected.sign_corrections, loaded.sign_corrections);
    }

    #[test]
    fn test_parameters_from_yaml_5dof() {
        let filename = "src/tests/data/fanuc/fanuc_m16ib20_5dof.yaml";
        let loaded =
            Parameters::from_yaml_file(filename).expect(READ_ERROR);

        let expected = Parameters {
            a1: 0.15,
            a2: -0.10,
            b: 0.0,
            c1: 0.525,
            c2: 0.77,
            c3: 0.74,
            c4: 0.10,
            offsets: [0.0, 0.0, -90.0_f64.to_radians(), 0.0, 180.0_f64.to_radians(), 0.0 ],
            sign_corrections: [1, 1, -1, -1, -1, 0], // Sign corrections last member 0
            dof: 5, // Degrees of freedom 5
        };


        assert_eq!(expected.a1, loaded.a1);
        assert_eq!(expected.a2, loaded.a2);
        assert_eq!(expected.b, loaded.b);
        assert_eq!(expected.c1, loaded.c1);
        assert_eq!(expected.c2, loaded.c2);
        assert_eq!(expected.c3, loaded.c3);
        assert_eq!(expected.c4, loaded.c4);
        assert_eq!(expected.offsets, loaded.offsets);
        assert_eq!(expected.sign_corrections, loaded.sign_corrections);
        assert_eq!(expected.dof, loaded.dof);
    }

    #[test]
    fn test_parameters_from_yaml_missing_sign_corrections() {
        let filename = "src/tests/data/fanuc/fanuc_m16ib20_no_sign.yaml";
        let loaded =
            Parameters::from_yaml_file(filename).expect(READ_ERROR);

        let expected = Parameters {
            a1: 0.15,
            a2: -0.10,
            b: 0.0,
            c1: 0.525,
            c2: 0.77,
            c3: 0.74,
            c4: 0.10,
            offsets: [0.0, 0.0, -90.0_f64.to_radians(), 0.0, 180.0_f64.to_radians(), 0.0],
            sign_corrections: [1, 1, 1, 1, 1, 1],
            dof: 6,
        };

        assert_eq!(expected.sign_corrections, loaded.sign_corrections);
        assert_eq!(expected.dof, loaded.dof);
    }

    #[test]
    fn test_parameters_from_yaml_rejects_bad_dof() {
        let filename = "src/tests/data/fanuc/invalid_dof.yaml";
        let err = Parameters::from_yaml_file(filename).unwrap_err();
        let msg = err.to_string();
        // println!("{msg}");
        assert!(msg.contains("^ validation error: greater than 6 for `dof`"), "{msg}");
    }

    #[test]
    fn test_parameters_from_yaml_rejects_bad_sign_correction() {
        let filename = "src/tests/data/fanuc/invalid_sign_corrections.yaml";
        let err = Parameters::from_yaml_file(filename).unwrap_err();
        let msg = err.to_string();
        // println!("{msg}");
        assert!(
            msg.contains("^ validation error: must be -1 or 1 for `opw_kinematics_joint_sign_corrections[2]`"),
            "{msg}"
        );
    }

    #[test]
    fn test_parameters_from_yaml_rejects_bad_offset_range() {
        let filename = "src/tests/data/fanuc/invalid_offsets.yaml";
        let err = Parameters::from_yaml_file(filename).unwrap_err();
        let msg = err.to_string();
        // println!("{msg}");
        assert!(
            msg.contains("^ validation error: must be within ["),
            "{msg}"
        );
    }

    #[test]
    fn test_parameters_from_yaml_rejects_nan_geometric_parameter() {
        let filename = "src/tests/data/fanuc/invalid_nan_geom.yaml";
        let err = Parameters::from_yaml_file(filename).unwrap_err();
        let msg = err.to_string();
        // println!("{msg}");
        assert!(msg.contains("^ validation error: must be finite for `opw_kinematics_geometric_parameters.a1`"), "{msg}");
    }
}

use std::fs::read_to_string;
use crate::kinematic_traits::{Joints};
use crate::parameters::opw_kinematics::Parameters;
use crate::urdf;
use crate::urdf::URDFParameters;

fn read_urdf(path: &str) -> URDFParameters {
    let opw_parameters = urdf::from_urdf(read_to_string(path)
                                             .expect("Failed to read test data file"), &None).expect("Faile to interpret URDF");
    // Output the results or further process
    println!("{:?}", opw_parameters);
    opw_parameters
}

#[test]
fn test_extraction_m10ia() {
    let opw_parameters = read_urdf("src/tests/data/fanuc/m10ia_macro.xacro");

    // opw_kinematics_geometric_parameters:
    //   a1: 0.15
    //   a2: -0.20
    //   b: 0.0
    //   c1: 0.45
    //   c2: 0.60
    //   c3: 0.64
    //   c4: 0.10
    // opw_kinematics_joint_offsets: [0.0, 0.0, deg(-90.0), 0.0, 0.0, deg(180.0)]
    // opw_kinematics_joint_sign_corrections: [1, 1, -1, -1, -1, -1]

    assert_eq!(opw_parameters.a1, 0.15, "a1 parameter mismatch");
    assert_eq!(opw_parameters.a2, -0.2, "a2 parameter mismatch");
    assert_eq!(opw_parameters.b, 0.0, "b parameter mismatch");
    assert_eq!(opw_parameters.c1, 0.45, "c1 parameter mismatch");
    assert_eq!(opw_parameters.c2, 0.6, "c2 parameter mismatch");
    assert_eq!(opw_parameters.c3, 0.64, "c3 parameter mismatch");
    assert_eq!(opw_parameters.c4, 0.1, "c4 parameter mismatch");

    let expected_sign_corrections: [i32; 6] = [1, 1, -1, -1, -1, -1];
    let expected_from: Joints = [-3.14, -1.57, -3.14, -3.31, -3.31, -6.28];
    let expected_to: Joints = [3.14, 2.79, 4.61, 3.31, 3.31, 6.28];

    for (i, &val) in expected_sign_corrections.iter().enumerate() {
        assert_eq!(opw_parameters.sign_corrections[i], val as i8,
                   "Mismatch in sign_corrections at index {}", i);
    }

    for (i, &val) in expected_from.iter().enumerate() {
        assert_eq!(opw_parameters.from[i], val, "Mismatch in from at index {}", i);
    }

    for (i, &val) in expected_to.iter().enumerate() {
        assert_eq!(opw_parameters.to[i], val, "Mismatch in to at index {}", i);
    }
}

#[test]
fn test_extraction_lrmate200ib() {
    let opw_parameters = read_urdf("src/tests/data/fanuc/lrmate200ib_macro.xacro");

    // opw_kinematics_geometric_parameters:
    //   a1: 0.15
    //   a2: -0.075
    //   b: 0.0
    //   c1: 0.35
    //   c2: 0.25
    //   c3: 0.290
    //   c4: 0.08
    // opw_kinematics_joint_offsets: [0.0, 0.0, deg(-90.0), 0.0, 0.0, deg(180.0)]
    // opw_kinematics_joint_sign_corrections: [1, 1, -1, -1, -1, -1]

    assert_eq!(opw_parameters.a1, 0.15, "a1 parameter mismatch");
    assert_eq!(opw_parameters.a2, -0.075, "a2 parameter mismatch");
    assert_eq!(opw_parameters.b, 0.0, "b parameter mismatch");
    assert_eq!(opw_parameters.c1, 0.35, "c1 parameter mismatch");
    assert_eq!(opw_parameters.c2, 0.25, "c2 parameter mismatch");
    assert_eq!(opw_parameters.c3, 0.290, "c3 parameter mismatch");
    assert_eq!(opw_parameters.c4, 0.08, "c4 parameter mismatch");

    let expected_sign_corrections: [i32; 6] = [1, 1, -1, -1, -1, -1];
    let expected_from: Joints = [-2.7925, -0.5759, -2.6145, -3.3161, -2.0943, -6.2831];
    let expected_to: Joints = [2.7925, 2.6529, 2.8797, 3.3161, 2.0943, 6.2831];

    for (i, &val) in expected_sign_corrections.iter().enumerate() {
        assert_eq!(opw_parameters.sign_corrections[i], val as i8,
                   "Mismatch in sign_corrections at index {}", i);
    }

    for (i, &val) in expected_from.iter().enumerate() {
        assert_eq!(opw_parameters.from[i], val, "Mismatch in constraints from at index {}", i);
    }

    for (i, &val) in expected_to.iter().enumerate() {
        assert_eq!(opw_parameters.to[i], val, "Mismatch in constraints to at index {}", i);
    }
}

#[test]
fn test_rx160() {
    let opw_parameters = Parameters::staubli_rx160();

    // opw_kinematics_geometric_parameters:
    //   a1: 0.15
    //   a2: 0.0
    //   b: 0.0
    //   c1: 0.55
    //   c2: 0.825
    //   c3: 0.625
    //   c4: 0.11
    // opw_kinematics_joint_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    // opw_kinematics_joint_sign_corrections: [1, 1, 1, 1, 1, 1]

    assert_eq!(opw_parameters.a1, 0.15, "a1 parameter mismatch");
    assert_eq!(opw_parameters.a2, 0.0, "a2 parameter mismatch");
    assert_eq!(opw_parameters.b, 0.0, "b parameter mismatch");
    assert_eq!(opw_parameters.c1, 0.55, "c1 parameter mismatch");
    assert_eq!(opw_parameters.c2, 0.825, "c2 parameter mismatch");
    assert_eq!(opw_parameters.c3, 0.625, "c3 parameter mismatch");
    assert_eq!(opw_parameters.c4, 0.11, "c4 parameter mismatch");

    let expected_sign_corrections: [i32; 6] = [1, 1, 1, 1, 1, 1];

    for (i, &val) in expected_sign_corrections.iter().enumerate() {
        assert_eq!(opw_parameters.sign_corrections[i], val as i8,
                   "Mismatch in sign_corrections at index {}", i);
    }
}

#[test]
fn test_extraction_m6ib() {
    let opw_parameters = read_urdf("src/tests/data/fanuc/m6ib_macro.xacro");

    // opw_kinematics_geometric_parameters:
    //  a1: 0.15
    //  a2: -0.10
    //  b: 0.0
    //  c1: 0.45
    //  c2: 0.600
    //  c3: 0.615
    //  c4: 0.10

    assert_eq!(opw_parameters.a1, 0.15, "a1 parameter mismatch");
    assert_eq!(opw_parameters.a2, -0.10, "a2 parameter mismatch");
    assert_eq!(opw_parameters.b, 0.0, "b parameter mismatch");
    assert_eq!(opw_parameters.c1, 0.45, "c1 parameter mismatch");
    assert_eq!(opw_parameters.c2, 0.6, "c2 parameter mismatch");
    assert_eq!(opw_parameters.c3, 0.615, "c3 parameter mismatch");
    assert_eq!(opw_parameters.c4, 0.10, "c4 parameter mismatch");
}

fn assert_parameter_extraction(yaml: Parameters, urdf: URDFParameters, robot: &str) {
    assert_eq!(urdf.a1, yaml.a1, "a1 parameter mismatch for {}", robot);
    assert_eq!(urdf.a2, yaml.a2, "a2 parameter mismatch for {}", robot);
    assert_eq!(urdf.b, yaml.b, "b parameter mismatch for {}", robot);
    assert_eq!(urdf.c1, yaml.c1, "c1 parameter mismatch for {}", robot);
    assert_eq!(urdf.c2, yaml.c2, "c2 parameter mismatch for {}", robot);
    assert_eq!(urdf.c3, yaml.c3, "c3 parameter mismatch for {}", robot);
    assert_eq!(urdf.c4, yaml.c4, "c4 parameter mismatch for {}", robot);
}

#[test]
fn test_extraction_kr6r700sixx() {
    let urdf =
        read_urdf("src/tests/data/kuka/kr6r700sixx_macro.xacro");

    let params = Parameters::kuka_kr6_r700_sixx();
    assert_parameter_extraction(params, urdf, "_kr6r700sixx");
}

#[test]
fn test_extraction_kr150() {
    let yaml = Parameters::from_yaml_file("\
    src/tests/data/kuka/opw_parameters_kr150r3100_2.yaml")
        .expect("Failed to read or parse URDF");
    let urdf = read_urdf("src/tests/data/kuka/kr150r3100_2_macro.xacro");

    assert_parameter_extraction(yaml, urdf, "kr150r3100_2");
}

#[test]
fn test_extraction_kr10r1420() {
    let yaml = Parameters::from_yaml_file("\
    src/tests/data/kuka/opw_parameters_kr10r1420.yaml")
        .expect("Failed to read or parse URDF");
    let urdf = read_urdf("src/tests/data/kuka/kr10r1420_macro.xacro");

    assert_parameter_extraction(yaml, urdf, "kr10r1420");
}

#[test]
fn test_extraction_kr5_arc() {
    let yaml = Parameters::from_yaml_file("\
    src/tests/data/kuka/opw_parameters_kr6r900_2.yaml")
        .expect("Failed to read or parse URDF");
    let urdf = read_urdf("src/tests/data/kuka/kr6r900_2_macro.xacro");

    assert_parameter_extraction(yaml, urdf, "kr6r900_2");
}

#[test]
fn test_extraction_m20ia() {
    let yaml = Parameters::from_yaml_file("\
    src/tests/data/fanuc/opw_parameters_m20ia.yaml")
        .expect("Failed to read or parse URDF");
    let urdf = read_urdf("src/tests/data/fanuc/m20ia_macro.xacro");

    assert_parameter_extraction(yaml, urdf, "m20ia");
}

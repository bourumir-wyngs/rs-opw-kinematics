use crate::constraints::BY_PREV;
use crate::kinematic_traits::{Joints, Kinematics, Pose};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;
use crate::urdf::from_urdf_chain;
use glam::{DMat4, DQuat, DVec3};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};
use sxd_document::{dom::Element, parser};

fn assert_parameters_match(actual: Parameters, expected: Parameters) {
    assert_eq!(actual.a1, expected.a1, "a1");
    assert_eq!(actual.a2, expected.a2, "a2");
    assert_eq!(actual.b, expected.b, "b");
    assert_eq!(actual.c1, expected.c1, "c1");
    assert_eq!(actual.c2, expected.c2, "c2");
    assert_eq!(actual.c3, expected.c3, "c3");
    assert_eq!(actual.c4, expected.c4, "c4");
    assert_eq!(actual.offsets, expected.offsets, "offsets");
    assert_eq!(actual.sign_corrections, expected.sign_corrections, "signs");
    assert_eq!(actual.dof, expected.dof, "dof");
}

fn optional_child<'d>(element: Element<'d>, name: &str) -> Option<Element<'d>> {
    element
        .children()
        .into_iter()
        .filter_map(|node| node.element())
        .find(|element| element.name().local_part() == name)
}

fn child<'d>(element: Element<'d>, name: &str) -> Element<'d> {
    optional_child(element, name).unwrap_or_else(|| panic!("missing {name} element in fixture"))
}

// These are the only xacro expressions used by the fixtures' joint transforms.
// Unexpected expressions fail instead of silently being treated as zero.
fn vector(attribute: &str) -> DVec3 {
    let values: Vec<f64> = attribute
        .split_whitespace()
        .map(|value| match value {
            "${pi}" => PI,
            "${-pi/2.0}" => -FRAC_PI_2,
            "${radians(90)}" => FRAC_PI_2,
            _ => value.parse().expect("unsupported fixture coordinate"),
        })
        .collect();
    DVec3::from_array(values.try_into().expect("expected three coordinates"))
}

// Return the base-to-tip transform in the fixture XML at the given joint angles,
// independently of OPW geometry and the solver's joint-offset/sign conventions.
fn urdf_forward(xml: &str, base: &str, tip: &str, angles: &Joints) -> DMat4 {
    let xml = xml.replace("${prefix}", "");
    let package = parser::parse(&xml).expect("valid fixture XML");
    let robot = package
        .as_document()
        .root()
        .children()
        .into_iter()
        .find_map(|node| node.element())
        .expect("robot element");
    let container = robot
        .children()
        .into_iter()
        .filter_map(|node| node.element())
        .find(|element| element.name().local_part() == "macro")
        .unwrap_or(robot);
    let joints: Vec<_> = container
        .children()
        .into_iter()
        .filter_map(|node| node.element())
        .filter(|element| element.name().local_part() == "joint")
        .collect();

    // Follow the actual chain so the fixed flange/tool0 rotations are included
    // and the separate Fanuc base_link -> base (controller world) branch is not.
    let mut link = tip;
    let mut chain = Vec::new();
    while link != base {
        assert!(chain.len() < joints.len(), "cycle in fixture joint chain");
        let joint = *joints
            .iter()
            .find(|joint| child(**joint, "child").attribute_value("link") == Some(link))
            .unwrap_or_else(|| panic!("no parent joint for {link}"));
        chain.push(joint);
        link = child(joint, "parent")
            .attribute_value("link")
            .expect("parent link");
    }

    let mut transform = DMat4::IDENTITY;
    let mut angle_index = 0;
    let mut joint_angles = HashMap::new();
    for joint in chain.into_iter().rev() {
        if let Some(origin) = optional_child(joint, "origin") {
            let xyz = vector(origin.attribute_value("xyz").unwrap_or("0 0 0"));
            let rpy = vector(origin.attribute_value("rpy").unwrap_or("0 0 0"));
            // URDF uses extrinsic roll, pitch, yaw: Rz(yaw) * Ry(pitch) * Rx(roll).
            let rotation = DQuat::from_rotation_z(rpy.z)
                * DQuat::from_rotation_y(rpy.y)
                * DQuat::from_rotation_x(rpy.x);
            transform *= DMat4::from_rotation_translation(rotation, xyz);
        }
        match joint.attribute_value("type").expect("joint type") {
            "fixed" => {}
            "revolute" | "continuous" => {
                let axis = vector(child(joint, "axis").attribute_value("xyz").expect("axis"));
                let angle = if let Some(mimic) = optional_child(joint, "mimic") {
                    let source = joint_angles[mimic.attribute_value("joint").unwrap()];
                    let multiplier: f64 = mimic
                        .attribute_value("multiplier")
                        .unwrap_or("1")
                        .parse()
                        .unwrap();
                    let offset: f64 = mimic
                        .attribute_value("offset")
                        .unwrap_or("0")
                        .parse()
                        .unwrap();
                    multiplier * source + offset
                } else {
                    let angle = angles[angle_index];
                    angle_index += 1;
                    angle
                };
                joint_angles.insert(joint.attribute_value("name").unwrap(), angle);
                transform *= DMat4::from_quat(DQuat::from_axis_angle(axis, angle));
            }
            other => panic!("unsupported fixture joint type {other}"),
        }
    }
    assert_eq!(angle_index, 6, "expected six revolute joints");
    transform
}

fn assert_pose_matches_urdf(actual: Pose, expected: DMat4, angles: &Joints) {
    let position_error = actual
        .translation
        .distance(expected.transform_point3(DVec3::ZERO));
    assert!(
        position_error < 1e-10,
        "position error {position_error} at {angles:?}"
    );
    // Comparing the three rotated unit axes avoids quaternion sign ambiguity.
    for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
        let rotation_error = (actual.rotation * axis).distance(expected.transform_vector3(axis));
        assert!(
            rotation_error < 1e-10,
            "rotation error {rotation_error} on {axis:?} at {angles:?}"
        );
    }
}

// Check the robot's FK and IK against the selected base-to-tip path in the fixture XML.
fn assert_kinematics_match_urdf(robot: &dyn Kinematics, xml: &str, base: &str, tip: &str) {
    for angles in [
        [0.0; 6],
        [0.3, -0.4, 0.5, -0.6, 0.7, -0.8],
        [-0.9, 0.2, -0.7, 1.1, -0.5, 0.4],
    ] {
        let expected = urdf_forward(xml, base, tip, &angles);
        assert_pose_matches_urdf(robot.forward(&angles), expected, &angles);
        if angles != [0.0; 6] {
            let target = Pose::from_parts(
                expected.transform_point3(DVec3::ZERO),
                DQuat::from_mat4(&expected),
            );
            let solutions = robot.inverse(&target);
            assert!(!solutions.is_empty(), "no inverse solutions at {angles:?}");
            for solution in solutions {
                // Check each inverse branch in the source model as well.
                let expected_at_solution = urdf_forward(xml, base, tip, &solution);
                assert_pose_matches_urdf(target, expected_at_solution, &solution);
            }
        }
    }
}

macro_rules! preset_test {
    ($constructor:ident, $xacro:literal $(, $yaml:literal)?) => {
        #[test]
        fn $constructor() {
            let parameters = Parameters::$constructor();
            $(
                let expected = Parameters::from_yaml_file(concat!(
                    env!("CARGO_MANIFEST_DIR"), "/src/tests/data/", $yaml
                )).expect("valid OPW YAML fixture");
                assert_parameters_match(parameters, expected);
            )?
            assert_kinematics_match_urdf(
                &OPWKinematics::new(parameters),
                include_str!(concat!("data/", $xacro)),
                "base_link",
                "tool0",
            );
        }
    };
}

preset_test!(
    kuka_kr6_r900_2,
    "kuka/kr6r900_2_macro.xacro",
    "kuka/opw_parameters_kr6r900_2.yaml"
);
preset_test!(
    kuka_kr10_r1420,
    "kuka/kr10r1420_macro.xacro",
    "kuka/opw_parameters_kr10r1420.yaml"
);
preset_test!(
    kuka_kr150_r3100_2,
    "kuka/kr150r3100_2_macro.xacro",
    "kuka/opw_parameters_kr150r3100_2.yaml"
);
preset_test!(
    fanuc_m10ia,
    "fanuc/m10ia_macro.xacro",
    "fanuc/opw_parameters_m10ia.yaml"
);
preset_test!(
    fanuc_m20ia,
    "fanuc/m20ia_macro.xacro",
    "fanuc/opw_parameters_m20ia.yaml"
);
preset_test!(
    fanuc_lrmate200ib,
    "fanuc/lrmate200ib_macro.xacro",
    "fanuc/opw_parameters_lrmate200ib.yaml"
);
preset_test!(
    fanuc_m6ib,
    "fanuc/m6ib_macro.xacro",
    "fanuc/opw_parameters_m6ib.yaml"
);
preset_test!(
    fanuc_m16ib20,
    "fanuc/m16ib20.urdf",
    "fanuc/opw_parameters_m16ib20.yaml"
);
preset_test!(
    fanuc_m20ib25,
    "fanuc/m20ib25.urdf",
    "fanuc/opw_parameters_m20ib25.yaml"
);
preset_test!(irb120_3_58, "abb/irb120_3_58.urdf");
preset_test!(irb1200_5_90, "abb/irb1200_5_90.urdf");
preset_test!(irb1200_7_70, "abb/irb1200_7_70.urdf");
preset_test!(
    irb1600_6_120,
    "abb/irb1600_6_120.urdf",
    "abb/opw_parameters_irb1600_6_120.yaml"
);
preset_test!(irb4600_40_255, "abb/irb4600_40_255.urdf");

const IRB120_URDF: &str = include_str!("data/abb/irb120_3_58.urdf");

// Name each fixture separately so a failed geometry or pose check identifies the model.
macro_rules! imported_arm_test {
    ($test:ident, $file:literal, $base:literal, $tip:literal $(, $preset:ident, $reference:literal)?) => {
        #[test]
        fn $test() {
            let xml = include_str!(concat!("data/", $file));
            let model = from_urdf_chain(xml, $base, $tip).unwrap();
            let robot = model.to_robot(BY_PREV).unwrap();
            assert_kinematics_match_urdf(&robot, xml, $base, $tip);
            $(
                let parameters = Parameters::$preset();
                let expected = Parameters::from_yaml_file(concat!(
                    env!("CARGO_MANIFEST_DIR"), "/src/tests/data/", $reference
                )).expect("valid OPW YAML fixture");
                assert_parameters_match(parameters, expected);
                assert_kinematics_match_urdf(&OPWKinematics::new(parameters), xml, $base, $tip);
            )?
        }
    };
}

imported_arm_test!(
    import_abb_irb2400,
    "abb/irb2400_12_155.urdf",
    "base_link",
    "tool0"
);
imported_arm_test!(
    import_abb_irb4400l,
    "abb/irb4400l_30_243.urdf",
    "base_link",
    "tool0"
);
// The fixed joint from world to the DENSO base omits its origin.
imported_arm_test!(import_denso_vs060, "denso/vs060.urdf", "world", "J6");
imported_arm_test!(
    import_fanuc_lrmate200ic,
    "fanuc/lrmate200ic.urdf",
    "base_link",
    "tool0",
    fanuc_lrmate200ic,
    "fanuc/opw_parameters_lrmate200ic.yaml"
);
imported_arm_test!(
    import_fanuc_moveit_m10ia,
    "fanuc/moveit_m10ia.urdf",
    "base_link",
    "tool0"
);
imported_arm_test!(
    import_fanuc_r2000ic165f,
    "fanuc/r2000ic165f.urdf",
    "base_link",
    "tool0",
    fanuc_r2000ic_165f,
    "fanuc/opw_parameters_r2000ic165f.yaml"
);
imported_arm_test!(
    import_kawasaki_rs030n,
    "kawasaki/rs030n.urdf",
    "base_link",
    "link6"
);
imported_arm_test!(
    import_kuka_kr16,
    "kuka/kr16_r1610_2.urdf",
    "base_link",
    "tool0"
);
imported_arm_test!(
    import_kuka_kr70,
    "kuka/kr70_r2100.urdf",
    "base_link",
    "tool0"
);
imported_arm_test!(
    import_kuka_kr360,
    "kuka/kr360_r2830.urdf",
    "base_link",
    "tool0"
);
imported_arm_test!(
    import_pilz_prbt,
    "pilz/pilz-prbt.urdf",
    "prbt_base_link",
    "prbt_tool0"
);
imported_arm_test!(
    import_staubli_rx160l,
    "staubli/rx160l.urdf",
    "base_link",
    "tool0"
);
imported_arm_test!(
    import_staubli_tx2_60,
    "staubli/tx2_60.urdf",
    "base_link",
    "tool0",
    staubli_tx2_60,
    "staubli/opw_parameters_tx2_60.yaml"
);
imported_arm_test!(
    import_yaskawa_gp7,
    "yaskawa/gp7.urdf",
    "base_link",
    "tool0",
    yaskawa_gp7,
    "yaskawa/opw_parameters_gp7.yaml"
);
imported_arm_test!(
    import_yaskawa_gp12,
    "yaskawa/gp12.urdf",
    "base_link",
    "tool0"
);
imported_arm_test!(
    import_yaskawa_gp180,
    "yaskawa/gp180.urdf",
    "base_link",
    "tool0",
    yaskawa_gp180,
    "yaskawa/opw_parameters_gp180.yaml"
);
imported_arm_test!(
    import_yaskawa_gp180_120,
    "yaskawa/gp180_120.urdf",
    "base_link",
    "tool0",
    yaskawa_gp180_120,
    "yaskawa/opw_parameters_gp180_120.yaml"
);
imported_arm_test!(
    import_yaskawa_motomini,
    "yaskawa/motomini.urdf",
    "base_link",
    "tool0"
);

#[test]
fn import_ur5e_identifies_parallel_elbow_and_wrist_axes() {
    let xml = include_str!("data/universal_robots/ur5e.urdf");
    let error = from_urdf_chain(xml, "base_link", "tool0").unwrap_err();
    assert!(
        error
            .to_string()
            .contains("J3 and J4 must be perpendicular"),
        "{error}"
    );
}

#[test]
fn import_lrmate_5h_identifies_five_axis_arm() {
    let xml = include_str!("data/fanuc/lrmate200ic5h.urdf");
    let error = from_urdf_chain(xml, "base_link", "tool0").unwrap_err();
    assert!(error.to_string().contains("5 rotary joints"), "{error}");
}

#[test]
fn lateral_offset_roundoff_preserves_prbt_continuation() {
    let xml = include_str!("data/pilz/pilz-prbt.urdf");
    let base = "prbt_base_link";
    let tip = "prbt_tool0";
    let robot = from_urdf_chain(xml, base, tip)
        .unwrap()
        .to_robot(BY_PREV)
        .unwrap();
    // With the arm straight up, only the sum of J1 and J4 matters. The
    // previous angles already reach the target, so neither should jump.
    let previous = [1.4, 0.0, 0.0, 0.7, 0.9, -0.5];
    let expected = urdf_forward(xml, base, tip, &previous);
    let target = Pose::from_parts(
        expected.transform_point3(DVec3::ZERO),
        DQuat::from_mat4(&expected),
    );
    let solutions = robot.inverse_continuing(&target, &previous);
    let first = solutions.first().expect("reachable singular pose");
    for (i, (actual, prior)) in first.iter().zip(previous).enumerate() {
        assert!(
            (actual - prior).abs() < 1e-7,
            "unexpected J{} movement: {first:?}",
            i + 1
        );
    }
}

#[test]
fn lateral_offset_roundoff_preserves_explicit_small_offset() {
    // With exactly aligned axes, even a sub-ulp lateral offset of an arm
    // link is separately representable in its Y coordinate.
    let xml = IRB120_URDF.replace("0 0 0.27", "0 1e-17 0.27");
    let chain = from_urdf_chain(&xml, "base_link", "tool0").unwrap();
    assert_eq!(chain.parameters.b, 1e-17);
}

#[test]
fn imported_tx2_60_respects_full_turn_j1_stops() {
    let xml = include_str!("data/staubli/tx2_60.urdf");
    let robot = from_urdf_chain(xml, "base_link", "tool0")
        .unwrap()
        .to_robot(BY_PREV)
        .unwrap();
    let limits = robot.constraints().as_ref().unwrap();
    for stop in [-PI, PI] {
        let mut angles = [0.0; 6];
        angles[0] = stop;
        assert!(limits.compliant(&angles));
        angles[0] += stop.signum() * 1e-6;
        assert!(!limits.compliant(&angles));
    }
}

#[test]
fn imported_irb5400_identifies_the_coupled_wrist_joint() {
    let xml = include_str!("data/abb/irb5400.urdf");
    // An OPW wrist's fifth joint would rotate the tool by the same relative
    // rotation for both 30-degree increments. The two coupled axes do not.
    let orientation = |pitch| {
        let wrist = [0.0, 0.0, 0.0, 0.0, pitch, 0.0];
        DQuat::from_mat4(&urdf_forward(xml, "base_link", "tool0", &wrist))
    };
    let step = PI / 6.0;
    let first = orientation(step) * orientation(0.0).inverse();
    let second = orientation(2.0 * step) * orientation(step).inverse();
    assert!(first.angle_between(second) > 0.1);

    let error = from_urdf_chain(xml, "base_link", "tool0")
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("joint_5b") && error.contains("mimic"),
        "{error}"
    );
}

#[test]
fn imported_abb_chains_match_their_urdf() {
    for (xml, expected) in [
        (IRB120_URDF, Parameters::irb120_3_58()),
        (
            include_str!("data/abb/irb1600_6_120.urdf"),
            Parameters::irb1600_6_120(),
        ),
    ] {
        let model = from_urdf_chain(xml, "base_link", "tool0").unwrap();
        // The shoulder has both radial and vertical components on the IRB1600.
        // Its forearm length is split over two successive joint translations.
        let p = model.parameters;
        assert!((p.a1 - expected.a1).abs() < 1e-12, "a1");
        assert!((p.c1 - expected.c1).abs() < 1e-12, "c1");
        assert!((p.c3 - expected.c3).abs() < 1e-12, "c3");
        assert!(
            (p.offsets[2] - expected.offsets[2]).abs() < 1e-12,
            "J3 offset"
        );
        let robot = model.to_robot(BY_PREV).unwrap();
        assert_kinematics_match_urdf(&robot, xml, "base_link", "tool0");
    }
}

#[test]
fn imported_chain_preserves_rotated_mount_and_offset_tcp() {
    let xml = IRB120_URDF.replace(
        "</robot>",
        r#"
          <joint name="mount" type="fixed">
            <origin xyz="0.3 -0.2 1.1" rpy="0.7 -0.4 0.2"/>
            <parent link="cell"/><child link="base_link"/>
          </joint>
          <joint name="probe" type="fixed">
            <origin xyz="0.04 0.08 -0.02" rpy="0.3 0.1 -0.2"/>
            <parent link="tool0"/><child link="probe_tip"/>
          </joint>
        </robot>"#,
    );
    let model = from_urdf_chain(&xml, "cell", "probe_tip").unwrap();
    let robot = model.to_robot(BY_PREV).unwrap();
    assert_kinematics_match_urdf(&robot, &xml, "cell", "probe_tip");
}

#[test]
fn imported_chain_interprets_axes_in_the_joint_frame() {
    // Rotate the J3 frame by 180 degrees around Z, then undo that rotation
    // with a fixed adapter before J4. J3's local +Y points along the arm's -Y.
    let xml = IRB120_URDF
        .replace(
            r#"rpy="0 0 0" xyz="0 0 0.27""#,
            r#"rpy="0 0 3.141592653589793" xyz="0 0 0.27""#,
        )
        .replace(
            r#"<parent link="link_3" />"#,
            r#"<parent link="adapter" />"#,
        )
        .replace(
            "</robot>",
            r#"
          <joint name="adapter_mount" type="fixed">
            <origin rpy="0 0 -3.141592653589793" xyz="0 0 0"/>
            <parent link="link_3"/><child link="adapter"/>
          </joint></robot>"#,
        );
    let model = from_urdf_chain(&xml, "base_link", "tool0").unwrap();
    assert_eq!(model.parameters.sign_corrections[2], -1);
    let robot = model.to_robot(BY_PREV).unwrap();
    assert_kinematics_match_urdf(&robot, &xml, "base_link", "tool0");
}

#[test]
fn imported_chain_rejects_offset_wrist_axis() {
    // Shifting J6 sideways leaves all axis directions unchanged, but the three
    // wrist axes no longer pass through a common center.
    let xml = IRB120_URDF.replace("0.072 0 0", "0.072 0.001 0");
    assert!(from_urdf_chain(&xml, "base_link", "tool0").is_err());
}

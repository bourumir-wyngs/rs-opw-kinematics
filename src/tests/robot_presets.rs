use crate::constraints::BY_PREV;
use crate::kinematic_traits::{Joints, Kinematics, Pose};
use crate::kinematics_impl::OPWKinematics;
use crate::parameters::opw_kinematics::Parameters;
use crate::urdf::from_urdf_chain;
use glam::{DMat4, DQuat, DVec3};
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

fn child<'d>(element: Element<'d>, name: &str) -> Element<'d> {
    element
        .children()
        .into_iter()
        .filter_map(|node| node.element())
        .find(|element| element.name().local_part() == name)
        .unwrap_or_else(|| panic!("missing {name} element in fixture"))
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
    for joint in chain.into_iter().rev() {
        let origin = child(joint, "origin");
        let xyz = vector(origin.attribute_value("xyz").expect("joint translation"));
        let rpy = vector(origin.attribute_value("rpy").expect("joint rotation"));
        // URDF uses extrinsic roll, pitch, yaw: Rz(yaw) * Ry(pitch) * Rx(roll).
        let rotation = DQuat::from_rotation_z(rpy.z)
            * DQuat::from_rotation_y(rpy.y)
            * DQuat::from_rotation_x(rpy.x);
        transform *= DMat4::from_rotation_translation(rotation, xyz);
        match joint.attribute_value("type").expect("joint type") {
            "fixed" => {}
            "revolute" => {
                let axis = vector(child(joint, "axis").attribute_value("xyz").expect("axis"));
                transform *= DMat4::from_quat(DQuat::from_axis_angle(axis, angles[angle_index]));
                angle_index += 1;
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
                assert_pose_matches_urdf(robot.forward(&solution), expected, &solution);
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

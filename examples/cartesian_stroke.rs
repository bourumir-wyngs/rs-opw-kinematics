use std::vec::Vec;
#[cfg(feature = "stroke_planning")]
use {
    rrt::dual_rrt_connect,
    std::time::Instant,
    nalgebra::{Isometry3, Translation3, UnitQuaternion},
    rs_opw_kinematics::kinematic_traits::{Joints, Kinematics},
    rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape,
    rs_opw_kinematics::parameters::opw_kinematics::Parameters,
    rs_opw_kinematics::constraints::{Constraints, BY_PREV},
    rs_opw_kinematics::collisions::CollisionBody,
    rs_opw_kinematics::utils,
    rs_opw_kinematics::utils::dump_joints,
};
use rs_opw_kinematics::cartesian::{Cartesian, DEFAULT_TRANSITION_COSTS};
use rs_opw_kinematics::kinematic_traits::Pose;
use rs_opw_kinematics::rrt::RRTPlanner;

#[cfg(feature = "stroke_planning")]
pub fn create_rx160_robot() -> KinematicsWithShape {
    use rs_opw_kinematics::read_trimesh::load_trimesh_from_stl;

    let monolith = load_trimesh_from_stl("src/tests/data/object.stl");

    KinematicsWithShape::new(
        Parameters {
            a1: 0.15,
            a2: 0.0,
            b: 0.0,
            c1: 0.55,
            c2: 0.825,
            c3: 0.625,
            c4: 0.11,
            ..Parameters::new()
        },
        // Constraints are used also to sample constraint-compliant random positions
        // as needed by this path planner.
        Constraints::from_degrees(
            [
                -225.0..=225.0, -225.0..=225.0, -225.0..=225.0,
                -225.0..=225.0, -225.0..=225.0, -225.0..=225.0,
            ],
            BY_PREV,
        ),
        [
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_1.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_2.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_3.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_4.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_5.stl"),
            load_trimesh_from_stl("src/tests/data/staubli/rx160/link_6.stl"),
        ],
        load_trimesh_from_stl("src/tests/data/staubli/rx160/base_link.stl"),
        Isometry3::from_parts(
            Translation3::new(0.4, 0.7, 0.0).into(),
            UnitQuaternion::identity(),
        ),
        load_trimesh_from_stl("src/tests/data/flag.stl"),
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.5).into(),
            UnitQuaternion::identity(),
        ),
        vec![
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(-1., 0., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., 1., 0.) },
            CollisionBody { mesh: monolith.clone(), pose: Isometry3::translation(0., -1., 0.) },
        ],
        true,
    )
}

fn pose(kinematics: &KinematicsWithShape, angles_in_degrees: [f32; 6]) -> Pose {
    kinematics.forward(&utils::joints(&angles_in_degrees))
}

#[cfg(feature = "collisions")]
fn main() {
    // Initialize kinematics with your robot's specific parameters
    let k = create_rx160_robot();

    // Starting point, where the robot exists in the beginning of the task.
    let start = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    
    // In production, other poses are normally given in Cartesian, but here they are given
    // in joints as this way it is easier to craft then in rs-opw-kinematics IDE.
    
    // "Landing" pose close to the surface, from where Cartesian landing on the surface
    // is possible and easy. Robot will change into one of possible alternative configurations
    // between start and land.
    let land = pose(&k, [-120.0, -10.0, -92.51, 18.42, 82.23, 189.35]);
    
    let steps : Vec<Pose> = [
        pose(&k, [-225.0, -27.61, 88.35, -85.42, 44.61, 138.0]),
        //pose(&k, [-225.0, -27.61, 88.35, -85.42, 44.61, 130.0]),
        //pose(&k, [-225.0, -27.61, 88.35, -85.42, 44.61, 120.0]),

        pose(&k, [-225.0, -33.02, 134.48, -121.08, 54.82, 191.01]),
        pose(&k, [-225.0, 57.23, 21.61, -109.48, 97.50, 148.38])
        
    ].into();
    
    // "Parking" pose, Cartesian lifting from the surface at the end of the stroke. Park where we landed.
    let park = pose(&k, [-225.0, -27.61, 88.35, -85.42, 44.61, 110.0]);
    
    // Creat Cartesian planner
    let planner = Cartesian {
        robot: &k, // The robot
        check_step_m: 0.01, // Distance check accuracy in meters (for translation)
        check_step_rad: 1.0_f64.to_radians(), // Distance check accuracy in radians (for rotation)
        max_transition_cost: 6.0_f64.to_radians(), // Maximal transition costs 
        // (weighted sum of abs differences between 'from' and 'to' for all joints, radians).
        transition_coefficients: DEFAULT_TRANSITION_COSTS, // Joint weights to compute transition cost
        
        // RRT planner that computes the non-Cartesian path from starting position to landing pose
        rrt: RRTPlanner {
            step_size_joint_space: 1.0_f64.to_radians(), // RRT planner step in joint space
            max_try: 1000,
            debug: true
        },
        include_linear_interpolation: true, // If true, intermediate Cartesian poses are 
        // included in the output. Otherwise, they are checked, but not included in the output
        debug: true
    };
    
    // plan path
    let path = planner.plan_sequential(&start, &land, steps, &park);
    
    // print trajectory sequence in joints)
    utils::dump_solutions(&path);
}
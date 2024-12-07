use rs_opw_kinematics::cartesian::{Cartesian, DEFAULT_TRANSITION_COSTS};
use rs_opw_kinematics::collisions::{CheckMode, SafetyDistances, NEVER_COLLIDES};
use rs_opw_kinematics::kinematic_traits::{Pose, J2, J3, J4, J6, J_BASE, J_TOOL};
use rs_opw_kinematics::rrt::RRTPlanner;
#[cfg(feature = "stroke_planning")]
use {
    nalgebra::{Isometry3, Translation3, UnitQuaternion},
    rs_opw_kinematics::collisions::CollisionBody,
    rs_opw_kinematics::constraints::{Constraints, BY_PREV},
    rs_opw_kinematics::kinematic_traits::Kinematics,
    rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape,
    rs_opw_kinematics::parameters::opw_kinematics::Parameters,
    rs_opw_kinematics::utils,
    std::time::Instant,
    std::vec::Vec,
};

#[cfg(feature = "stroke_planning")]
pub fn create_rx160_robot() -> KinematicsWithShape {
    use rs_opw_kinematics::read_trimesh::load_trimesh_from_stl;

    let monolith = load_trimesh_from_stl("src/tests/data/object.stl");

    KinematicsWithShape::with_safety(
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
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
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
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(1., 0., 0.),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(-1., 0., 0.),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(0., 1., 0.),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Isometry3::translation(0., -1., 0.),
            },
        ],
        SafetyDistances {
            to_environment: 0.05,   // Robot should not come closer than 5 cm to pillars
            to_robot_default: 0.05, // No closer than 5 cm to itself.
            special_distances: SafetyDistances::distances(&[
                // Due construction of this robot, these joints are very close, so
                // special rules are needed for them.
                ((J2, J_BASE), NEVER_COLLIDES), // base and J2 cannot collide
                ((J3, J_BASE), NEVER_COLLIDES), // base and J3 cannot collide
                ((J2, J4), NEVER_COLLIDES),
                ((J3, J4), NEVER_COLLIDES),
                ((J4, J_TOOL), 0.02_f32), // reduce distance requirement to 2 cm.
                ((J4, J6), 0.02_f32),     // reduce distance requirement to 2 cm.
            ]),
            mode: CheckMode::FirstCollisionOnly, // First pose only (true, enough for path planning)
        },
    )
}

fn pose(kinematics: &KinematicsWithShape, angles_in_degrees: [f32; 6]) -> Pose {
    kinematics.forward(&utils::joints(&angles_in_degrees))
}

#[cfg(feature = "collisions")]
fn main() {
    // Initialize kinematics with your robot's specific parameters
    let k = create_rx160_robot();

    // Starting point, where the robot exists at the beginning of the task.
    let start = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);

    // In production, other poses are normally given in Cartesian, but here they are given
    // in joints as this way it is easier to craft when in rs-opw-kinematics IDE.

    // "Landing" pose close to the surface, from where Cartesian landing on the surface
    // is possible and easy. Robot will change into one of the possible alternative configurations 
    // between start and land.
    let land = pose(&k, [-120.0, -10.0, -92.51, 18.42, 82.23, 189.35]);

    let steps: Vec<Pose> = [
        pose(&k, [-225.0, -27.61, 88.35, -85.42, 44.61, 138.0]),
        pose(&k, [-225.0, -33.02, 134.48, -121.08, 54.82, 191.01]),
        //pose(&k, [-225.0, 57.23, 21.61, -109.48, 97.50, 148.38]) // this collides
    ]
    .into();

    // "Parking" pose, Cartesian lifting from the surface at the end of the stroke. Park where we landed.
    let park = pose(&k, [-225.0, -27.61, 88.35, -85.42, 44.61, 110.0]);

    // Creat Cartesian planner
    let planner = Cartesian {
        robot: &k, // The robot, instance of KinematicsWithShape
        check_step_m: 0.02, // Pose distance check accuracy in meters (for translation)
        check_step_rad: 3.0_f64.to_radians(), // Pose distance check accuracy in radians (for rotation)
        max_transition_cost: 3_f64.to_radians(), // Maximal transition costs (not tied to the parameter above)
        // (weighted sum of abs differences between 'from' and 'to' for all joints, radians).
        transition_coefficients: DEFAULT_TRANSITION_COSTS, // Joint weights to compute transition cost
        linear_recursion_depth: 8,

        // RRT planner that computes the non-Cartesian path from starting position to landing pose
        rrt: RRTPlanner {
            step_size_joint_space: 2.0_f64.to_radians(), // RRT planner step in joint space
            max_try: 1000,
            debug: true,
        },
        include_linear_interpolation: true, // If true, intermediate Cartesian poses are
        // included in the output. Otherwise, they are checked but not included in the output
        debug: true,
    };

    // plan path
    let started = Instant::now();
    let path = planner.plan(&start, &land, steps, &park);
    let elapsed = started.elapsed();

    match path {
        Ok(path) => {
            for joints in path {
                println!("{:?}", &joints);
            }
        }
        Err(message) => {
            println!("Failed: {}", message);
        }
    }
    println!("Took {:?}", elapsed);
}

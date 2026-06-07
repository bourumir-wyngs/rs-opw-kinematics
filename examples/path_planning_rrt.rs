#[cfg(all(feature = "stroke_planning", feature = "rs-read-trimesh"))]
use anyhow::anyhow;
use anyhow::Result;

#[cfg(all(feature = "stroke_planning", feature = "rs-read-trimesh"))]
use {
    rs_opw_kinematics::collisions::CollisionBody,
    rs_opw_kinematics::collisions::{CheckMode, SafetyDistances, NEVER_COLLIDES},
    rs_opw_kinematics::constraints::{Constraints, BY_PREV},
    rs_opw_kinematics::glam::{DVec3, Vec3},
    rs_opw_kinematics::kinematic_traits::{Joints, Pose},
    rs_opw_kinematics::kinematic_traits::{J2, J3, J4, J6, J_BASE, J_TOOL},
    rs_opw_kinematics::kinematics_with_shape::KinematicsWithShape,
    rs_opw_kinematics::parameters::opw_kinematics::Parameters,
    rs_opw_kinematics::pose::Pose32,
    rs_opw_kinematics::rrt::RRTPlanner,
    rs_opw_kinematics::utils,
    rs_opw_kinematics::utils::dump_joints,
    rs_read_trimesh::load_trimesh,
    std::sync::atomic::AtomicBool,
    std::time::Instant,
};

#[cfg(all(
    feature = "stroke_planning",
    feature = "rs-read-trimesh",
    feature = "visualization"
))]
use {
    std::io::{self, IsTerminal},
    std::thread::sleep,
    std::time::Duration,
};

#[cfg(all(feature = "stroke_planning", feature = "rs-read-trimesh"))]
pub fn create_rx160_robot() -> anyhow::Result<KinematicsWithShape, String> {
    // Environment object to collide with.
    let monolith = load_trimesh("src/tests/data/object.stl", 1.0)?;

    Ok(KinematicsWithShape::with_safety(
        // OPW parameters for Staubli RX 160
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
        // Define constraints directly in degrees, converting internally to radians.
        Constraints::from_degrees(
            [
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -225.0..=225.0,
                -360.0..=360.0,
            ],
            BY_PREV, // Prioritize previous joint position
        ),
        // Joint meshes
        [
            // If your meshes, if offset in .stl file, use Trimesh::transform_vertices,
            // you may also need Trimesh::scale in some extreme cases.
            // If your joints or tool consist of multiple meshes, combine these
            // with Trimesh::append
            load_trimesh("src/tests/data/staubli/rx160/link_1.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_2.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_3.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_4.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_5.stl", 1.0)?,
            load_trimesh("src/tests/data/staubli/rx160/link_6.stl", 1.0)?,
        ],
        // Base link mesh
        load_trimesh("src/tests/data/staubli/rx160/base_link.stl", 1.0)?,
        // Base transform, this is where the robot is standing
        Pose::from_translation(DVec3::new(0.4, 0.7, 0.0)),
        // Tool mesh. Load it from .ply file for feature demonstration
        load_trimesh("src/tests/data/flag.ply", 1.0)?,
        // Tool transform, tip (not base) of the tool. The point past this
        // transform is known as tool center point (TCP).
        Pose::from_translation(DVec3::new(0.0, 0.0, 0.5)),
        // Objects around the robot, with global transforms for them.
        vec![
            CollisionBody {
                mesh: monolith.clone(),
                pose: Pose32::from_translation(Vec3::new(1.0, 0.0, 0.0)),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Pose32::from_translation(Vec3::new(-1.0, 0.0, 0.0)),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Pose32::from_translation(Vec3::new(0.0, 1.0, 0.0)),
            },
            CollisionBody {
                mesh: monolith.clone(),
                pose: Pose32::from_translation(Vec3::new(0.0, -1.0, 0.0)),
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
            mode: CheckMode::AllCollsions, // we need to report all for visualization
                                           // mode: CheckMode::NoCheck, // this is very fast but no collision check
        },
    ))
}

/// Plans a path from `start` to `goal` joint configuration, using `KinematicsWithShape` for collision checking.
#[cfg(all(feature = "stroke_planning", feature = "rs-read-trimesh"))]
fn plan_path(
    kinematics: &KinematicsWithShape,
    start: Joints,
    goal: Joints,
) -> Result<Vec<Joints>, String> {
    let stop = AtomicBool::new(false);
    RRTPlanner {
        step_size_joint_space: 3_f64.to_radians(),
        max_try: 2000,
        smooth: 500,
        debug: true,
    }
    .plan_rrt(&start, &goal, kinematics, &stop)
}

#[cfg(all(feature = "stroke_planning", feature = "rs-read-trimesh"))]
fn print_summary(path: &[Joints]) {
    println!("Steps:");
    for step in path {
        dump_joints(step);
    }
}

#[cfg(all(feature = "stroke_planning", feature = "rs-read-trimesh"))]
fn main() -> Result<()> {
    // Initialize kinematics with your robot's specific parameters
    let kinematics = create_rx160_robot().map_err(|e| anyhow!("Failed to create robot: {}", e))?;

    // This is a pretty tough path that requires to lift the initially low placed
    // tool over the obstacle and then lower again. The direct path is interrupted
    // by an obstacle.
    let start = utils::joints(&[-120.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    let goal = utils::joints(&[40.0, -90.0, -92.51, 18.42, 82.23, 189.35]);
    example(
        "Bow deeply before these stones, robot! (building plan ...)",
        start,
        goal,
        kinematics,
    )?;

    Ok(())
}

#[cfg(all(feature = "stroke_planning", feature = "rs-read-trimesh"))]
fn example(name: &str, start: Joints, goal: Joints, kinematics: KinematicsWithShape) -> Result<()> {
    println!("\n** {} **", name);

    let started = Instant::now();
    let path = plan_path(&kinematics, start, goal);
    let spent = started.elapsed();

    match path {
        Ok(path) => {
            println!("Took {:?}", &spent);
            print_summary(&path);
            play_planned_path(kinematics, &path)?;
        }
        Err(message) => {
            println!("Planning failed: {}", message);
        }
    }

    Ok(())
}

#[cfg(all(
    feature = "stroke_planning",
    feature = "rs-read-trimesh",
    feature = "visualization"
))]
fn play_planned_path(robot: KinematicsWithShape, path: &[Joints]) -> Result<()> {
    if path.is_empty() {
        return Ok(());
    }

    let tcp_box = [-2.0..=2.0, -2.0..=2.0, 1.0..=2.0];
    let handle = rs_opw_kinematics::visualization::visualize_robot_async(
        robot,
        utils::to_degrees(&path[0]),
        tcp_box,
    );

    println!("Playing planned path...");
    for joints in path {
        if !handle.is_running() {
            return Ok(());
        }
        handle
            .set_joint_angles(utils::to_degrees(joints))
            .map_err(|err| anyhow!(err))?;
        sleep(Duration::from_millis(50));
    }

    wait_for_visualization(&handle)?;
    handle.close().map_err(|err| anyhow!(err))
}

#[cfg(all(
    feature = "stroke_planning",
    feature = "rs-read-trimesh",
    feature = "visualization"
))]
fn wait_for_visualization(
    handle: &rs_opw_kinematics::visualization::VisualizationHandle,
) -> Result<()> {
    if io::stdin().is_terminal() {
        println!("Window is running. Press Enter here to close it...");
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        return Ok(());
    }

    println!("Window is running. Close the window to exit.");
    while handle.is_running() {
        sleep(Duration::from_millis(100));
    }
    Ok(())
}

#[cfg(all(
    feature = "stroke_planning",
    feature = "rs-read-trimesh",
    not(feature = "visualization")
))]
fn play_planned_path(_robot: KinematicsWithShape, _path: &[Joints]) -> Result<()> {
    println!("Build configuration does not support visualization");
    Ok(())
}

#[cfg(not(all(feature = "stroke_planning", feature = "rs-read-trimesh")))]
fn main() -> Result<()> {
    println!("Build configuration does not support this example");
    Ok(())
}

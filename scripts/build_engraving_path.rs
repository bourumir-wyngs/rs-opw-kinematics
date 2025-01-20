use nalgebra::Isometry3;
use once_cell::sync::Lazy;
use parry3d::shape::TriMesh;
use rayon::iter::ParallelIterator;
use rayon::prelude::IntoParallelRefIterator;
use rs_cxx_ros2_opw_bridge::sender::Sender;
use rs_opw_kinematics::annotations::{AnnotatedPathStep, AnnotatedPose, PathFlags};
use rs_opw_kinematics::cartesian::Cartesian;
use rs_opw_kinematics::engraving::{generate_raster_points, project_flat_to_rect_on_mesh};
use rs_opw_kinematics::head_lifter::HeadLifter;
use rs_opw_kinematics::projector::{Axis, Projector, RayDirection};
use rs_opw_kinematics::synthetic_meshes::{cylinder_mesh, sphere_mesh, sphere_with_recessions};
use rs_read_trimesh::load_trimesh;
use std::f32::consts::PI;
use std::fs::File;
use std::io;
use std::io::Write;
use std::time::Instant;

static PROJECTOR: Projector = Projector {
    check_points: 24, // Defined number of normals to check
    radius: 0.02,     // Radius, defined as PROJECTOR_RADIUS
};

static WRITE_JSON: bool = true;

fn pause() {
    print!("Press Enter to continue...");
    io::stdout().flush().unwrap(); // Ensure the prompt is displayed immediately
    let _ = io::stdin().read_line(&mut String::new()).unwrap(); // Wait for the user to press Enter
}

/// Generate the waypoint that make the letter R
#[allow(non_snake_case)] // we generate uppercase R
fn generate_R_waypoints(height: f32, min_dist: f32) -> Vec<AnnotatedPathStep> {
    let mut waypoints = Vec::new();

    let width = height / 2.0; // Define the width as half the height
    let half_circle_radius = width / 2.0;

    // Helper to generate straight line points
    let generate_line = |start: (f32, f32), end: (f32, f32)| {
        let mut points = Vec::new();
        let dx = end.0 - start.0;
        let dy = end.1 - start.1;
        let dist = (dx * dx + dy * dy).sqrt();
        let steps = (dist / min_dist).ceil() as usize;
        for step in 0..=steps {
            let t = step as f32 / steps as f32;
            points.push(AnnotatedPathStep {
                x: start.0 + t * dx,
                y: start.1 + t * dy,
                flags: PathFlags::NONE,
            })
        }
        points
    };

    // Generate points for the curved part (half-circle)
    let generate_half_circle =
        |center: (f32, f32), radius: f32, start_angle: f32, end_angle: f32| {
            let mut points = Vec::new();
            let arc_length = radius * (end_angle - start_angle).abs(); // Arc length of the curve
            let num_points = (arc_length / min_dist).ceil() as usize; // Number of points based on min_dist
            let step = (end_angle - start_angle) / num_points as f32;

            for i in 0..=num_points {
                let angle = start_angle + i as f32 * step;
                let x = center.0 + radius * angle.cos();
                let y = center.1 - radius * angle.sin();
                points.push(AnnotatedPathStep {
                    x,
                    y,
                    flags: PathFlags::NONE,
                });
            }

            points
        };

    // Step 1: Generate points for the vertical line from bottom-left to top-left
    waypoints.extend(generate_line((0.0, 0.0), (0.0, height)));

    // Step 2: Generate the points for the half-circle
    let half_circle_center = (0.0, height * 0.75);
    waypoints.extend(generate_half_circle(
        half_circle_center,
        half_circle_radius,
        -PI / 2.0, // Start at the bottom of the half-circle
        PI / 2.0,  // End at the top of the half-circle
    ));

    // Step 3: Generate points for the line from circle's top to vertical midpoint
    waypoints.extend(generate_line(
        (0.0, height * 0.75 - half_circle_radius),
        (0.0, height * 0.5),
    ));

    // Step 4: Generate points for the diagonal stroke
    waypoints.extend(generate_line((0.0, height * 0.5), (width * 0.5, 0.0)));

    waypoints
}

fn generate_square_points(n: usize) -> Vec<(f32, f32)> {
    let mut points = Vec::with_capacity(4 * n); // Pre-allocate capacity for 4 sides
    let step = 2.0 / (n as f32 - 1.0); // Distance between consecutive points, including corners

    // Bottom side: From (-1, -1) to (1, -1)
    for i in 0..n {
        let x = -1.0 + i as f32 * step;
        points.push((x, -1.0));
    }

    // Right side: From (1, -1) to (1, 1)
    for i in 0..n {
        let y = -1.0 + i as f32 * step;
        points.push((1.0, y));
    }

    // Top side: From (1, 1) to (-1, 1)
    for i in 0..n {
        let x = 1.0 - i as f32 * step;
        points.push((x, 1.0));
    }

    // Left side: From (-1, 1) to (-1, -1)
    for i in 0..n {
        let y = 1.0 - i as f32 * step;
        points.push((-1.0, y));
    }

    points
}

fn write_isometries_to_json(
    file_path: &str,
    isometries: &Vec<AnnotatedPose>,
) -> Result<(), String> {
    let mut json_output = String::from("[");

    for (i, iso) in isometries.iter().enumerate() {
        let translation = iso.pose.translation;
        let rotation = iso.pose.rotation;

        json_output.push_str(&format!(
            "{{\"position\":{{\"x\":{},\"y\":{},\"z\":{}}},\"rotation\":{{\"x\":{},\"y\":{},\"z\":{},\"w\":{}}},\"flags\":\"{:?}\"}}\n",
            translation.x,
            translation.y,
            translation.z,
            rotation.i,
            rotation.j,
            rotation.k,
            rotation.w,
            iso.flags // Serialize flags as Debug at the end
        ));
        if i < isometries.len() - 1 {
            json_output.push_str(",");
        }
    }

    json_output.push_str("]");

    let mut file = File::create(file_path).expect("Failed to create file");
    file.write_all(json_output.as_bytes())
        .expect("Failed to write to file");
    Ok(())
}

fn filter_valid_poses(poses: &Vec<AnnotatedPose>) -> Vec<Isometry3<f64>> {
    poses
        .iter()
        .filter(|pose| {
            // Extract translation and rotation components
            let translation = pose.pose.translation.vector;
            let rotation = pose.pose.rotation;

            // Check for NaN values in translation and rotation
            if translation.x.is_nan()
                || translation.y.is_nan()
                || translation.z.is_nan()
                || rotation.i.is_nan()
                || rotation.j.is_nan()
                || rotation.k.is_nan()
                || rotation.w.is_nan()
            {
                return false; // NaN values -> invalid
            }

            // Check for zero-length quaternion
            let quaternion_magnitude =
                (rotation.i.powi(2) + rotation.j.powi(2) + rotation.k.powi(2) + rotation.w.powi(2))
                    .sqrt();
            if quaternion_magnitude < 1e-6 {
                // Threshold to consider near zero-length
                return false; // Zero-length quaternion -> invalid
            }

            // Pose passes all checks -> valid
            true
        })
        .map(|pose| pose.pose)
        .collect()
}

fn generate_flat_projections() -> Result<(), String> {
    let sender = Sender::new("127.0.0.1", 5555);

    let mesh = sphere_mesh(0.5, 256);
    let path = generate_raster_points(20, 20);
    let mut shape = Vec::new();

    for axis in [Axis::X, Axis::Y, Axis::Z] {
        for direction in [RayDirection::FromPositive, RayDirection::FromNegative] {
            let engraving =
                project_flat_to_rect_on_mesh(&PROJECTOR, &mesh, &path, axis, direction)?;
            println!(
                "Engraving length of {} for the path of length {}",
                engraving.len(),
                path.len()
            );
            //send_message(&sender, &engraving)?;
            if WRITE_JSON {
                write_isometries_to_json(
                    &format!(
                        "src/tests/data/projector/flat_on_sphere_{:?}_{:?}.json",
                        axis, direction
                    ),
                    &engraving,
                )?;
            };
            shape.extend(engraving);
        }
    }
    send_pose_message(&sender, &shape);
    Ok(())
}

fn generate_cyl_on_sphere() -> Result<(), String> {
    let path = generate_raster_points(40, 200); // Cylinder
    let mesh = sphere_mesh(0.5, 256);

    for axis in [Axis::Y, Axis::X, Axis::Z] {
        let t_ep = Instant::now();
        let engraving =
            PROJECTOR.project_cylinder_path(&mesh, &path, 1.0, -1.5..1.5, 0. ..2.0 * PI, axis)?;

        let engraving = engraving
            .iter()
            .map(|pose| pose.twist(0., 0., 45_f64.to_radians()))
            .collect();

        println!("Mesh on {:?}: {:?}", axis, t_ep.elapsed());
        let sender = Sender::new("127.0.0.1", 5555);
        send_pose_message(&sender, &engraving);

        pause();

        if WRITE_JSON {
            write_isometries_to_json(
                &format!("src/tests/data/projector/cyl_on_sphere_{:?}.json", axis),
                &engraving,
            )?
        }
    }
    Ok(())
}

fn generate_cyl_on_sphere_with_recs() -> Result<(), String> {
    let path = generate_raster_points(50, 50); // Cylinder
    let mesh = sphere_with_recessions(1.0, 0.5, 0.4, 128);

    for axis in [Axis::X, Axis::Y, Axis::Z] {
        let t_ep = Instant::now();
        let engraving =
            PROJECTOR.project_cylinder_path(&mesh, &path, 1.0, -1.5..1.5, 0. ..2.0 * PI, axis)?;

        println!("Mesh on {:?}: {:?}", axis, t_ep.elapsed());
        let sender = Sender::new("127.0.0.1", 5555);

        match sender.send_pose_message(&filter_valid_poses(&engraving)) {
            Ok(_) => {
                println!("Pose message sent successfully.");
            }
            Err(err) => {
                eprintln!("Failed to send pose message: {}", err);
            }
        }

        pause();

        if WRITE_JSON {
            write_isometries_to_json(
                &format!(
                    "src/tests/data/projector/cyl_on_sphere_recs_{:?}.json",
                    axis
                ),
                &engraving,
            )?
        }
    }
    Ok(())
}

fn uplifter_on_sphere_with_recs() -> Result<(), String> {
    use rayon::iter::ParallelIterator;

    let sender = Sender::new("127.0.0.1", 5555);

    let path = generate_raster_points(128, 128); // Cylinder
    let mesh = sphere_mesh(1.0, 128);
    //let mesh = sphere_with_recessions(1.0,  0.5, 0.4, 128);
    let axis = Axis::Z;
    let engraving =
        PROJECTOR.project_cylinder_path(&mesh, &path, 1.0, -1.5..1.5, 0. ..2.0 * PI, axis)?;

    let engraving: Vec<AnnotatedPose>/* Type */ = engraving.par_iter() // Parallel iteration over poses
        .map(|pose| {
            pose.elevate(0.06)
        }).filter_map(|pose| {
        if false && pose.pose.translation.vector.z.abs() < 0.9 {
            None
        } else {
            Some(pose)
        }
    })

        .collect();

    //send_pose_message(&sender, &engraving);
    //pause();

    let toolhead = sphere_mesh(0.05, 64);
    let lifter = HeadLifter::new(&mesh, &toolhead, 0.05, 0.125, 0.01);

    let now = Instant::now();
    let adjusted = adjust_head_collisions(engraving, lifter);
    println!("Lifting took {:?}", now.elapsed());

    send_pose_message(&sender, &adjusted);
    Ok(())
}

fn adjust_head_collisions(engraving: Vec<AnnotatedPose>, lifter: HeadLifter) -> Vec<AnnotatedPose> {
    lifter.adjust_head_collisions(&engraving)
}

fn generate_R_on_goblet() -> Result<(), String> {
    let mesh = load_trimesh("src/tests/data/goblet/goblet.stl", 1.0)?;
    let path = generate_R_waypoints(1.0, 0.001);

    let t_ep = Instant::now();
    let axis = Axis::Z;
    //let mesh = cylinder_mesh(0.2, 1.5, 64, axis);
    let engraving =
        PROJECTOR.project_cylinder_path(&mesh,
                                        &path,
                                        0.5,
                                        0.4 ..0.58,
                                        0. ..0.5 * PI,
                                        axis)?;
    let el_ep = t_ep.elapsed();

    println!(
        "Engraving {:?},  path {} points, engraving {} poses",
        el_ep, path.len(), engraving.len()
    );
   
    send_pose_message(&Sender::new("127.0.0.1", 5555), &engraving);

    if WRITE_JSON {
        write_isometries_to_json(
            &format!("src/tests/data/projector/r_{:?}.json", axis),
            &engraving,
        )?
    }
    Ok(())
}

fn send_pose_message(sender: &Sender, adjusted: &Vec<AnnotatedPose>) {
    match sender.send_pose_message(&filter_valid_poses(&adjusted)) {
        Ok(_) => {
            println!("Pose message sent successfully.");
        }
        Err(err) => {
            eprintln!("Failed to send pose message: {}", err);
        }
    }
}

// https://www.brack.ch/lenovo-workstation-thinkstation-p3-ultra-sff-intel-1813977
fn main() -> Result<(), String> {
    //generate_cyl_on_sphere()?;
    //uplifter_on_sphere_with_recs()?;
    //generate_cyl_on_sphere_with_recs()?;
    //return Ok(());
    //generate_flat_projections()?;
    //return Ok(());
    generate_R_on_goblet()
}

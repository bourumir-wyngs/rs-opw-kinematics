use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use serde::Deserialize;
use std::fs;

#[derive(Deserialize, Debug)]
struct Transform {
    position: Position,
    rotation: Rotation,
}

#[derive(Deserialize, Debug)]
struct Position {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Deserialize, Debug)]
struct Rotation {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

/// Reads a JSON file and parses it into a vector of `nalgebra::Isometry3<f64>`.
fn read_isometries_from_file(file_path: &str) -> Result<Vec<Isometry3<f64>>, Box<dyn std::error::Error>> {
    const SCALE: f64 = 0.005;
    // Read the JSON file as a string
    let file_content = fs::read_to_string(file_path)?;

    // Parse the JSON content into a vector of `Transform`
    let transforms: Vec<Transform> = serde_json::from_str(&file_content)?;

    // Convert the parsed transformations into nalgebra isometries
    let isometries: Vec<Isometry3<f64>> = transforms
        .into_iter()
        .map(|transform| {
            // Create translation
            let translation = Translation3::new(transform.position.x * SCALE, transform.position.y * SCALE, transform.position.z * SCALE);

            // Create a quaternion from the provided rotation
            let raw_quaternion = Quaternion::new(
                transform.rotation.w, // Scalar part
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
            );

            // Normalize the quaternion and construct a UnitQuaternion
            let unit_quaternion = UnitQuaternion::from_quaternion(raw_quaternion);

            // Combine translation and unit quaternion into an Isometry3
            Isometry3::from_parts(translation, unit_quaternion)
        })
        .collect();

    Ok(isometries)
}

fn main() {
    use rs_cxx_ros2_opw_bridge::sender::Sender;

    // Call the function and read the isometries from the JSON file
    let isometries; 
    match read_isometries_from_file("work/isometries.json") {
        Ok(isos) => {
            println!("Isometries read successfully.");
            isometries = isos;
        }
        Err(err) => {
            eprintln!("Failed to read: {}", err);
            panic!();
        }
    }

    // Print the parsed isometries
    for (i, isometry) in isometries.iter().enumerate() {
        println!("Isometry #{}:\n{}", i + 1, isometry);
    }

    let sender = Sender::new("127.0.0.1", 5555);

    // Handle the result of `send_pose_message`
    match sender.send_pose_message(&isometries) {
        Ok(_) => {
            println!("Pose message sent successfully.");
        }
        Err(err) => {
            eprintln!("Failed to send pose message: {}", err);
        }
    }
}
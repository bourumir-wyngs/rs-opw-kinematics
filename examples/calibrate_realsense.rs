
use rs_opw_kinematics::realsense::calibrate_realsense;
use rs_opw_kinematics::transform_io::transform_to_json;
use std::fs::File;
use std::io::Write;

pub fn main() -> anyhow::Result<()>{
    let (serial, transform) = calibrate_realsense()?;
    println!("{:?}", transform);
    
    // Save to file
    let json_data = transform_to_json(&serial, &transform);

    // Write the JSON string to the specified file
    let mut file = File::create("calibration.json")?;
    file.write_all(json_data.as_bytes())?;
    
    
    Ok(())
}
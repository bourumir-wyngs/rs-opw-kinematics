use crate::calipers::largest_fitting_rectangle;
use geo::{ConcaveHull, ConvexHull, LineString, Point as GeoPoint, Polygon};
use parry3d::math::Point as ParryPoint;
use parry3d::shape::TriMesh;

pub fn yz_bounding_rectangle(mesh: &TriMesh) -> (ParryPoint<f32>, ParryPoint<f32>) {
    let concavity = 0.1; // as the mesh we have is is mm
    // Step 1: Project all mesh points onto the YZ plane
    let projected_points: Vec<GeoPoint<f32>> = mesh
        .vertices()
        .iter()
        .map(|vertex| GeoPoint::new(vertex.y, vertex.z)) // Use Z for Geo X, and Y for Geo Y
        .collect();

    // Step 2: Compute the concavehull of the projected points
    
    let concave_hull: Polygon<f32> = Polygon::new(LineString::from(projected_points), vec![]).concave_hull(concavity);

    // Step 3: Compute the largest fitting rectangle in the YZ plane
    if let Some((bottom_left, top_right)) = largest_fitting_rectangle(&concave_hull) {
        // Step 4: Convert GeoPoints to ParryPoints with X set to 0
        let bottom_left_parry = ParryPoint::new(0.0, bottom_left.x(), bottom_left.y());
        let top_right_parry = ParryPoint::new(0.0, top_right.x(), top_right.y());

        (bottom_left_parry, top_right_parry)
    } else {
        panic!("Failed to compute the largest fitting rectangle.");
    }
}

// Conversion functions between Parry and Geo points (if needed)
fn parry_to_geo(parry_point: ParryPoint<f32>) -> GeoPoint<f32> {
    GeoPoint::new(parry_point.y, parry_point.z)
}

fn geo_to_parry(geo_point: GeoPoint<f32>) -> ParryPoint<f32> {
    ParryPoint::new(0.0, geo_point.x(), geo_point.y()) // Project X to 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read_trimesh::load_trimesh_from_ply;
    use parry3d::math::Point;
    use std::fs::File; // Adjust as needed to import `load_trimesh_from_ply` and `yz_bounding_rectangle`
    use parry3d::math::Point as ParryPoint;
    use std::io::{self, Write};


    // Writes the rectangle corners and mesh points to a JSON file
    fn write_rectangle_and_mesh_to_json(
        corners: &[ParryPoint<f32>],
        bottom_left: &ParryPoint<f32>,
        top_right: &ParryPoint<f32>,
        mesh_points: &[ParryPoint<f32>],
        file_path: &str,
    ) -> io::Result<()> {
        let mut file = File::create(file_path)?;

        // Start JSON object
        writeln!(file, "{{")?;

        // Write corners array
        writeln!(file, "\"corners\": [")?;
        for (i, corner) in corners.iter().enumerate() {
            writeln!(
                file,
                "  {{\"x\": {}, \"y\": {}, \"z\": {}}}{}",
                corner.x, corner.y, corner.z,
                if i == corners.len() - 1 { "" } else { "," }
            )?;
        }
        writeln!(file, "],")?;

        // Write bottom-left and top-right points
        writeln!(
            file,
            "\"bottom_left\": {{\"x\": {}, \"y\": {}, \"z\": {}}},",
            bottom_left.x, bottom_left.y, bottom_left.z,
        )?;
        writeln!(
            file,
            "\"top_right\": {{\"x\": {}, \"y\": {}, \"z\": {}}},",
            top_right.x, top_right.y, top_right.z
        )?;

        // Write all mesh points
        writeln!(file, "\"mesh_points\": [")?;
        for (i, point) in mesh_points.iter().enumerate() {
            writeln!(
                file,
                "  {{\"x\": {}, \"y\": {}, \"z\": {}}}{}",
                point.x, point.y, point.z,
                if i == mesh_points.len() - 1 { "" } else { "," }
            )?;
        }
        writeln!(file, "]")?;

        // End JSON object
        writeln!(file, "}}")?;

        Ok(())
    }

    #[test]
    fn test_yz_bounding_rectangle() {
        // Load a test mesh
        let mesh = load_trimesh_from_ply("src/tests/data/goblet/ToriLeighR.ply");

        // Compute the bounding rectangle on the YZ plane
        let (bottom_left, top_right) = yz_bounding_rectangle(&mesh);

        println!("Bounding rectangle on YZ plane:");
        println!("Bottom-left corner: {:?}", bottom_left);
        println!("Top-right corner: {:?}", top_right);

        // Rectangle corners
        let corners = vec![
            ParryPoint::new(0.0, bottom_left.y, bottom_left.z),
            ParryPoint::new(0.0, bottom_left.y, top_right.z),
            ParryPoint::new(0.0, top_right.y, bottom_left.z),
            ParryPoint::new(0.0, top_right.y, top_right.z),
        ];

        // Extract all mesh points
        let mesh_points: Vec<ParryPoint<f32>> = mesh.vertices().to_vec();

        // Write data to JSON
        let json_path = "work/rectangle_and_mesh_data.json";
        write_rectangle_and_mesh_to_json(&corners, &bottom_left, &top_right, &mesh_points, json_path)
            .expect("Failed to write rectangle and mesh data to JSON");
        println!("Rectangle and mesh data written to: {}", json_path);

        // Expected values
        let expected_bottom_left = ParryPoint::new(0.0, -36.34134, 0.77647775);
        let expected_top_right = ParryPoint::new(0.0, 36.21518, 126.9999);

        // Assertions
        assert!((bottom_left.x - expected_bottom_left.x).abs() < 1e-6);
        assert!((bottom_left.y - expected_bottom_left.y).abs() < 1e-6);
        assert!((top_right.x - expected_top_right.x).abs() < 1e-6);
        assert!((top_right.y - expected_top_right.y).abs() < 1e-6);
    }
}

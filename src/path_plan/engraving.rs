use crate::annotations::{AnnotatedPathStep, AnnotatedPose, PathFlags};
use crate::calipers::largest_fitting_rectangle;
use crate::projector::{Axis, Projector, RayDirection};
use geo::{ConcaveHull, LineString, Point as GeoPoint, Polygon};
use parry3d::shape::TriMesh;

const MARGIN: f32 = 0.04;

pub fn generate_raster_points(r: usize, n: usize) -> Vec<AnnotatedPathStep> {
    let mut points = Vec::with_capacity(r * n);
    let y_step = 2.0 / (r as f32 - 1.0); // Vertical spacing between rows
    let x_step = 2.0 / (n as f32 - 1.0); // Horizontal spacing between points in a row

    for i in 0..r {
        let y = -1.0 + i as f32 * y_step; // Calculate the y-coordinate for the row

        // Conditional to alternate row directions
        if i % 2 == 0 {
            // Even row: left-to-right
            for j in 0..n {
                let x = -1.0 + j as f32 * x_step;
                points.push(AnnotatedPathStep {
                    x,
                    y,
                    flags: PathFlags::FORWARDS,
                });
            }
        } else {
            // Odd row: right-to-left
            for j in (0..n).rev() {
                let x = -1.0 + j as f32 * x_step;
                points.push(AnnotatedPathStep {
                    x,
                    y,
                    flags: PathFlags::BACKWARDS,
                });
            }
        }
    }
    points
}

pub fn axis_aligned_bounding_rectangle(
    mesh: &TriMesh,
    axis: Axis,
) -> Result<(GeoPoint<f32>, GeoPoint<f32>), String> {
    // THIS CODE IS SENSITIVE TO ABSOLUTE DIMENSIONS
    // Concavity cannot be scaled with 'scale'
    let concavity = 0.1;
    let scale = 1.0 / 0.005;
    // Step 1: Project all mesh points onto the relevant plane based on the axis
    let projected_points: Vec<GeoPoint<f32>> = mesh
        .vertices()
        .iter()
        .map(|vertex| match axis {
            Axis::X => GeoPoint::new(vertex.y * scale, vertex.z * scale), // YZ plane
            Axis::Y => GeoPoint::new(vertex.x * scale, vertex.z * scale), // XZ plane
            Axis::Z => GeoPoint::new(vertex.x * scale, vertex.y * scale), // XY plane
        })
        .collect();

    // Step 2: Compute the concave hull of the projected points

    let concave_hull: Polygon<f32> =
        Polygon::new(LineString::from(projected_points), vec![]).concave_hull(concavity);

    // Step 3: Compute the largest fitting rectangle in the relevant plane
    let lff = largest_fitting_rectangle(&concave_hull)?;

    Ok((lff.0 / scale, lff.1 / scale))
}

pub(crate) fn find_min_max(path: &Vec<AnnotatedPathStep>) -> ((f32, f32), (f32, f32)) {
    // Initialize min and max values to INFINITY and NEG_INFINITY.
    let (mut path_min_x, mut path_max_x) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut path_min_y, mut path_max_y) = (f32::INFINITY, f32::NEG_INFINITY);

    // Iterate over all points in the path to determine min and max values for x and y.
    for step in path {
        let x = step.x; // Access the x coordinate
        let y = step.y; // Access the y coordinate

        if x < path_min_x {
            path_min_x = x;
        }
        if x > path_max_x {
            path_max_x = x;
        }
        if y < path_min_y {
            path_min_y = y;
        }
        if y > path_max_y {
            path_max_y = y;
        }
    }

    // Return the computed min/max values as tuples ((min_x, min_y), (max_x, max_y)).
    ((path_min_x, path_min_y), (path_max_x, path_max_y))
}

/// Build a series of poses for "engraving" over the mesh.
pub fn project_flat_to_rect_on_mesh(
    projector: &Projector,
    mesh: &TriMesh,
    path: &Vec<AnnotatedPathStep>,
    axis: Axis,
    ray_direction: RayDirection,
) -> Result<Vec<AnnotatedPose>, String> {
    // Determine a suitable rectangle where we could engrave on the mesh.
    let (bottom_left, top_right) = axis_aligned_bounding_rectangle(mesh, axis)?;

    println!(
        "top_rights: {:?}, bottom_left: {:?}",
        bottom_left, top_right
    );

    // Calculate dimensions of the bounding rectangle
    let rect_width = top_right.x() - bottom_left.x();
    let rect_height = top_right.y() - bottom_left.y();

    // Calculate dimensions of the path
    let path_min_x = path
        .iter()
        .map(|step| step.x) // Access point and its x coordinate
        .fold(f32::INFINITY, f32::min);
    let path_max_x = path
        .iter()
        .map(|step| step.x) // Access point and its x coordinate
        .fold(f32::NEG_INFINITY, f32::max);

    let path_min_y = path
        .iter()
        .map(|step| step.y) // Access point and its y coordinate
        .fold(f32::INFINITY, f32::min);
    let path_max_y = path
        .iter()
        .map(|step| step.y) // Access point and its y coordinate
        .fold(f32::NEG_INFINITY, f32::max);

    // Compute path width and height
    let path_width = path_max_x - path_min_x;
    let path_height = path_max_y - path_min_y;

    // Compute scaling factors to stretch the path into the bounding rectangle with margins
    let scale_x = (rect_width - 2.0 * MARGIN) / path_width;
    let scale_y = (rect_height - 2.0 * MARGIN) / path_height;

    // Transform the path to fit into the bounding rectangle based on the axis
    let transformed_path = path
        .iter()
        .map(|step| AnnotatedPathStep {
            x: (step.x - path_min_x) * scale_x + bottom_left.x() + MARGIN,
            y: (step.y - path_min_y) * scale_y + bottom_left.y() + MARGIN,
            flags: step.flags,
        })
        .collect();

    projector.project_flat_path(&mesh, &transformed_path, axis, ray_direction)
}

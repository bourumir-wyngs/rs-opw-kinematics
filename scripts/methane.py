# Create Methane on stand
from copy import deepcopy

meters = 1000.0
units = meters

ball_radius = 0.01 * units
plain_bond = 0.032 * units
bond_diameter = 0.004 * units
disk_radius = 0.06 * units
disk_thickness = 0.002 * units
triangle_thickness = disk_thickness

hydrogen_radius = ball_radius
carbon_radius = hydrogen_radius * 2 / 3
bond_length = plain_bond + 2 * ball_radius  # Bond length from carbon to hydrogen

import bpy
import mathutils
import math

# === Clear the scene ===
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()


def create_sphere(radius, position, name="Sphere"):
    """Creates a sphere at the given position."""
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=position)
    obj = bpy.context.object
    obj.name = name
    return obj


def create_cylinder(diameter, start, end, name="Cylinder"):
    """Creates a cylinder (bond) connecting two points."""
    start_vec = mathutils.Vector(start)
    end_vec = mathutils.Vector(end)
    direction = end_vec - start_vec
    length = direction.length
    mid_point = (start_vec + end_vec) / 2

    bpy.ops.mesh.primitive_cylinder_add(radius=diameter / 2, depth=length, location=mid_point)
    obj = bpy.context.object
    obj.name = name
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = direction.to_track_quat('Z', 'Y')
    return obj


def create_disk(radius, thickness, position, normal, name="Disk"):
    """Creates a disk (flat base) at the given position, aligned with a normal vector."""
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=thickness, location=position)
    obj = bpy.context.object
    obj.name = name
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = normal.to_track_quat('Z', 'Y')
    return obj

def create_massive_triangle(p1, p2, p3, thickness, name="MassiveTriangle"):
    thickness = thickness /2
    # Convert points to Blender vectors
    v1 = mathutils.Vector(p1)
    v2 = mathutils.Vector(p2)
    v3 = mathutils.Vector(p3)

    # Compute the triangle's normal vector
    normal = (v2 - v1).cross(v3 - v1).normalized()

    # Offset vertices by thickness along the normal
    v1_a = v1 + normal * thickness
    v2_a = v2 + normal * thickness
    v3_a = v3 + normal * thickness

    # Offset vertices by thickness along the normal
    v1_b = v1 - normal * thickness
    v2_b = v2 - normal * thickness
    v3_b = v3 - normal * thickness

    # Combine all vertices into a list
    verts = [v1_b, v2_b, v3_b, v1_a, v2_a, v3_a]

    # Define faces (each face is defined by the indices of its vertices)
    faces = [
        (2, 1, 0),  # Bottom face
        (4, 5, 3),  # Top face (order reversed)
        (1, 4, 3, 0),  # Side face 1 (reversed order)
        (2, 5, 4, 1),  # Side face 2 (reversed order)
        (0, 3, 5, 2),  # Side face 3 (reversed order)
    ]

    # Create mesh and object
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)

    # Link object to the scene
    bpy.context.collection.objects.link(obj)

    # Create the prism mesh
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    return obj


def create_triangle(p1, p2, p3, name="Triangle"):
    """
    Creates a triangular face in Blender given three vertices.

    :param p1: Tuple (x, y, z) - First vertex of the triangle.
    :param p2: Tuple (x, y, z) - Second vertex of the triangle.
    :param p3: Tuple (x, y, z) - Third vertex of the triangle.
    :param name: Name of the created triangle object.
    """
    # Convert tuples to Blender vectors
    verts = [mathutils.Vector(p1), mathutils.Vector(p2), mathutils.Vector(p3)]

    # Create mesh and object
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)

    # Link object to the scene
    bpy.context.collection.objects.link(obj)

    # Create the triangle mesh
    mesh.from_pydata(verts, [], [(0, 1, 2)])
    mesh.update()

    return obj

def land_base_on_disk(base_center, base_normal, point_to_project):
    """
    Projects point_to_project onto the plane of the disk to land the base properly.

    :param base_center: Center of the disk (disk's position in space).
    :param base_normal: Normal vector defining the orientation of the disk.
    :param point_to_project: The point to project to land it on the disk.
    :return: The new position of the point after projection onto the plane.
    """
    base_vector = mathutils.Vector(point_to_project) - mathutils.Vector(base_center)
    distance = base_vector.dot(base_normal)
    projection = base_vector - (base_normal * distance)
    projected_point = mathutils.Vector(base_center) + projection
    return projected_point


def mid(p1, p2):
    return ((mathutils.Vector(p1) + mathutils.Vector(p2)) / 2).to_tuple()

def create_methane_with_stand():
    # Create Carbon atom
    create_sphere(carbon_radius, (0, 0, 0), name="Carbon")

    sqrt_3 = math.sqrt(3)

    hydrogen_positions = [
        (bond_length / sqrt_3, bond_length / sqrt_3, bond_length / sqrt_3),
        (-bond_length / sqrt_3, -bond_length / sqrt_3, bond_length / sqrt_3),
        (-bond_length / sqrt_3, bond_length / sqrt_3, -bond_length / sqrt_3),
        (bond_length / sqrt_3, -bond_length / sqrt_3, -bond_length / sqrt_3),
    ]

    # Create Hydrogen atoms and bonds
    for i1, h_pos in enumerate(hydrogen_positions):
        create_sphere(hydrogen_radius, h_pos, name=f"Hydrogen_{i1 + 1}")
        create_cylinder(bond_diameter, (0, 0, 0), h_pos, name=f"Bond_{i1 + 1}")

    for i1, h_pos in enumerate(hydrogen_positions):
        for i2, hh_pos in enumerate(hydrogen_positions):
            for i3, hhh_pos in enumerate(hydrogen_positions):
                if i1 != i2 and i2 != i3 and i3 != i1:
                    h1 = (h_pos[0]/3, h_pos[1]/3, h_pos[2]/3)
                    h2 = (hh_pos[0]/3, hh_pos[1]/3, hh_pos[2]/3)
                    h3 = (hhh_pos[0]/3, hhh_pos[1]/3, hhh_pos[2]/3)
                    create_triangle(h2, h1, h3, name=f"Triangle_{i1 + 1}_{i2 + 1}_{i3 + 1}")

    # Extend **one** bond further and add a **stand** (choosing Hydrogen_1)
    h_pos = hydrogen_positions[0]  # Pick one hydrogen atom
    direction = mathutils.Vector(h_pos) - mathutils.Vector((0, 0, 0))
    extended_end = mathutils.Vector(h_pos) + direction  # Extend the bond further

    # Create the extended part of the bond
    create_cylinder(bond_diameter, h_pos, extended_end, name="Extended_Bond")

    # Add a **flat disk** (stand) at the end of the extended bond
    create_disk(disk_radius, disk_thickness, extended_end, direction, name="Stand")

    disk_normal = direction.normalized()
    disk_center = extended_end


    # Project points for the triangle so they correctly align with the disk
    base_point = (extended_end[0], extended_end[1], hydrogen_positions[1][2])
    landed_base = land_base_on_disk(disk_center, disk_normal, base_point)

    bc = mid(hydrogen_positions[0], disk_center)
    for i1, h_pos in enumerate(hydrogen_positions):
        landed_base = land_base_on_disk(disk_center, disk_normal, h_pos)
        create_massive_triangle(landed_base, bc, disk_center, bond_diameter/2, name=f"Support")

    # Validate distances
    test_hydrogen_distances(hydrogen_positions)



def test_hydrogen_distances(hydrogen_positions):
    """
    Tests whether all hydrogen atoms are equidistant from each other.
    """
    distances = []
    for i in range(len(hydrogen_positions)):
        for j in range(i + 1, len(hydrogen_positions)):
            p1 = mathutils.Vector(hydrogen_positions[i])
            p2 = mathutils.Vector(hydrogen_positions[j])
            distance = (p1 - p2).length
            distances.append(distance)

    min_distance = min(distances)
    max_distance = max(distances)
    tolerance = 1e-5

    print(f"Hydrogen-Hydrogen Distances: {distances}")
    if abs(max_distance - min_distance) < tolerance:
        print("\\u2705 Test Passed: All hydrogen atoms are equidistant.")
    else:
        print("\\u274C Test Failed: Hydrogen atom distances are not equal.")


def export_methane_stl(filepath="methane_with_stand.stl"):
    """Exports the Methane molecule with a stand as an STL file."""
    bpy.ops.object.select_all(action='DESELECT')

    # Select all objects in the scene
    for obj in bpy.data.objects:
        obj.select_set(True)

    bpy.ops.export_mesh.stl(filepath=filepath, use_selection=True)
    print(f"Exported Methane model with stand to {filepath}")


# Create and export the model
create_methane_with_stand()
export_methane_stl(
    "/home/audrius/opw/rs-opw-kinematics-private/calibrations/calibration_target/methane_with_stand.stl")  # Change path as needed

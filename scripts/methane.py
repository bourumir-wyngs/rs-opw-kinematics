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

def create_methane_with_stand(molecule_radius=1.0, bond_diameter=0.2, disk_radius=2.0, disk_thickness=0.2):
    """
    Creates a Methane (CH4) molecule with correct tetrahedral symmetry and a stand.
    """
    carbon_radius = molecule_radius * 0.5
    hydrogen_radius = molecule_radius

    # Create Carbon atom
    create_sphere(carbon_radius, (0, 0, 0), name="Carbon")

    # Correct tetrahedral hydrogen positions
    bond_length = molecule_radius * 2.5  # Bond length from carbon to hydrogen
    sqrt_3 = math.sqrt(3)

    hydrogen_positions = [
        (bond_length / sqrt_3, bond_length / sqrt_3, bond_length / sqrt_3),
        (-bond_length / sqrt_3, -bond_length / sqrt_3, bond_length / sqrt_3),
        (-bond_length / sqrt_3, bond_length / sqrt_3, -bond_length / sqrt_3),
        (bond_length / sqrt_3, -bond_length / sqrt_3, -bond_length / sqrt_3),
    ]

    # Create Hydrogen atoms and bonds
    for i, h_pos in enumerate(hydrogen_positions):
        create_sphere(hydrogen_radius, h_pos, name=f"Hydrogen_{i+1}")
        create_cylinder(bond_diameter, (0, 0, 0), h_pos, name=f"Bond_{i+1}")

    # Extend **one** bond further and add a **stand** (choosing Hydrogen_1)
    h_pos = hydrogen_positions[0]  # Pick one hydrogen atom
    direction = mathutils.Vector(h_pos) - mathutils.Vector((0, 0, 0))
    extended_end = mathutils.Vector(h_pos) + direction  # Extend the bond further

    # Create the extended part of the bond
    create_cylinder(bond_diameter, h_pos, extended_end, name="Extended_Bond")

    # Add a **flat disk** (stand) at the end of the extended bond
    create_disk(disk_radius, disk_thickness, extended_end, direction, name="Stand")

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
create_methane_with_stand(molecule_radius=1.0, bond_diameter=0.2, disk_radius=2.0, disk_thickness=0.2)
export_methane_stl("methane_with_stand.stl")  # Change path as needed

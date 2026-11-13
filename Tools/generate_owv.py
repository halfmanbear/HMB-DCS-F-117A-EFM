"""
F-117A OWV Spawn Point Generator for Blender
=============================================
Generates a DCS World .owv overwing vapour spawn point file from
selected mesh faces in Blender.

USAGE
-----
1. Open Blender and load your F-117 airframe model.
2. Select the mesh object in Object Mode (no face selection needed).
3. Open the Blender Text Editor, paste/load this script.
4. Edit the CONFIGURATION block at the bottom.
5. Click "Run Script".

The script automatically filters to upward-facing faces only (controlled by
MIN_UP_DOT). You can also manually select faces in Edit Mode — if any faces
are selected the script uses only those instead of the normal filter.

COORDINATE SYSTEM
-----------------
Blender (default orientation)  →  DCS model space
  +X (right)                   →  +Z (right)
  +Y (forward / into scene)    →  +X (forward)
  +Z (up)                      →  +Y (up)

If your Blender model uses a different convention, adjust BLENDER_TO_DCS
in the configuration section.

OWV FILE LAYOUT
---------------
Header  : 128 bytes
  [0:4]   magic "owv\x00"
  [4:40]  9 × float32 — bounding box: min/max x, min/max y, min/max z, 0, 0, 0
            (floats[6-8] must be 0; setting [8]=n_points crashes DCS when the effect loads)
  [40:44] uint32  — version (= 1)
  [44:48] uint32  — CDF entry count (= 10)
  [48:128] 10 × (uint32 cumulative_count, float32 threshold)

Points  : N × 28 bytes
  Each point: 7 × float32 — pos_x, pos_y, pos_z, norm_x, norm_y, norm_z, weight

Normals are *inward-facing* (pointing into the surface, i.e. downward for
the upper wing). The script flips face normals automatically.
"""

import bpy
import bmesh
import struct
import math
import os
from mathutils import Vector, Matrix


# -----------------------------------------------------------------------
# Coordinate transform helpers
# -----------------------------------------------------------------------

def blender_to_dcs(v, axis_map):
    """
    Transform a Blender vector to DCS model space.
    axis_map is a 3-tuple of (dcs_component, blender_component, sign), e.g.:
      [('x', 'y', +1), ('y', 'z', +1), ('z', 'x', +1)]
    means DCS.x = +Blender.y, DCS.y = +Blender.z, DCS.z = +Blender.x
    """
    result = [0.0, 0.0, 0.0]
    for i, (_, src, sign) in enumerate(axis_map):
        result[i] = sign * getattr(v, src)
    return Vector(result)


# -----------------------------------------------------------------------
# OWV writer
# -----------------------------------------------------------------------

def build_cdf(n_points, n_bins=10):
    """
    Build a uniform cumulative distribution function table.
    Returns list of (cumulative_count: int, threshold: float).
    """
    cdf = []
    for i in range(n_bins):
        cumulative = int(round(n_points * (i + 1) / n_bins))
        threshold = (i + 1) / n_bins          # 0.1, 0.2, ..., 1.0
        cdf.append((cumulative, threshold))
    return cdf


def write_owv(filepath, points_dcs, n_cdf_bins=10):
    """
    Write .owv binary file.

    Parameters
    ----------
    filepath   : output path (str)
    points_dcs : list of 7-tuples (px, py, pz, nx, ny, nz, weight)
                 in DCS model space with *inward* normals
    n_cdf_bins : number of CDF bins (almost certainly 10 for DCS)
    """
    n_points = len(points_dcs)

    # --- Metadata floats (bytes 4-40) ---
    # Always compute from actual point data — preserving from an existing file
    # with a different point count crashes DCS.
    meta_floats = [0.0] * 9
    if n_points:
        xs = [p[0] for p in points_dcs]
        ys = [p[1] for p in points_dcs]
        zs = [p[2] for p in points_dcs]
        meta_floats[0] = min(xs);  meta_floats[1] = max(xs)
        meta_floats[2] = min(ys);  meta_floats[3] = max(ys)
        meta_floats[4] = min(zs);  meta_floats[5] = max(zs)
        meta_floats[6] = 0.0
        meta_floats[7] = 0.0
        meta_floats[8] = 0.0  # NOT n_points — DCS misuses this value; old working files have ~0 here

    # --- CDF table ---
    cdf = build_cdf(n_points, n_cdf_bins)

    # --- Verify header size ---
    header_size = 4 + 9*4 + 4 + 4 + n_cdf_bins * 8   # = 128 for n_cdf_bins=10
    assert header_size == 128, f"Header size {header_size} != 128"

    with open(filepath, 'wb') as f:
        f.write(b'owv\x00')                           # magic
        f.write(struct.pack('<9f', *meta_floats))      # metadata
        f.write(struct.pack('<I', 1))                  # version
        f.write(struct.pack('<I', n_cdf_bins))         # CDF count
        for count, threshold in cdf:
            f.write(struct.pack('<If', count, threshold))
        for pt in points_dcs:
            f.write(struct.pack('<7f', *pt))

    print(f"  Wrote {n_points} spawn points → {filepath}")


# -----------------------------------------------------------------------
# Triangle subdivision helpers
# -----------------------------------------------------------------------

def triangle_grid_points(v0, v1, v2, n):
    """
    Return the centroids of all n² sub-triangles formed by uniformly
    subdividing triangle (v0, v1, v2) into an n×n grid.

    For n=1 returns [centroid], for n=2 returns 4 points, n=3 → 9, etc.
    """
    pts = []
    inv_n = 1.0 / n
    for row in range(n):
        cols = n - row
        for col in range(cols):
            # "downward" sub-triangle centroid
            a = (row + 1.0/3.0) * inv_n
            b = (col + 1.0/3.0) * inv_n
            c = 1.0 - a - b
            pts.append(v0 * a + v1 * b + v2 * c)
            # "upward" sub-triangle centroid (exists for all but the last col)
            if col < cols - 1:
                a2 = (row + 2.0/3.0) * inv_n
                b2 = (col + 2.0/3.0) * inv_n
                c2 = 1.0 - a2 - b2
                pts.append(v0 * a2 + v1 * b2 + v2 * c2)
    return pts


def subdivide_face(face, world, max_face_area):
    """
    Fan-triangulate a BMFace and subdivide any triangle larger than max_face_area.

    Returns a list of (pos_world: Vector, sub_area: float).
    The normal is NOT returned here; the face normal is used by the caller.
    """
    verts_world = [world @ v.co for v in face.verts]
    if len(verts_world) < 3:
        return []

    # Fan triangulation from the first vertex
    v0 = verts_world[0]
    triangles = [
        (v0, verts_world[i], verts_world[i + 1])
        for i in range(1, len(verts_world) - 1)
    ]

    samples = []
    for v0, v1, v2 in triangles:
        edge1 = v1 - v0
        edge2 = v2 - v0
        tri_area = edge1.cross(edge2).length * 0.5
        if tri_area < 1e-12:
            continue
        n = max(1, math.ceil(math.sqrt(tri_area / max_face_area)))
        sub_area = tri_area / (n * n)
        for pt in triangle_grid_points(v0, v1, v2, n):
            samples.append((pt, sub_area))
    return samples


# -----------------------------------------------------------------------
# Blender mesh sampling
# -----------------------------------------------------------------------

def sample_faces(obj, axis_map, min_up_dot=0.1, flip_normals=True,
                 weight_by_area=True, max_face_area=None):
    """
    Sample faces from a mesh object, returning spawn points in DCS model space.

    Face selection priority:
      1. If the object is in Edit Mode with faces manually selected → use those.
      2. Otherwise → use all faces whose world-space normal has a DCS-Y (up)
         component >= min_up_dot (i.e. upward-facing surfaces only).

    Parameters
    ----------
    min_up_dot    : dot-product threshold against DCS up-axis. 0.1 keeps slanted
                    wing panels while rejecting the underside.
    flip_normals  : flip face normals before storing (OWV expects inward-facing).
    max_face_area : if set (m²), large faces are tessellated and subdivided into
                    sub-triangles so no sample point covers more than this area.
                    E.g. 0.25 gives one point per ~0.25 m² of wing surface.
                    None = one point per face (original behaviour).
    """
    if obj.type != 'MESH':
        print(f"  WARNING: {obj.name} is not a mesh — skipped.")
        return []

    in_edit_with_selection = False
    if obj.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        selected = [f for f in bm.faces if f.select]
        if selected:
            faces = selected
            in_edit_with_selection = True
            print(f"  Using {len(faces)} manually-selected faces from {obj.name}.")
        else:
            faces = list(bm.faces)
        free_bm = False
    else:
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        faces = list(bm.faces)
        free_bm = True

    world = obj.matrix_world
    rot   = world.to_3x3().normalized()   # rotation-only (no scale) for normals

    # If not using a manual selection, filter by upward-facing normal
    if not in_edit_with_selection:
        up_blender = Vector(blender_to_dcs(Vector((0, 0, 1)), axis_map))  # DCS up in blender terms — actually just need the DCS Y axis direction
        # The DCS Y axis corresponds to: from axis_map, find which blender axis maps to DCS index 1
        # Simpler: just compute the DCS-space normal for each face and check .y
        filtered = []
        for face in faces:
            n_world = (rot @ face.normal).normalized()
            n_dcs   = blender_to_dcs(n_world, axis_map)
            if n_dcs.y >= min_up_dot:
                filtered.append(face)
        print(f"  Normal filter (min_up_dot={min_up_dot}): {len(filtered)}/{len(faces)} faces kept from {obj.name}.")
        faces = filtered

    if not faces:
        print(f"  WARNING: No faces passed the filter on {obj.name}.")
        if free_bm:
            bm.free()
        return []

    total_area = sum(f.calc_area() for f in faces) if weight_by_area else 1.0

    points = []
    for face in faces:
        norm_world = (rot @ face.normal).normalized()
        norm_out   = -norm_world if flip_normals else norm_world
        norm_dcs   = blender_to_dcs(norm_out, axis_map)

        if max_face_area is not None:
            # Subdivide large faces — each sub-triangle gets its own sample point
            sub_samples = subdivide_face(face, world, max_face_area)
            for pos_world, sub_area in sub_samples:
                pos_dcs = blender_to_dcs(pos_world, axis_map)
                weight  = (sub_area / total_area) if weight_by_area and total_area > 0 else 1.0
                points.append((
                    pos_dcs.x,  pos_dcs.y,  pos_dcs.z,
                    norm_dcs.x, norm_dcs.y, norm_dcs.z,
                    weight,
                ))
        else:
            # One point per face at the face centroid
            pos_world = world @ face.calc_center_median()
            pos_dcs   = blender_to_dcs(pos_world, axis_map)
            area      = face.calc_area()
            weight    = (area / total_area) if weight_by_area and total_area > 0 else 1.0 / len(faces)
            points.append((
                pos_dcs.x,  pos_dcs.y,  pos_dcs.z,
                norm_dcs.x, norm_dcs.y, norm_dcs.z,
                weight,
            ))

    if max_face_area is not None:
        print(f"  Subdivided {len(faces)} faces → {len(points)} sample points (max_face_area={max_face_area} m²).")

    if free_bm:
        bm.free()
    return points


# -----------------------------------------------------------------------
# Main entry point
# -----------------------------------------------------------------------

def generate_owv(output_path, axis_map=None, min_up_dot=0.1, max_face_area=None):
    """
    Export upward-facing mesh faces to an .owv file.

    Parameters
    ----------
    output_path   : where to write the .owv
    axis_map      : coordinate axis mapping (see CONFIGURATION below)
    min_up_dot    : faces with DCS-Y normal component < this are discarded
    max_face_area : subdivide faces larger than this (m²); None = one point/face
    """
    if axis_map is None:
        axis_map = [
            ('x', 'y', +1),
            ('y', 'z', +1),
            ('z', 'x', +1),
        ]

    selected = [o for o in bpy.context.selected_objects if o.type == 'MESH']
    if not selected:
        print("ERROR: No mesh objects selected.")
        return

    all_points = []
    for obj in selected:
        all_points.extend(sample_faces(obj, axis_map, min_up_dot=min_up_dot,
                                       max_face_area=max_face_area))

    if not all_points:
        print("ERROR: No faces passed the filter. Try lowering MIN_UP_DOT.")
        return

    # Normalise weights so they sum to 1
    total_w = sum(p[6] for p in all_points)
    if total_w > 0:
        all_points = [p[:6] + (p[6] / total_w,) for p in all_points]

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    write_owv(output_path, all_points)
    print("Done.")


# =======================================================================
# CONFIGURATION  ← edit these before running
# =======================================================================

# Output path for the new .owv file
OUTPUT_PATH = r"E:\Saved Games\DCS\Mods\aircraft\F-117-Nighthawk\Shapes\f-117-nighthawk_overwingVapor.owv"

# Axis mapping: list of 3 triples (ignored_label, blender_axis, sign)
# Assumes the Blender model is oriented with:
#   Y = aircraft nose direction (forward)
#   Z = up
#   X = right (starboard)
# Adjust if your model uses a different convention.
AXIS_MAP = [
    ('x', 'y', +1),   # DCS +X (forward) ← Blender +Y
    ('y', 'z', +1),   # DCS +Y (up)      ← Blender +Z
    ('z', 'x', +1),   # DCS +Z (right)   ← Blender +X
]

# Minimum dot product of face normal against DCS up-axis (+Y) for a face
# to be included.  Range: -1.0 (any direction) to 1.0 (flat top only).
#   0.1  → keeps slanted wing panels, rejects underside  (recommended)
#   0.3  → stricter: only moderately-upward faces
#   0.0  → keeps all faces pointing even slightly upward
MIN_UP_DOT = 0.1

# Maximum face area (m²) before a face is subdivided into sub-triangle samples.
# Set to None to use one point per face (original behaviour).
# The F-117 wing is ~84 m² total; with 185 upper faces that's ~0.45 m²/face avg.
#   0.25 → ~2× more points  (~370 total)
#   0.10 → ~5× more points  (~900 total)
#   0.05 → ~10× more points (~1800 total)
MAX_FACE_AREA = 0.25   # m²

# =======================================================================
# RUN
# =======================================================================
generate_owv(OUTPUT_PATH, AXIS_MAP,
             min_up_dot=MIN_UP_DOT, max_face_area=MAX_FACE_AREA)

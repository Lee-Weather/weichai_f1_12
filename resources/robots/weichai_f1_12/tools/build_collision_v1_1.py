"""Build the v1.1 calf collision hulls in link-local coordinates (meters).

The exact visual meshes and foot collision hulls are intentionally retained.
The distal calf hull is shortened to clear the foot's complete ankle sweep.
This offline asset tool requires numpy, scipy (via trimesh), and trimesh.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ASSET = Path(__file__).resolve().parents[1]
URDF = ASSET / "urdf/weichai_f1_12.urdf"
DESTINATION = ASSET / "meshes/collision/v1_1"
MIN_VERTICAL_CLEARANCE = 0.025
SOURCE_CALVES = {
    "left": "left_knee_pitch_e156674d2a81_exact_p01.stl",
    "right": "right_knee_pitch_d7c98c8e9717_exact_p01.stl",
}


def collision_vertices(element):
    """Return the referenced collision vertices in their owning link frame."""
    mesh = element.find("geometry/mesh")
    loaded = trimesh.load(URDF.parent / mesh.get("filename"), force="mesh", process=False)
    vertices = loaded.vertices * np.array(list(map(float, mesh.get("scale", "1 1 1").split())))
    origin = element.find("origin")
    if origin is None:
        return vertices
    transform = trimesh.transformations.euler_matrix(*map(float, origin.get("rpy", "0 0 0").split()))
    transform[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
    return vertices @ transform[:3, :3].T + transform[:3, 3]


def original_calf(side):
    source = ASSET / "meshes/mujoco_stl" / SOURCE_CALVES[side]
    vertices = trimesh.load(source, force="mesh", process=False).vertices * .001
    transform = trimesh.transformations.euler_matrix(1.5708, 0, 1.5708)
    transform[:3, 3] = [0, -.18295 if side == "left" else .18295, .236]
    return trimesh.convex.convex_hull(vertices @ transform[:3, :3].T + transform[:3, 3])


def max_swept_z(vertices, pitch_limits, roll_limits):
    """Exact vertex support maximum for Ry(pitch) Rx(roll), |pitch| < pi/2.

    z' = -x sin(p) + cos(p) (y sin(r) + z cos(r)). Since cos(p) > 0,
    each vertex can first maximize over r, then over p. Each one-dimensional
    sinusoid reaches its maximum at an interval endpoint or its stationary
    maximum. Mesh convexification cannot increase a linear support maximum.
    """
    assert -math.pi / 2 < pitch_limits[0] <= pitch_limits[1] < math.pi / 2
    x, y, z = vertices.T
    r_peak = np.arctan2(y, z)
    r_values = np.array([
        y * np.sin(roll_limits[0]) + z * np.cos(roll_limits[0]),
        y * np.sin(roll_limits[1]) + z * np.cos(roll_limits[1]),
        np.where((r_peak >= roll_limits[0]) & (r_peak <= roll_limits[1]),
                 np.hypot(y, z), -np.inf),
    ])
    rz = r_values.max(axis=0)
    p_peak = np.arctan2(-x, rz)
    maxima = np.array([
        -x * np.sin(pitch_limits[0]) + rz * np.cos(pitch_limits[0]),
        -x * np.sin(pitch_limits[1]) + rz * np.cos(pitch_limits[1]),
        np.where((p_peak >= pitch_limits[0]) & (p_peak <= pitch_limits[1]),
                 np.hypot(x, rz), -np.inf),
    ])
    return float(maxima.max())


def clip_hull_above(hull, plane_z):
    """Clip a convex hull by z >= plane_z, retaining its original upper shape."""
    vertices = hull.vertices
    edge_vertices = vertices[hull.edges_unique]
    first, second = edge_vertices[:, 0], edge_vertices[:, 1]
    crosses = ((first[:, 2] < plane_z) & (second[:, 2] > plane_z)) | (
        (first[:, 2] > plane_z) & (second[:, 2] < plane_z))
    first, second = first[crosses], second[crosses]
    amount = (plane_z - first[:, 2]) / (second[:, 2] - first[:, 2])
    intersections = first + amount[:, None] * (second - first)
    retained = np.vstack([vertices[vertices[:, 2] >= plane_z], intersections])
    result = trimesh.convex.convex_hull(retained)
    if not result.is_watertight or not result.is_convex or result.volume <= 0:
        raise ValueError("Generated calf collider is not a closed positive-volume convex hull")
    return result


def main():
    tree = ET.parse(URDF).getroot()
    details = {}
    for side in ("left", "right"):
        foot = tree.find(f"link[@name='{side}_ankle_roll_link']/collision")
        foot_mesh = foot.find("geometry/mesh")
        pitch = tree.find(f"joint[@name='{side}_ankle_pitch_joint']")
        roll = tree.find(f"joint[@name='{side}_ankle_roll_joint']")
        assert list(map(float, pitch.find("axis").get("xyz").split())) == [0, 1, 0]
        assert list(map(float, roll.find("axis").get("xyz").split())) == [1, 0, 0]
        assert list(map(float, roll.find("origin").get("xyz").split())) == [0, 0, 0]
        for joint in (pitch, roll):
            assert list(map(float, joint.find("origin").get("rpy", "0 0 0").split())) == [0, 0, 0]
        limits = lambda joint: [float(joint.find("limit").get(k)) for k in ("lower", "upper")]
        pitch_limits, roll_limits = limits(pitch), limits(roll)
        ankle_z = float(pitch.find("origin").get("xyz").split()[2])
        sweep_height = max_swept_z(collision_vertices(foot), pitch_limits, roll_limits)
        details[side] = {"ankle_pitch_limits_rad": pitch_limits, "ankle_roll_limits_rad": roll_limits,
                         "ankle_z_in_calf_frame_m": ankle_z,
                         "max_foot_sweep_z_in_ankle_frame_m": sweep_height,
                         "foot_collision_sha256": hashlib.sha256((URDF.parent / foot_mesh.get("filename")).read_bytes()).hexdigest()}
    # Use the same plane for both legs, rounded away from the moving foot.
    plane_z = math.ceil(max(d["ankle_z_in_calf_frame_m"] + d["max_foot_sweep_z_in_ankle_frame_m"]
                            for d in details.values()) * 1000 + MIN_VERTICAL_CLEARANCE * 1000) / 1000
    DESTINATION.mkdir(parents=True, exist_ok=True)
    for side, detail in details.items():
        old = original_calf(side)
        new = clip_hull_above(old, plane_z)
        destination = DESTINATION / f"{side}_knee_pitch_collision.stl"
        new.export(destination)
        source = ASSET / "meshes/mujoco_stl" / SOURCE_CALVES[side]
        detail.update({"source_mesh": source.relative_to(ASSET).as_posix(),
                       "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                       "mesh": destination.relative_to(ASSET).as_posix(),
                       "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
                       "original_bounds_m": old.bounds.tolist(), "bounds_m": new.bounds.tolist(),
                       "volume_retained_fraction": float(new.volume / old.volume),
                       "minimum_vertical_sweep_clearance_m": plane_z - detail["ankle_z_in_calf_frame_m"]
                           - detail["max_foot_sweep_z_in_ankle_frame_m"]})
    metadata = {"model": "weichai_f1_12", "version": "1.1", "units": "meter",
                "collision_method": "convex calf hull clipped above complete ankle-foot sweep",
                "calf_clip_min_z_m": plane_z, "requested_vertical_clearance_m": MIN_VERTICAL_CLEARANCE,
                "legs": details}
    (ASSET / "collision_geometry_v1_1.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()

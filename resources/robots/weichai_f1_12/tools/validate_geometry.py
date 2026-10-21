"""Regression checks for F1 v1.1 collision assets, using local MuJoCo.

These checks do not replace an Isaac Gym/PhysX runtime regression.
Dependencies: numpy, trimesh, MuJoCo with mj_geomDistance (tested on 3.15).
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

from build_collision_v1_1 import ASSET, URDF, collision_vertices, max_swept_z

MJCF = ASSET / "mjcf/weichai_f1_12.xml"
JOINT_NAMES = [f"{side}_{joint}_joint" for side in ("left", "right")
               for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch", "ankle_roll")]


def load_urdf(root):
    root = ET.fromstring(ET.tostring(root))
    compiler = root.find("mujoco/compiler")
    compiler.attrib.update({"strippath": "false", "discardvisual": "true", "meshdir": "", "fusestatic": "false"})
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str((URDF.parent / mesh.get("filename")).resolve()))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    # URDF import fixes the root; restore the floating base used in training.
    with tempfile.NamedTemporaryFile(suffix=".xml", delete=False) as handle:
        temporary = Path(handle.name)
    try:
        mujoco.mj_saveLastXML(str(temporary), model)
        scene = ET.parse(temporary).getroot()
        base = scene.find("worldbody/body[@name='base_link']")
        base.set("pos", "0 0 0.7")
        ET.SubElement(base, "freejoint", {"name": "validation_root"})
        ET.SubElement(scene.find("worldbody"), "geom", {"name": "ground", "type": "plane", "size": "0 0 0.1"})
        return mujoco.MjModel.from_xml_string(ET.tostring(scene, encoding="unicode"))
    finally:
        temporary.unlink()


def absolute_mjcf(root):
    root = ET.fromstring(ET.tostring(root))
    root.find("compiler").attrib.update({"meshdir": "", "strippath": "false"})
    for mesh in root.findall("asset/mesh"):
        mesh.set("file", str((MJCF.parent / mesh.get("file")).resolve()))
    return root


def contact_rows(model, data):
    return [{"bodies": [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g]))
                         for g in c.geom], "distance_m": float(c.dist)} for c in data.contact]


def set_pose(model, data, joint_values, height=.7):
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0
    data.qpos[2] = height
    for name, value in zip(JOINT_NAMES, joint_values):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_forward(model, data)


def reference_offsets():
    cfg = ASSET.parents[2] / "humanoid/envs/weichai_f1_12_config.py"
    for node in ast.walk(ast.parse(cfg.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "final_swing_joint_delta_pos"
                                                for t in node.targets):
            return np.array(ast.literal_eval(node.value), dtype=float)
    raise AssertionError("F1 reference joint offsets not found")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ASSET / "validation_v1_1.json")
    args = parser.parse_args()
    urdf = ET.parse(URDF).getroot()
    mjcf = ET.parse(MJCF).getroot()
    metadata = json.loads((ASSET / "collision_geometry_v1_1.json").read_text(encoding="utf-8"))
    assert (ASSET / "VERSION").read_text().strip() == metadata["version"] == "1.1"
    assert urdf.get("name") == mjcf.get("model") == "weichai_f1_12_v1_1"
    assert [j.get("name") for j in urdf.findall("joint") if j.get("type") != "fixed"] == JOINT_NAMES
    assert [a.get("joint") for a in mjcf.find("actuator")] == JOINT_NAMES
    assert not mjcf.findall("contact/exclude"), "The geometry fix must not depend on new collision exclusions"
    for element in urdf.findall(".//mesh"):
        path = (URDF.parent / element.get("filename")).resolve()
        path.relative_to(ASSET)
        assert path.is_file(), path
    for element in mjcf.findall("asset/mesh"):
        path = (MJCF.parent / element.get("file")).resolve()
        path.relative_to(ASSET)
        assert path.is_file(), path
    # Exercise the actual on-disk relative paths, including visual assets.
    mujoco.MjModel.from_xml_path(str(URDF))

    clearances = {}
    for side, info in metadata["legs"].items():
        collision = urdf.find(f"link[@name='{side}_knee_pitch_link']/collision")
        foot = urdf.find(f"link[@name='{side}_ankle_roll_link']/collision")
        asset_path = ASSET / info["mesh"]
        assert hashlib.sha256(asset_path.read_bytes()).hexdigest() == info["sha256"]
        assert (URDF.parent / collision.find("geometry/mesh").get("filename")).resolve() == asset_path
        foot_path = (URDF.parent / foot.find("geometry/mesh").get("filename")).resolve()
        assert hashlib.sha256(foot_path.read_bytes()).hexdigest() == info["foot_collision_sha256"]
        hull = trimesh.load(asset_path, force="mesh")
        assert hull.is_watertight and hull.is_convex and hull.volume > 0
        calf_vertices = collision_vertices(collision)
        foot_vertices = collision_vertices(foot)
        sweep_max = max_swept_z(foot_vertices, info["ankle_pitch_limits_rad"], info["ankle_roll_limits_rad"])
        clearance = float(calf_vertices[:, 2].min() - info["ankle_z_in_calf_frame_m"] - sweep_max)
        assert clearance >= metadata["requested_vertical_clearance_m"] - 1e-6, (side, clearance)
        clearances[side] = clearance

    source = absolute_mjcf(mjcf)
    # Recover the pre-fix collider for a positive regression control.
    legacy = ET.fromstring(ET.tostring(source))
    for side, info in metadata["legs"].items():
        geom = legacy.find(f".//geom[@name='{side}_knee_pitch_collision']")
        elem = legacy.find(f"asset/mesh[@name='{geom.get('mesh')}']")
        elem.set("file", str(ASSET / info["source_mesh"]))
        elem.set("scale", ".001 .001 .001")
        geom.set("pos", f"0 {'-.18295' if side == 'left' else '.18295'} .236")
        geom.set("quat", ".499998 .5 .500002 .5")
    baseline = mujoco.MjModel.from_xml_string(ET.tostring(legacy, encoding="unicode"))
    baseline_data = mujoco.MjData(baseline)
    set_pose(baseline, baseline_data, np.zeros(12))
    old_contacts = contact_rows(baseline, baseline_data)
    assert len(old_contacts) == 4 and min(c["distance_m"] for c in old_contacts) < -.027

    models = {"urdf_import": load_urdf(urdf),
              "native_mjcf": mujoco.MjModel.from_xml_path(str(MJCF))}
    offsets = reference_offsets()
    results = {}
    for label, model in models.items():
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
                 for i in range(model.njnt) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE]
        assert names == JOINT_NAMES
        data = mujoco.MjData(model)
        set_pose(model, data, np.zeros(12))
        assert data.ncon == 0, (label, "zero_pose", contact_rows(model, data))
        set_pose(model, data, np.array([-.1, 0, 0, .2, -.1, 0] * 2))
        assert data.ncon == 0, (label, "bent_stance", contact_rows(model, data))
        for phase in np.linspace(0, 1, 201):
            sine = np.sin(2 * np.pi * phase)
            values = offsets * np.array([max(0, -sine)] * 6 + [max(0, sine)] * 6)
            if abs(sine) < .1:
                values[:] = 0
            set_pose(model, data, values)
            assert data.ncon == 0, (label, "reference_gait", phase, contact_rows(model, data))
        min_distance = {side: float("inf") for side in ("left", "right")}
        pairs = {side: [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_{part}_collision")
                        for part in ("knee_pitch", "ankle_roll")] for side in min_distance}
        for pitch in np.linspace(-.523599, .523599, 41):
            for roll in np.linspace(-.523599, .523599, 41):
                values = np.zeros(12)
                values[[4, 10]] = pitch
                values[[5, 11]] = [roll, -roll]
                set_pose(model, data, values)
                assert data.ncon == 0, (label, "ankle_limits", pitch, roll, contact_rows(model, data))
                for side, pair in pairs.items():
                    distance = float(mujoco.mj_geomDistance(model, data, *pair, 1, None))
                    min_distance[side] = min(min_distance[side], distance)
        assert min(min_distance.values()) > .025, min_distance
        # Verify that both original sole colliders still contact the floor.
        set_pose(model, data, np.zeros(12), height=.5995)
        floor_contacts = contact_rows(model, data)
        feet = {name for c in floor_contacts for name in c["bodies"] if name != "world"}
        assert feet == {"left_ankle_roll_link", "right_ankle_roll_link"}, (label, floor_contacts)
        assert all("world" in c["bodies"] for c in floor_contacts), floor_contacts
        results[label] = {"zero_pose_internal_contacts": 0, "bent_stance_internal_contacts": 0,
                          "reference_gait_samples": 201, "ankle_sweep_samples": 1681,
                          "minimum_sampled_calf_foot_distance_m": min_distance,
                          "ground_contact_bodies": sorted(feet)}
    summary = {"version": "1.1", "mujoco_version": mujoco.__version__,
               "on_disk_urdf_and_mjcf_relative_paths": "passed",
               "baseline_zero_pose_contacts": old_contacts,
               "analytical_minimum_vertical_clearance_m": clearances,
               "checks": results,
               "scope": "Local convex geometry regression. Isaac Gym/PhysX runtime and training performance not measured."}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

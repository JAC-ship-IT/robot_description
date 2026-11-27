"""Convert a MuJoCo MJCF kinematic tree to URDF.

The converter keeps the body tree, joint axes and limits, inertias, visual
meshes, and collision primitives. MuJoCo capsules become URDF cylinders of the
same radius and cylindrical length; the hemispherical caps are not representable
in URDF. Equality polynomials, sites, sensors, actuators, and contact excludes
are written as comments because URDF has no equivalent.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def _floats(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if text is None:
        return default
    return tuple(float(part) for part in text.split())


def _fmt(value: float | tuple[float, ...]) -> str:
    if isinstance(value, tuple):
        return " ".join(_fmt(item) for item in value)
    if abs(value) < 1e-12:
        return "0"
    return f"{value:.9g}"


def _quat_wxyz_to_matrix(quat: tuple[float, float, float, float]) -> list[list[float]]:
    w, x, y, z = quat
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        raise ValueError(f"Zero quaternion: {quat}")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def _matvec(matrix: list[list[float]], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))


def _sub(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(ai - bi for ai, bi in zip(a, b))


def _add(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(ai + bi for ai, bi in zip(a, b))


def _matrix_to_rpy(matrix: list[list[float]]) -> tuple[float, float, float]:
    """Extrinsic XYZ roll-pitch-yaw, the URDF rpy convention."""
    cy = math.sqrt(matrix[0][0] ** 2 + matrix[1][0] ** 2)
    if cy > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        pitch = math.atan2(-matrix[2][0], cy)
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        pitch = math.atan2(-matrix[2][0], cy)
        yaw = 0.0
    return roll, pitch, yaw


def _rpy_from_quat(quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    return _matrix_to_rpy(_quat_wxyz_to_matrix(quat))


def _body_rotation(body: ET.Element, degrees: bool) -> tuple[list[list[float]], tuple[float, float, float]]:
    if body.get("quat") is not None:
        quat = _floats(body.get("quat"), (1.0, 0.0, 0.0, 0.0))
        if len(quat) != 4:
            raise ValueError(f"Body {body.get('name')} quat must have 4 values")
        matrix = _quat_wxyz_to_matrix(quat)  # type: ignore[arg-type]
        return matrix, _matrix_to_rpy(matrix)
    if body.get("euler") is not None:
        euler = _floats(body.get("euler"), (0.0, 0.0, 0.0))
        if degrees:
            euler = tuple(math.radians(value) for value in euler)
        roll, pitch, yaw = euler
        cx, sx = math.cos(roll), math.sin(roll)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cz, sz = math.cos(yaw), math.sin(yaw)
        # R = Rz(yaw) Ry(pitch) Rx(roll)
        matrix = [
            [cy * cz, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [cy * sz, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ]
        return matrix, (roll, pitch, yaw)
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], (0.0, 0.0, 0.0)


def _mesh_files(root: ET.Element) -> dict[str, str]:
    files: dict[str, str] = {}
    asset = root.find("asset")
    if asset is None:
        return files
    for mesh in asset.findall("mesh"):
        name = mesh.get("name")
        filename = mesh.get("file")
        if name and filename:
            files[name] = filename
    return files


def _actuator_effort(root: ET.Element) -> dict[str, float]:
    effort: dict[str, float] = {}
    actuator = root.find("actuator")
    if actuator is None:
        return effort
    for element in actuator:
        joint = element.get("joint")
        force = element.get("forcerange")
        if joint and force:
            low, high = _floats(force, (0.0, 0.0))
            effort[joint] = max(abs(low), abs(high))
    return effort


def _origin(parent: ET.Element, xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> None:
    ET.SubElement(parent, "origin", {"xyz": _fmt(xyz), "rpy": _fmt(rpy)})


def _geometry(parent: ET.Element, geom: ET.Element, meshes: dict[str, str]) -> None:
    kind = geom.get("type", "sphere")
    size = _floats(geom.get("size"), ())
    geometry = ET.SubElement(parent, "geometry")
    if kind == "box":
        if len(size) != 3:
            raise ValueError("Box geom requires 3 half-sizes")
        ET.SubElement(geometry, "box", {"size": _fmt(tuple(2.0 * value for value in size))})
    elif kind in ("capsule", "cylinder"):
        if len(size) < 2:
            raise ValueError(f"{kind} geom requires radius and half-length")
        # MuJoCo's second size is the cylinder half-length. Capsule end caps have no URDF tag.
        ET.SubElement(geometry, "cylinder", {"radius": _fmt(size[0]), "length": _fmt(2.0 * size[1])})
    elif kind == "sphere":
        if len(size) < 1:
            raise ValueError("Sphere geom requires a radius")
        ET.SubElement(geometry, "sphere", {"radius": _fmt(size[0])})
    elif kind == "mesh":
        mesh_name = geom.get("mesh")
        if mesh_name not in meshes:
            raise ValueError(f"Geom references unknown mesh {mesh_name!r}")
        ET.SubElement(geometry, "mesh", {"filename": meshes[mesh_name]})
    else:
        raise ValueError(f"Unsupported geom type {kind!r}")


def _append_geoms(link: ET.Element, body: ET.Element, meshes: dict[str, str], shift: tuple[float, float, float]) -> None:
    for geom in body.findall("geom"):
        contype = int(geom.get("contype", "1"))
        group = int(geom.get("group", "0"))
        is_visual = contype == 0 or group <= 2
        is_collision = contype != 0
        quat = _floats(geom.get("quat"), (1.0, 0.0, 0.0, 0.0))
        if len(quat) != 4:
            raise ValueError(f"Geom quat on {body.get('name')} must have 4 values")
        pos = _sub(_floats(geom.get("pos"), (0.0, 0.0, 0.0)), shift)
        rpy = _rpy_from_quat(quat)  # type: ignore[arg-type]
        if is_visual:
            visual = ET.SubElement(link, "visual")
            _origin(visual, pos, rpy)  # type: ignore[arg-type]
            _geometry(visual, geom, meshes)
            if geom.get("rgba"):
                material = ET.SubElement(visual, "material", {"name": ""})
                ET.SubElement(material, "color", {"rgba": geom.get("rgba", "")})
        if is_collision:
            collision = ET.SubElement(link, "collision")
            _origin(collision, pos, rpy)  # type: ignore[arg-type]
            _geometry(collision, geom, meshes)


def _append_link(robot: ET.Element, body: ET.Element, meshes: dict[str, str], shift: tuple[float, float, float]) -> None:
    link = ET.SubElement(robot, "link", {"name": body.get("name", "")})
    inertial = body.find("inertial")
    if inertial is not None:
        node = ET.SubElement(link, "inertial")
        pos = _sub(_floats(inertial.get("pos"), (0.0, 0.0, 0.0)), shift)
        quat = _floats(inertial.get("quat"), (1.0, 0.0, 0.0, 0.0))
        _origin(node, pos, _rpy_from_quat(quat))  # type: ignore[arg-type]
        ET.SubElement(node, "mass", {"value": _fmt(float(inertial.get("mass", "0")))})
        diag = _floats(inertial.get("diaginertia"), (0.0, 0.0, 0.0))
        if len(diag) != 3:
            raise ValueError(f"Inertial on {body.get('name')} requires diaginertia")
        ET.SubElement(node, "inertia", {
            "ixx": _fmt(diag[0]), "ixy": "0", "ixz": "0",
            "iyy": _fmt(diag[1]), "iyz": "0", "izz": _fmt(diag[2]),
        })
    _append_geoms(link, body, meshes, shift)


def _append_joint(
    robot: ET.Element,
    body: ET.Element,
    parent_name: str,
    parent_shift: tuple[float, float, float],
    degrees: bool,
    effort_by_joint: dict[str, float],
) -> tuple[float, float, float]:
    joints = body.findall("joint")
    if len(joints) > 1:
        raise ValueError(f"Body {body.get('name')} has {len(joints)} joints; URDF needs one joint per link")
    pos = _sub(_floats(body.get("pos"), (0.0, 0.0, 0.0)), parent_shift)
    rotation, rpy = _body_rotation(body, degrees)
    joint_pos = (0.0, 0.0, 0.0)
    if joints:
        joint = joints[0]
        kind = joint.get("type", "hinge")
        if kind not in ("hinge", "revolute"):
            raise ValueError(f"Joint {joint.get('name')} has unsupported type {kind!r}")
        joint_pos = _floats(joint.get("pos"), (0.0, 0.0, 0.0))
        origin = _add(pos, _matvec(rotation, joint_pos))
        node = ET.SubElement(robot, "joint", {"name": joint.get("name", ""), "type": "revolute"})
        _origin(node, origin, rpy)  # type: ignore[arg-type]
        ET.SubElement(node, "parent", {"link": parent_name})
        ET.SubElement(node, "child", {"link": body.get("name", "")})
        axis = _floats(joint.get("axis"), (0.0, 0.0, 1.0))
        ET.SubElement(node, "axis", {"xyz": _fmt(axis)})
        lower, upper = _floats(joint.get("range"), (0.0, 0.0))
        if degrees:
            lower, upper = math.radians(lower), math.radians(upper)
        effort = effort_by_joint.get(joint.get("name", ""), 0.0)
        # MJCF has no joint speed limit. URDF requires one; 0 would freeze the joint in PhysX.
        ET.SubElement(node, "limit", {
            "lower": _fmt(lower), "upper": _fmt(upper),
            "effort": _fmt(effort), "velocity": "100",
        })
        dynamics = {}
        if joint.get("damping") is not None:
            dynamics["damping"] = _fmt(float(joint.get("damping", "0")))
        if joint.get("frictionloss") is not None:
            dynamics["friction"] = _fmt(float(joint.get("frictionloss", "0")))
        if dynamics:
            ET.SubElement(node, "dynamics", dynamics)
        if joint.get("armature") is not None:
            node.append(ET.Comment(f" armature={joint.get('armature')} has no URDF field "))
        if joint.get("name") not in effort_by_joint:
            node.append(ET.Comment(" no MJCF actuator; effort left at 0 "))
    else:
        node = ET.SubElement(robot, "joint", {"name": f"{body.get('name')}_fixed", "type": "fixed"})
        _origin(node, pos, rpy)  # type: ignore[arg-type]
        ET.SubElement(node, "parent", {"link": parent_name})
        ET.SubElement(node, "child", {"link": body.get("name", "")})
    return joint_pos


def _header_comments(root: ET.Element) -> list[str]:
    comments = [
        " Generated from MJCF. Capsules are cylinders of the cylindrical section only. ",
        " Joint velocity is 100 rad/s because the MJCF specifies no speed limit. ",
        " Dropped: sites, sensors, actuators, keyframes, and contact excludes. ",
    ]
    equality = root.find("equality")
    if equality is not None:
        for element in equality.findall("joint"):
            comments.append(
                " equality "
                f"{element.get('joint1')} = poly({element.get('joint2')}) "
                f"coef={element.get('polycoef')} "
            )
    contact = root.find("contact")
    if contact is not None:
        for element in contact.findall("exclude"):
            comments.append(f" contact exclude {element.get('body1')} {element.get('body2')} ")
    return comments


def convert_mjcf_to_urdf(mjcf_path: Path, urdf_path: Path) -> None:
    root = ET.parse(mjcf_path).getroot()
    if root.tag != "mujoco":
        raise ValueError(f"{mjcf_path} is not an MJCF file")
    compiler = root.find("compiler")
    degrees = compiler is not None and compiler.get("angle") == "degree"
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF has no worldbody")
    meshes = _mesh_files(root)
    effort = _actuator_effort(root)
    robot = ET.Element("robot", {"name": root.get("model", mjcf_path.stem)})
    for comment in _header_comments(root):
        robot.append(ET.Comment(comment))

    def walk(body: ET.Element, parent_name: str | None, parent_shift: tuple[float, float, float]) -> None:
        if parent_name is None:
            shift = (0.0, 0.0, 0.0)
            _append_link(robot, body, meshes, shift)
        else:
            shift = _append_joint(robot, body, parent_name, parent_shift, degrees, effort)
            _append_link(robot, body, meshes, shift)
        for child in body.findall("body"):
            walk(child, body.get("name", ""), shift)

    roots = worldbody.findall("body")
    if len(roots) != 1:
        raise ValueError(f"Expected one root body, found {len(roots)}")
    walk(roots[0], None, (0.0, 0.0, 0.0))
    ET.indent(robot, space="  ")
    urdf_path.write_text(ET.tostring(robot, encoding="unicode", xml_declaration=True) + "\n", encoding="utf-8")


def check_kinematics(mjcf_path: Path, urdf_path: Path, tolerance: float = 1e-5) -> None:
    """Compare body frames at the zero pose and at each joint's range midpoint."""
    import mujoco

    source = mujoco.MjModel.from_xml_path(str(mjcf_path))
    converted = mujoco.MjModel.from_xml_path(str(urdf_path))
    source_data = mujoco.MjData(source)
    converted_data = mujoco.MjData(converted)

    root_name = mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_BODY, 1)

    def body_id(model: mujoco.MjModel, name: str) -> int:
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        # MuJoCo's URDF importer welds the root link into the world body.
        if index < 0 and name == root_name:
            return 0
        if index < 0:
            raise ValueError(f"Body {name} missing after conversion")
        return index

    names = [mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_BODY, index) for index in range(1, source.nbody)]

    def max_error() -> float:
        mujoco.mj_forward(source, source_data)
        mujoco.mj_forward(converted, converted_data)
        error = 0.0
        for name in names:
            src = body_id(source, name)
            dst = body_id(converted, name)
            error = max(error, float(max(abs(source_data.xpos[src] - converted_data.xpos[dst]))))
            error = max(error, float(max(abs(source_data.xmat[src] - converted_data.xmat[dst]))))
        return error

    source_data.qpos[:] = 0
    converted_data.qpos[:] = 0
    worst = max_error()
    for joint_index in range(source.njnt):
        if source.jnt_type[joint_index] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        name = mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_JOINT, joint_index)
        destination = mujoco.mj_name2id(converted, mujoco.mjtObj.mjOBJ_JOINT, name)
        if destination < 0:
            raise ValueError(f"Joint {name} missing after conversion")
        lower, upper = source.jnt_range[joint_index]
        value = 0.5 * (lower + upper)
        source_data.qpos[:] = 0
        converted_data.qpos[:] = 0
        source_data.qpos[source.jnt_qposadr[joint_index]] = value
        converted_data.qpos[converted.jnt_qposadr[destination]] = value
        worst = max(worst, max_error())
    if worst > tolerance:
        raise SystemExit(f"Kinematic check failed: max frame error {worst:.3e} > {tolerance:.3e}")
    print(f"Kinematic check passed for {len(names)} bodies, max frame error {worst:.3e}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mjcf", type=Path, nargs="?", default=Path(__file__).with_name("yimu_v2_left.xml"))
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--check", action="store_true", help="Load both models in MuJoCo and compare body frames")
    args = parser.parse_args()
    output = args.output or args.mjcf.with_suffix(".urdf")
    convert_mjcf_to_urdf(args.mjcf, output)
    print(f"Wrote {output}")
    if args.check:
        check_kinematics(args.mjcf, output)


if __name__ == "__main__":
    main()

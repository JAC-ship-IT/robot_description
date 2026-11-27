import re
from pathlib import Path

from mjcf_urdf_simple_converter import convert

URDF = Path("/home/freedom/projects/dobot_l20_RL/assets/urdf/cross_hand/yimu_v2_left/yimu_v2_left.urdf")


def _rgba(color_code: str) -> str:
    return " ".join(f"{int(color_code[i:i + 2], 16) / 255:.6f}" for i in (0, 2, 4, 6))


def add_urdf_materials(urdf_path: Path) -> None:
    """Write a named URDF material for every converted mesh.

    Viewers drop ``<material name="">``. An OBJ ``usemtl`` line also replaces
    the URDF color with an unloaded MTL, so the mesh is left at the default.
    """
    text = urdf_path.read_text()
    pattern = re.compile(
        r'<mesh filename="meshes/converted_[^"]+_([0-9a-fA-F]{8})\.obj"/>\s*</geometry>'
        r'(?:\s*<material name="[^"]*">\s*<color rgba="[^"]*"/>\s*</material>)?'
    )

    def replace(match: re.Match) -> str:
        color_code = match.group(1).lower()
        return (
            f"{match.group(0).split('</geometry>')[0]}</geometry>\n"
            f'         <material name="color_{color_code}">\n'
            f'            <color rgba="{_rgba(color_code)}"/>\n'
            "         </material>"
        )

    updated, count = pattern.subn(replace, text)
    if count == 0:
        raise RuntimeError(f"No converted meshes found in {urdf_path}")
    urdf_path.write_text(updated)
    print(f"Set {count} named material tags in {urdf_path}")


def add_collision_meshes(urdf_path: Path) -> None:
    """Copy each visual mesh into a collision element.

    MuJoCo's URDF importer ignores a link that only has a visual mesh, so the
    material color never reaches the viewer. A matching collision mesh keeps
    the geom and uses the visual color, which is how yimu_right.urdf is loaded.
    """
    pattern = re.compile(
        r'<visual name="(?P<name>[^"]*)">\s*'
        r'(?P<origin><origin xyz="[^"]*" rpy="[^"]*"/>)\s*'
        r'<geometry>\s*'
        r'(?P<mesh><mesh filename="[^"]+"/>)\s*'
        r'</geometry>\s*'
        r'<material name="(?P<mat>[^"]*)">\s*'
        r'<color rgba="(?P<rgba>[^"]*)"/>\s*'
        r'</material>\s*'
        r'</visual>'
        r'(?:\s*<collision>\s*<origin xyz="[^"]*" rpy="[^"]*"/>\s*'
        r'<geometry>\s*<mesh filename="[^"]+"/>\s*</geometry>\s*</collision>)?'
    )

    def replace(match: re.Match) -> str:
        return (
            f'<visual name="{match.group("name")}">\n'
            f'         {match.group("origin")}\n'
            "         <geometry>\n"
            f'            {match.group("mesh")}\n'
            "         </geometry>\n"
            f'         <material name="{match.group("mat")}">\n'
            f'            <color rgba="{match.group("rgba")}"/>\n'
            "         </material>\n"
            "      </visual>\n"
            "      <collision>\n"
            f'         {match.group("origin")}\n'
            "         <geometry>\n"
            f'            {match.group("mesh")}\n'
            "         </geometry>\n"
            "      </collision>"
        )

    text = urdf_path.read_text()
    updated, count = pattern.subn(replace, text)
    if count == 0:
        raise RuntimeError(f"No visual meshes found in {urdf_path}")
    urdf_path.write_text(updated)
    print(f"Added {count} collision meshes to {urdf_path}")


def bake_mesh_colors(urdf_path: Path) -> None:
    """Remove MTL references and store the same color on OBJ vertices."""
    meshes_dir = urdf_path.parent / "meshes"
    count = 0
    for obj_path in sorted(meshes_dir.glob("converted_*.obj")):
        color_code = obj_path.stem.rsplit("_", 1)[-1]
        if len(color_code) != 8:
            continue
        rgb = " ".join(f"{int(color_code[i:i + 2], 16) / 255:.6f}" for i in (0, 2, 4))
        output = []
        for line in obj_path.read_text().splitlines():
            if line.startswith(("mtllib ", "usemtl ")):
                continue
            if line.startswith("v "):
                x, y, z = line.split()[1:4]
                line = f"v {x} {y} {z} {rgb}"
            output.append(line)
        obj_path.write_text("\n".join(output) + "\n")
        count += 1
    print(f"Baked vertex colors into {count} OBJ files")


convert(
    "/home/freedom/projects/dobot_l20_RL/assets/urdf/cross_hand/yimu_v2_left/yimu_v2_left.xml",
    str(URDF),
)
add_urdf_materials(URDF)
add_collision_meshes(URDF)
bake_mesh_colors(URDF)

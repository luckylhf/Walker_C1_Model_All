"""Build the C1 visual-only neck sleeve in this directory from the no-hand model."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import networkx as nx
import numpy as np
import trimesh
from PIL import Image

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent
SOURCE = ROOT / "mujoco/walker_c1.xml"
PITCH = "head_pitch_link"
TORSO = "waist_roll_link"
AROUND = 128
# An oblique cut runs below the jaw in front and higher up the nape in back.
CUT_NORMAL = np.array([0.62, 0.0, 1.0])
CUT_POINT = np.array([0.0, 0.0, 0.007])


def section_points(segments, values, level):
    a, b = values.T
    mask = ((a <= level) & (b > level)) | ((b <= level) & (a > level))
    lines = segments[mask]
    factor = (level - a[mask]) / (b[mask] - a[mask])
    return lines[:, 0] + (lines[:, 1] - lines[:, 0]) * factor[:, None]


def meridian(head, theta):
    radial = np.array([np.cos(theta), np.sin(theta), 0.0])
    normal = [-radial[1], radial[0], 0.0]
    lines = trimesh.intersections.mesh_plane(head, normal, [0, 0, 0])
    sample = lines.reshape(-1, 3)
    sample = sample[sample @ radial > 0]
    hem = sample[sample[:, 2].argmin()]
    points = []
    levels = (lines - CUT_POINT) @ CUT_NORMAL
    start = (hem - CUT_POINT) @ CUT_NORMAL
    for t in np.linspace(0, 1, 23):
        level = start * (1 - t) + 0.006 * t
        # At the open ends use a small positive level offset to avoid a tangent.
        cross = section_points(lines, levels, max(start + 0.00005, level))
        outward = cross[cross @ radial > 0]
        point = outward[np.argmax(outward @ radial)].copy() if len(outward) else hem.copy()
        # Tuck the last six millimetres into the head. The silhouette at the
        # cut comes straight from the source rather than from a fitted ellipse.
        point -= radial * max(0.0, level) * 0.35
        # The original rigid neck was tight to the yaw bracket. Allow room for
        # the loss of cross-sectional area during large-angle linear skinning.
        rolloff = np.sin(np.pi * t)
        side_allowance = 0.001 + 0.006 * np.sin(theta) ** 2
        point += radial * side_allowance * rolloff ** 1.5
        # Narrow the exposed sleeve most strongly across the shoulders. Fade
        # the adjustment into the original chest opening and the under-jaw fit.
        taper = smoothstep(t / 0.34) * (1 - smoothstep((t - 0.54) / 0.28))
        point[:2] *= 1 - taper * np.array([0.065, 0.15])
        points.append(point)
    return np.array(points), hem


def smoothstep(t):
    t = np.clip(t, 0, 1)
    return t * t * (3 - 2 * t)


def write_weave():
    # An understated, seamless two-directional weave; the UVs follow the cloth.
    y, x = np.mgrid[0:128, 0:128] * (2 * np.pi / 16)
    warp = np.cos(x + 0.45 * np.sin(y / 2))
    weft = np.cos(y - 0.45 * np.sin(x / 2))
    weave = 235 + 7 * warp + 6 * weft + 3 * warp * weft
    Image.fromarray(weave.astype(np.uint8)).convert("RGB").save(OUT / "weave.png")


def write_skin(path, points, uv, faces, weights, transforms):
    with path.open("wb") as out:
        out.write(struct.pack("<4i", len(points), len(uv), len(faces), len(weights)))
        out.write(points.astype("<f4").tobytes())
        out.write(uv.astype("<f4").tobytes())
        out.write(faces.astype("<i4").tobytes())
        for name, row in weights.items():
            body_name, offset = name if isinstance(name, tuple) else (name, (0, 0, 0))
            ids = np.flatnonzero(row != 0)
            body = transforms.body(body_name)
            bind_position = body.xpos + offset
            out.write(struct.pack("<40s7fi", body_name.encode(), *bind_position, *body.xquat, len(ids)))
            out.write(ids.astype("<i4").tobytes())
            out.write(row[ids].astype("<f4").tobytes())


def outer_cut_loop(mesh):
    boundary = mesh.edges_unique[np.bincount(mesh.edges_unique_inverse) == 1]
    loops = [np.array(list(loop)) for loop in nx.connected_components(nx.from_edgelist(boundary))]
    if len(loops) != 2:
        raise ValueError("Expected inner and outer loops on the cut fabric.")
    return max(loops, key=lambda ids: np.linalg.norm(mesh.vertices[ids, :2], axis=1).mean())


def source_cuff(hood, angles, hem):
    """Clip a fixed 10 mm band from the original CAD facets, including their lip."""
    v = hood.vertices
    theta = (np.arctan2(v[:, 1], v[:, 0]) - np.pi) % (2*np.pi)
    z0 = np.interp(theta, angles, hem[:, 2], period=2*np.pi)
    height = v[:, 2] - z0 - 0.010
    points = list(v)
    faces = []
    cuts = {}
    for tri in hood.faces:
        if np.all(height[tri] > 0):
            continue
        polygon = []
        for a, b in zip(tri, np.roll(tri, -1)):
            if height[a] <= 0:
                polygon.append(a)
            if (height[a] <= 0) != (height[b] <= 0):
                edge = tuple(sorted((a, b)))
                if edge not in cuts:
                    t = height[a] / (height[a] - height[b])
                    cuts[edge] = len(points)
                    points.append(v[a] + t * (v[b] - v[a]))
                polygon.append(cuts[edge])
        faces.extend((polygon[0], polygon[i], polygon[i + 1]) for i in range(1, len(polygon)-1))
    cuff = trimesh.Trimesh(vertices=points, faces=faces, process=False)
    cuff.remove_unreferenced_vertices()
    return cuff


def bridge_rings(points, low, high):
    """Join two ordered star-shaped rings without changing either perimeter."""
    def ordered(ids):
        ang = (np.arctan2(points[ids, 1], points[ids, 0]) - np.pi) % (2*np.pi)
        order = np.argsort(ang)
        return ids[order], ang[order]
    low, la = ordered(low)
    high, ha = ordered(high)
    i = j = -1
    faces = []
    while i+1 < len(low) or j+1 < len(high):
        lower, upper = low[i], high[j]
        next_low = la[i+1] if i+1 < len(low) else np.inf
        next_high = ha[j+1] if j+1 < len(high) else np.inf
        if next_low <= next_high:
            i += 1
            faces.append((lower, low[i], upper))
        else:
            j += 1
            faces.append((lower, high[j], upper))
    return faces


def crown_and_neck(hood):
    vertices, faces, _ = trimesh.intersections.slice_faces_plane(
        hood.vertices, hood.faces, CUT_NORMAL, CUT_POINT)
    crown = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    angles = np.arange(AROUND) * (2 * np.pi / AROUND)
    sections = [meridian(hood, a + np.pi) for a in angles]
    grid = np.stack([s[0] for s in sections], axis=1)
    hem = np.array([s[1] for s in sections])
    cuff = source_cuff(hood, angles, hem)

    # Ring four is 13 to 21 mm above the hem. Its predecessor is the outer
    # cut loop of the source cuff. Blend into the narrow neck above this join.
    grid = grid[4:-4]
    rows = len(grid)
    crown_offset = rows * AROUND
    cuff_offset = crown_offset + len(crown.vertices)
    points = np.vstack((grid.reshape(-1, 3), crown.vertices, cuff.vertices))
    faces = list(crown.faces + crown_offset) + list(cuff.faces + cuff_offset)
    for k in range(rows - 1):
        for j in range(AROUND):
            left = k * AROUND + j
            right = k * AROUND + (j + 1) % AROUND
            faces.extend([(left, right, left + AROUND), (right, right + AROUND, left + AROUND)])
    ring = np.arange(AROUND)
    faces.extend(bridge_rings(points, (rows-1)*AROUND + ring, outer_cut_loop(crown) + crown_offset))
    faces.extend(bridge_rings(points, outer_cut_loop(cuff) + cuff_offset, ring))

    # Turn the fabric inward behind the source lip. This recessed facing screens
    # the yaw bracket where the original CAD has small gaps beside the shoulders.
    inward = hem[:, :2] / np.linalg.norm(hem[:, :2], axis=1, keepdims=True)
    inner_top, inner_bottom = hem.copy(), hem.copy()
    inner_top[:, :2] -= 0.009 * inward
    inner_top[:, 2] += 0.005
    inner_bottom[:, :2] -= 0.005 * inward
    inner_bottom[:, 2] -= 0.014
    facing_start = len(points)
    points = np.vstack((points, inner_bottom, inner_top))
    faces.extend(bridge_rings(points, facing_start + ring, facing_start + AROUND + ring))

    # Preserve the previously fitted upper and middle neck weights. The cuff
    # and first ring stay on the chest even at the yaw/pitch combination limits.
    t = np.repeat((np.arange(rows) + 6) / 24, AROUND)
    t[:AROUND] = 0
    t = np.r_[t, np.ones(len(crown.vertices)), np.zeros(len(cuff.vertices) + 2*AROUND)]
    turn = smoothstep((t - 0.16) / 0.65)
    pitch = turn * smoothstep((t - 0.25) / 0.59)
    # A pair of offset bind frames straightens the stretched side as pitch changes.
    # Their translations cancel at neutral and pure yaw. At pitch q their net
    # displacement in the yaw frame is length * [sin(q), 0, cos(q)-1].
    longitude = np.arctan2(points[:, 1], points[:, 0])
    back = np.maximum(-np.cos(longitude), 0)
    front = np.maximum(np.cos(longitude), 0)
    envelope = smoothstep((t - 0.17) / 0.34) * (1 - smoothstep((t - 0.65) / 0.27))
    correction = (0.020 * back ** 2 + 0.010 * front ** 2) * envelope / 0.05
    yaw_name = "head_yaw_link"
    # Let the throat follow yaw earlier, keeping it clear of the enclosed yaw
    # bracket without adding static material to the front or rear profile.
    turn = np.minimum(1.0, turn + 0.45 * front ** 2 * envelope)
    weights = {TORSO: 1-turn, yaw_name: turn-pitch-correction,
               PITCH: pitch-correction, (PITCH, (0, 0, -0.05)): correction,
               (yaw_name, (0, 0, 0.05)): correction}
    return points, np.asarray(faces), weights


def texture_coordinates(points, faces, weights):
    # Align fine knit rows around the head. Split UVs along the hidden back seam.
    angles = (np.arctan2(points[:, 1], points[:, 0]) - np.pi) % (2*np.pi)
    u = angles / (2*np.pi)
    uv = np.column_stack((u * 24, points[:, 2] / 0.016))
    wrapped = np.ptp(u[faces], axis=1) > 0.5
    corners = faces[wrapped].copy()
    low = u[corners] < 0.5
    ids, reverse = np.unique(corners[low], return_inverse=True)
    corners[low] = len(points) + reverse
    faces[wrapped] = corners
    return (np.vstack([points, points[ids]]), np.vstack([uv, uv[ids] + [24, 0]]),
            faces, {name: np.r_[w, w[ids]] for name, w in weights.items()})


def build():
    model = mujoco.MjModel.from_xml_path(str(SOURCE))
    pose = mujoco.MjData(model)
    mujoco.mj_forward(model, pose)
    original = trimesh.load(ROOT / "meshes/body/head_pitch_link.STL")
    parts = original.split(only_watertight=False, repair=False)
    crossing = [p for p in parts if np.min((p.vertices - CUT_POINT) @ CUT_NORMAL) < 0]
    if len(crossing) != 1:
        raise ValueError("Source head changed. Locate the rigid neck again before trimming.")
    hood = crossing[0]
    # The large continuous CAD piece follows the fabric/visor interface all
    # the way around the back of the skull. Preserve the rigid facial inserts.
    plastic = trimesh.util.concatenate([part for part in parts if part is not hood])
    plastic.export(OUT / "head_shell.stl")
    points, faces, weights = crown_and_neck(hood)
    points, uv, faces, weights = texture_coordinates(points, faces, weights)
    write_skin(OUT / "neck.skn", points + pose.body(PITCH).xpos, uv, faces, weights, pose)
    write_weave()

    tree = ET.parse(SOURCE)
    root = tree.getroot()
    geom = root.find(f".//geom[@name='{PITCH}_visual_0']")
    mesh = root.find(f"./asset/mesh[@name='{geom.get('mesh')}']")
    geom.attrib.pop("rgba", None)
    geom.set("material", "face_plastic")
    ET.SubElement(root.find("asset"), "material", name="face_plastic",
                  rgba="0.075 0.079 0.085 1", specular="0.60", shininess="0.55")
    mesh.set("file", (OUT / "head_shell.stl").relative_to(ROOT).as_posix())
    root.set("model", "walker_c1_neck_sleeve_preview")
    ET.SubElement(root.find("asset"), "texture", name="neck_weave", type="2d",
                  file="weave.png")
    ET.SubElement(root.find("asset"), "material", name="neck_fabric",
                  texture="neck_weave",
                  rgba="0.79 0.78 0.76 1", specular="0.02", shininess="0.02")
    deformable = ET.SubElement(root, "deformable")
    ET.SubElement(deformable, "skin", name="neck_sleeve",
                  file=(OUT / "neck.skn").relative_to(ROOT).as_posix(), material="neck_fabric")
    ET.indent(tree, space="  ")
    tree.write(OUT / "walker_c1_neck.xml", encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    build()

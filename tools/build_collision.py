#!/usr/bin/env python3
"""
Build the SO-101 description URDF used by MoveIt and Gazebo.

The CAD-exported URDF reuses the full-detail visual meshes as collision
geometry: 17 <collision> elements totalling roughly 300k triangles. MoveIt's
self-collision sampling, and every planning query afterwards, does
mesh-against-mesh checks whose cost scales with that triangle count.

This script rewrites collision geometry only. For each link it merges that
link's collision meshes into one point cloud (applying each mesh's own origin
first), takes the convex hull, and emits a single <collision> at identity
origin pointing at the hull STL. A hull is a few hundred triangles and is
tested with GJK rather than a BVH walk.

<visual> is left exactly as it was, apart from the package:// rewrite.

Links in KEEP_CONCAVE skip the hull step: their shape is a hook, and hulling
them would fill the gripper opening so the jaws read as permanently touching.
Their meshes are still merged, so every link ends up with one collision tag.
"""

import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull

SRC_URDF = "/work/models/so101/so101_new_calib.urdf"
SRC_ASSETS = "/work/models/so101/assets"
PKG_DIR = "/work/moveit_ws/src/so101_description"
PKG_NAME = "so101_description"

# Concave links whose convex hull would be a bad approximation.
KEEP_CONCAVE = set()


# ---------------------------------------------------------------- STL I/O

def load_stl(path):
    """Return an (N, 3, 3) array of triangles. Handles binary and ASCII STL."""
    raw = open(path, "rb").read()

    # Binary STL is 84 bytes of header plus 50 bytes per facet. ASCII files
    # start with "solid", but so do some binary ones, so check the length.
    if len(raw) >= 84:
        n = int(np.frombuffer(raw, dtype="<u4", count=1, offset=80)[0])
        if len(raw) == 84 + n * 50:
            facets = np.frombuffer(raw, dtype=np.uint8, count=n * 50,
                                   offset=84).reshape(n, 50)
            # Bytes 12..48 of each facet are the three vertices; 0..12 is the
            # normal, which we recompute rather than trust.
            verts = facets[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
            return verts.astype(np.float64)

    # ASCII fallback: every "vertex x y z" line, in order.
    coords = [list(map(float, line.split()[1:4]))
              for line in raw.decode("utf-8", "replace").splitlines()
              if line.strip().startswith("vertex")]
    return np.asarray(coords, dtype=np.float64).reshape(-1, 3, 3)


def save_stl(path, tris):
    """Write an (N, 3, 3) triangle array as a binary STL."""
    n = len(tris)
    # Right-hand-rule normal from the vertex winding.
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals),
                        where=lengths > 0)

    facets = np.zeros((n, 50), dtype=np.uint8)
    block = np.concatenate([normals[:, None, :], tris], axis=1)
    # 12 float32 per facet (normal + 3 vertices) = 48 bytes, then a 2-byte
    # attribute word that stays zero.
    facets[:, :48] = block.astype("<f4").reshape(n, 12).view(np.uint8)

    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(np.array([n], dtype="<u4").tobytes())
        fh.write(facets.tobytes())


# ------------------------------------------------------------- geometry

def rpy_to_matrix(roll, pitch, yaw):
    """URDF fixed-axis RPY, i.e. Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def read_origin(elem):
    """Return (R, t) for a URDF element's <origin>, identity if absent."""
    origin = elem.find("origin")
    if origin is None:
        return np.eye(3), np.zeros(3)
    xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    return rpy_to_matrix(*rpy), xyz


def hull_triangles(points):
    """Convex hull of a point cloud, as outward-facing triangles."""
    hull = ConvexHull(points)
    tris = []
    # ConvexHull does not guarantee consistent winding, so orient each facet
    # against the outward normal that Qhull reports in `equations`.
    for simplex, equation in zip(hull.simplices, hull.equations):
        a, b, c = points[simplex]
        if np.dot(np.cross(b - a, c - a), equation[:3]) < 0:
            b, c = c, b
        tris.append([a, b, c])
    return np.asarray(tris)


# ------------------------------------------------------------------ main

def main():
    if not os.path.isdir(PKG_DIR):
        sys.exit(f"package not found: {PKG_DIR} (create it first)")

    mesh_dir = os.path.join(PKG_DIR, "meshes")
    coll_dir = os.path.join(mesh_dir, "collision")
    urdf_dir = os.path.join(PKG_DIR, "urdf")
    for d in (coll_dir, urdf_dir):
        os.makedirs(d, exist_ok=True)

    # Visual meshes are copied across untouched.
    for name in os.listdir(SRC_ASSETS):
        if name.lower().endswith(".stl"):
            shutil.copyfile(os.path.join(SRC_ASSETS, name),
                            os.path.join(mesh_dir, name))

    tree = ET.parse(SRC_URDF)
    root = tree.getroot()

    for link in root.findall("link"):
        name = link.get("name")
        collisions = link.findall("collision")
        if not collisions:
            continue

        # Gather every collision mesh of this link in link-frame coordinates.
        chunks = []
        for coll in collisions:
            mesh = coll.find("geometry/mesh")
            if mesh is None:
                print(f"  {name}: non-mesh collision left alone")
                chunks = []
                break
            path = os.path.join(os.path.dirname(SRC_URDF),
                                mesh.get("filename"))
            tris = load_stl(path)
            rot, trans = read_origin(coll)
            # Bake the element's own origin into the vertices, so every chunk
            # ends up in link-frame coordinates and they can simply be stacked.
            chunks.append((tris.reshape(-1, 3) @ rot.T + trans).reshape(-1, 3, 3))
        if not chunks:
            continue

        merged = np.concatenate(chunks)
        if name in KEEP_CONCAVE:
            out, how = merged, "merged (concave, no hull)"
        else:
            out, how = hull_triangles(merged.reshape(-1, 3)), "hull"

        out_name = f"{name}.stl"
        save_stl(os.path.join(coll_dir, out_name), out)
        print(f"  {name:30s} {len(collisions)} mesh -> "
              f"{len(out):6d} tris  [{how}]")

        # Replace the link's collision elements with a single one. The mesh is
        # already in link-frame coordinates, so its origin is identity.
        for coll in collisions:
            link.remove(coll)
        new = ET.SubElement(link, "collision")
        ET.SubElement(new, "origin", xyz="0 0 0", rpy="0 0 0")
        geom = ET.SubElement(new, "geometry")
        ET.SubElement(geom, "mesh",
                      filename=f"package://{PKG_NAME}/meshes/collision/{out_name}")

    # Visual meshes: relative path -> package:// so ROS can resolve them.
    for mesh in root.findall(".//visual/geometry/mesh"):
        fn = mesh.get("filename")
        if fn.startswith("assets/"):
            mesh.set("filename",
                     f"package://{PKG_NAME}/meshes/{os.path.basename(fn)}")

    root.set("name", "so101")
    ET.indent(tree, space="  ")
    out_urdf = os.path.join(urdf_dir, "so101.urdf")
    tree.write(out_urdf, encoding="utf-8", xml_declaration=True)
    print(f"\nwrote {out_urdf}")


if __name__ == "__main__":
    main()

import argparse
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mitsuba as mi
import numpy as np
import pybullet as pb
import sionna_utils.utils
import trimesh
from sionna.rt import Camera, Scene as SionnaScene
from sionna.rt.scene_object import SceneObject
from sionna.rt.radio_materials import ITURadioMaterial

_OBJ_PLY = {".obj", ".ply"}

# Keyword hints for guessing ITU material from group names
_ITU_KEYWORDS = {
    "concrete": ["concrete", "wall", "column", "pillar", "slab"],
    "brick": ["brick"],
    "plasterboard": ["plaster", "drywall", "gypsum"],
    "wood": ["wood", "timber", "shelf", "table", "chair"],
    "glass": ["glass", "window", "glazing"],
    "metal": ["metal", "steel", "aluminum", "iron", "door", "frame"],
    "ceiling_board": ["ceiling"],
    "floorboard": ["floor", "parquet"],
    "marble": ["marble", "tile", "stone"],
    "very_dry_ground": ["ground", "soil", "earth", "terrain"],
}


def _suggest_itu(name):
    n = name.lower()
    for itu_type, keywords in _ITU_KEYWORDS.items():
        if any(kw in n for kw in keywords):
            return itu_type
    return None


def _load_trimesh(file_path):
    """Load file once; return (merged Trimesh, raw loaded object for material groups)."""
    raw = trimesh.load(str(file_path))
    if isinstance(raw, trimesh.Scene):
        merged = trimesh.util.concatenate(raw.dump())
    else:
        merged = raw
    return merged, raw


def _load_mesh(file_path):
    mesh, _ = _load_trimesh(file_path)
    return np.array(mesh.vertices, dtype=np.float32), np.array(
        mesh.faces, dtype=np.int32
    )


def _report_mesh(file_path, mesh, raw):
    """Print mesh statistics and return list of issue strings (empty = clean)."""
    W = 54
    sep = "─" * W

    lo, hi = mesh.vertices.min(axis=0), mesh.vertices.max(axis=0)
    dims = hi - lo

    def row(label, value, flag=""):
        return f"  {label:<22}{value!s:>18}  {flag}"

    print(f"\n{'═' * W}")
    print(f"  {file_path.name}")
    print(sep)

    # Geometry counts
    print(row("Vertices", f"{len(mesh.vertices):,}"))
    print(row("Faces", f"{len(mesh.faces):,}"))
    print(sep)

    # Scale / AABB
    max_dim = float(dims.max())
    if max_dim < 0.01:
        scale_flag, scale_note = "⚠", f"very small — mm?"
    elif max_dim > 1000:
        scale_flag, scale_note = "⚠", f"very large — mm?"
    else:
        scale_flag, scale_note = "✓", "looks OK"
    print(row("AABB X (m)", f"{dims[0]:.4f}"))
    print(row("AABB Y (m)", f"{dims[1]:.4f}"))
    print(row("AABB Z (m)", f"{dims[2]:.4f}"))
    print(row("Scale", scale_note, scale_flag))
    print(sep)

    issues = []
    if scale_flag == "⚠":
        issues.append(scale_note)

    # Topology checks
    flag = "✓" if mesh.is_watertight else "⚠"
    print(row("Watertight", "yes" if mesh.is_watertight else "NO", flag))
    if not mesh.is_watertight:
        issues.append("not watertight")

    flag = "✓" if mesh.is_winding_consistent else "⚠"
    print(
        row(
            "Normals",
            "consistent" if mesh.is_winding_consistent else "inconsistent",
            flag,
        )
    )
    if not mesh.is_winding_consistent:
        issues.append("inconsistent normals")

    degen = int(np.sum(mesh.area_faces < 1e-10))
    flag = "✓" if degen == 0 else "⚠"
    print(row("Degenerate faces", f"{degen:,}", flag))
    if degen:
        issues.append(f"{degen} degenerate faces")

    v = np.ascontiguousarray(mesh.vertices, dtype=np.float32)
    v_void = v.view(np.dtype((np.void, v.dtype.itemsize * v.shape[1])))
    n_dupes = len(v) - len(np.unique(v_void))
    flag = "✓" if n_dupes == 0 else "⚠"
    print(row("Duplicate verts", f"{n_dupes:,}", flag))
    if n_dupes:
        issues.append(f"{n_dupes} duplicate verts")

    print(sep)

    # Material groups — use already-loaded raw object, no second file read
    if isinstance(raw, trimesh.Scene) and len(raw.geometry) > 1:
        groups = list(raw.geometry.keys())
        print(row("Material groups", len(groups)))
        unmatched = []
        for g in groups:
            suggestion = _suggest_itu(g)
            if suggestion:
                print(f"    {g:<28} → {suggestion}  ✓")
            else:
                print(f"    {g:<28} → no ITU match  ⚠")
                unmatched.append(g)
        if unmatched:
            issues.append(f"{len(unmatched)} material group(s) without ITU match")
    else:
        print(row("Material groups", 1, "✓"))

    print(f"{'═' * W}")
    if issues:
        print(f"  ⚠  {len(issues)} issue(s): {', '.join(issues)}")
    else:
        print(f"  ✓  All checks passed")
    print(f"{'═' * W}\n")

    return issues


def _add_bbox_reference(scene, verts):
    """Add a 1 m³ reference cube beside the object's base corner."""
    lo = verts.min(axis=0)
    cube = trimesh.creation.box(extents=[1.0, 1.0, 1.0])
    # Place it just outside the -X, -Y corner at the base
    cube.apply_translation([lo[0] - 1.5, lo[1], lo[2] + 0.5])
    cv = np.array(cube.vertices, dtype=np.float32)
    cf = np.array(cube.faces, dtype=np.int32)
    mat = ITURadioMaterial("scale-ref", itu_type="metal", thickness=0.01)
    so = SceneObject(
        mi_mesh=_make_mi_mesh("scale_ref_1m", cv, cf),
        name="scale_ref_1m",
        radio_material=mat,
    )
    scene.edit(add=so)


def _make_mi_mesh(name, verts, faces):
    mesh = mi.Mesh(
        name,
        vertex_count=verts.shape[0],
        face_count=faces.shape[0],
        has_vertex_normals=False,
        has_vertex_texcoords=False,
    )
    params = mi.traverse(mesh)
    params["vertex_positions"] = mi.TensorXf(verts.ravel())
    params["faces"] = mi.TensorXu(faces.ravel())
    params.update()
    return mesh


def _camera_views(verts):
    """Return (name, Camera) pairs covering outside, close, and inside viewpoints."""
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    center = (lo + hi) * 0.5
    extent = float(np.linalg.norm(hi - lo))
    cx, cy, cz = center.tolist()
    far = extent * 1.5
    close = extent * 0.5

    return [
        (
            "front",
            Camera(position=[cx, cy - far, cz + far * 0.3], look_at=center.tolist()),
        ),
        (
            "front_close",
            Camera(
                position=[cx, cy - close, cz + close * 0.3], look_at=center.tolist()
            ),
        ),
        (
            "side",
            Camera(position=[cx + far, cy, cz + far * 0.3], look_at=center.tolist()),
        ),
        ("top", Camera(position=[cx, cy, cz + far], look_at=center.tolist())),
        (
            "iso",
            Camera(
                position=[cx + far * 0.7, cy - far * 0.7, cz + far * 0.7],
                look_at=center.tolist(),
            ),
        ),
        ("inside", Camera(position=center.tolist(), look_at=[cx, cy + 1.0, cz])),
    ]


def test_sionna_compat(file_path, out_dir):
    tri_mesh, raw = _load_trimesh(file_path)
    issues = _report_mesh(file_path, tri_mesh, raw)
    verts = np.array(tri_mesh.vertices, dtype=np.float32)
    faces = np.array(tri_mesh.faces, dtype=np.int32)

    material = ITURadioMaterial("concrete-wall", itu_type="metal", thickness=0.02)
    if file_path.suffix.lower() in _OBJ_PLY:
        so = SceneObject(
            fname=str(file_path), name=file_path.stem, radio_material=material
        )
    else:
        so = SceneObject(
            mi_mesh=_make_mi_mesh(file_path.stem, verts, faces),
            name=file_path.stem,
            radio_material=material,
        )

    extent = float(np.linalg.norm(verts.max(axis=0) - verts.min(axis=0)))
    scene = SionnaScene()
    sionna_utils.geometry.create_coordinate_frame(
        scene, position=[0, 0, 0], scale=extent * 1.2
    )
    scene.edit(add=so)
    _add_bbox_reference(scene, verts)
    print(f"[sionna] OK — '{so.name}' in scene, objects: {list(scene.objects.keys())}")

    scene.scene_geometry_updated()

    lo, hi = verts.min(axis=0), verts.max(axis=0)
    dims = hi - lo
    dim_label = f"{dims[0]:.3f} × {dims[1]:.3f} × {dims[2]:.3f} m"
    status_label = f"⚠ {len(issues)} issue(s)" if issues else "✓ clean"

    views = _camera_views(verts)
    renders = []
    for view_name, cam in views:
        render_out = out_dir / f"{file_path.stem}_{view_name}.exr"
        bitmap = scene.render_to_file(
            filename=str(render_out),
            camera=cam,
            lighting_scale=2.0,
            resolution=(1280, 960),
            clip_at=3,
        )
        img = np.clip(np.array(bitmap) ** (1 / 2.2), 0, 1)
        renders.append((view_name, img))
        print(f"[sionna] render saved → {render_out}")

    ncols = 3
    nrows = (len(renders) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(ncols * 4, nrows * 9))
    axes = np.array(axes).reshape(nrows, ncols)
    for i, (label, img) in enumerate(renders):
        ax = axes[i // ncols, i % ncols]
        ax.imshow(img)
        ax.set_title(label.replace("_", " "), fontsize=13, pad=6)
        ax.axis("off")
    for j in range(len(renders), nrows * ncols):
        axes[j // ncols, j % ncols].set_visible(False)
    fig.suptitle(
        f"{file_path.stem}   |   {dim_label}   |   {status_label}",
        fontsize=13,
        y=1.01,
    )
    fig.tight_layout()
    fig_out = out_dir / f"{file_path.stem}_views.png"
    fig.savefig(fig_out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[sionna] figure saved  → {fig_out}")

    html_out = out_dir / f"{file_path.stem}.html"
    sionna_utils.utils.scene_export_html(
        scene, html_out, show_orientations=True, clip_at=2, resolution=(1400, 1000)
    )
    print(f"[sionna] preview saved → {html_out}")


def test_pybullet_compat(file_path):
    verts, faces = _load_mesh(file_path)
    center = (verts.min(axis=0) + verts.max(axis=0)) * 0.5
    verts -= center

    cid = pb.connect(pb.DIRECT)
    col = pb.createCollisionShape(
        pb.GEOM_MESH,
        vertices=verts.tolist(),
        indices=faces.flatten().tolist(),
        flags=pb.GEOM_FORCE_CONCAVE_TRIMESH,
        physicsClientId=cid,
    )
    body = pb.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        basePosition=[0, 0, 0],
        baseOrientation=[0, 0, 0, 1],
        useMaximalCoordinates=True,
        physicsClientId=cid,
    )
    pb.changeDynamics(body, -1, contactProcessingThreshold=0, physicsClientId=cid)
    pb.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=cid)
    print(f"[pybullet] OK — body_id={body}, col_id={col}")
    pb.disconnect(cid)


def main():
    parser = argparse.ArgumentParser(
        description="Test Sionna and PyBullet mesh compatibility"
    )
    parser.add_argument(
        "file",
        type=pathlib.Path,
        help="Path to a mesh file (.obj, .ply, .stl, .glb, …)",
    )
    args = parser.parse_args()

    out_dir = pathlib.Path("out")
    out_dir.mkdir(exist_ok=True)

    test_sionna_compat(args.file, out_dir)
    test_pybullet_compat(args.file)


if __name__ == "__main__":
    main()

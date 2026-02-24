Scene 3D Compatibility Test
===========================

Just a tool to check the 3D files compatibility with the scenes.

<div style="display: flex; justify-content: center; gap: 10px;">
  <img src="./assets/conference_room_orbit.gif" width="45%" />
  <img src="./assets/human_orbit.gif" width="45%" />
</div>


Requirements
------------

* Python 3.12+
* uv

Installation
------------

```
git clone https://github.com/mlouielu/scene-3d-compat-test
cd scene-3d-compat-test
uv sync
```

Run compatibility test
----------------------

### Enable venv

```
source .venv/bin/activate
```

### Run compatibility test

```
$ python compat_test.py assets/human.obj
pybullet build time: Jan 29 2025 23:17:20

══════════════════════════════════════════════════════
  human.obj
──────────────────────────────────────────────────────
  Vertices                          10,475
  Faces                             20,908
──────────────────────────────────────────────────────
  AABB X (m)                        1.8375
  AABB Y (m)                        0.4116
  AABB Z (m)                        1.7780
  Scale                           looks OK  ✓
──────────────────────────────────────────────────────
  Watertight                            NO  ⚠
  Normals                       consistent  ✓
  Degenerate faces                       0  ✓
  Duplicate verts                        0  ✓
──────────────────────────────────────────────────────
  Material groups                        1  ✓
══════════════════════════════════════════════════════
  ⚠  1 issue(s): not watertight
══════════════════════════════════════════════════════

[sionna] OK — 'human' in scene, objects: ['_coord_origin', '_coord_arrow_x', '_coord_arrow_y', '_coord_arrow_z', 'human', 'scale_ref_1m']
[sionna] render saved → out/human_front.exr
[sionna] render saved → out/human_front_close.exr
[sionna] render saved → out/human_side.exr
[sionna] render saved → out/human_top.exr
[sionna] render saved → out/human_iso.exr
[sionna] render saved → out/human_inside.exr
[sionna] figure saved  → out/human_views.png
[sionna] preview saved → out/human.html
[pybullet] OK — body_id=0, col_id=0
```

### Error

```
$ python compat_test.py room.obj
pybullet build time: Jan 29 2025 23:17:20

══════════════════════════════════════════════════════
room.obj
──────────────────────────────────────────────────────
...

  Vertices                       5,151,303
  Faces                          1,741,330
──────────────────────────────────────────────────────
  AABB X (m)                       10.0000
  AABB Y (m)                       13.1992
  AABB Z (m)                        5.2067
  Scale                           looks OK  ✓
──────────────────────────────────────────────────────
  Watertight                            NO  ⚠
  Normals                       consistent  ✓
  Degenerate faces                     240  ⚠
  Duplicate verts                4,251,081  ⚠
──────────────────────────────────────────────────────
  Material groups                        1  ✓
══════════════════════════════════════════════════════
  ⚠  3 issue(s): not watertight, 240 degenerate faces, 4251081 duplicate verts
══════════════════════════════════════════════════════

[sionna] OK — 'conference_room' in scene, objects: ['_coord_origin', '_coord_arrow_x', '_coord_arrow_y', '_coord_arrow_z', 'conference_room', 'scale_ref_1m']
[sionna] render saved → out/conference_room_front.exr
[sionna] render saved → out/conference_room_front_close.exr
[sionna] render saved → out/conference_room_side.exr
[sionna] render saved → out/conference_room_top.exr
[sionna] render saved → out/conference_room_iso.exr
[sionna] render saved → out/conference_room_inside.exr
[sionna] figure saved  → out/conference_room_views.png
[sionna] preview saved → out/conference_room.html
b3Warning[examples/SharedMemory/../Importers/ImportURDFDemo/UrdfFindMeshFile.h,21]:
: invalid mesh filename './'
b3Warning[examples/SharedMemory/PhysicsDirect.cpp,1066]:
createCollisionShape failedTraceback (most recent call last):
  File "/mnt/e2tb/unc/mmwave/scene-3d-compat-test/compat_test.py", line 349, in <module>
    main()
  File "/mnt/e2tb/unc/mmwave/scene-3d-compat-test/compat_test.py", line 345, in main
    test_pybullet_compat(args.file)
  File "/mnt/e2tb/unc/mmwave/scene-3d-compat-test/compat_test.py", line 309, in test_pybullet_compat
    col = pb.createCollisionShape(
          ^^^^^^^^^^^^^^^^^^^^^^^^
pybullet.error: createCollisionShape failed.
```

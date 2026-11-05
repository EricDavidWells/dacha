# Wrist mount

Two-part passive marker mount for 1.5" webbing (arm, hand, torso).

- `wrist_base`: webbing runs under it (skin side), up through a slot at each end, over the end bar. Dovetail tongue on top. Print flat.
- `plate_grid`: curved plate (arc about Y) with a grid of M4 holes normal to the surface. Slides onto the tongue, locked with one M4x16 screw + nut in the base bottom. Print on one flat Y-end face, no supports.

Markers: M4 male stud -> M4 male-female hex standoff (6 mm male thread) -> nut in the hex pocket on the plate inside. Stack standoffs for height.

Parameters are at the top of `wrist_mount.py`.

```bash
pip install cadquery trimesh
python wrist_mount.py   # writes wrist_base.{step,stl}, plate_grid.{step,stl}
python make_viewer.py   # writes viewer.html (three.js, STLs embedded)
```

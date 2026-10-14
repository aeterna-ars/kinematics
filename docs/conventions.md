# Coordinate conventions

- Coordinates are right-handed; all angles are in radians.
- A link extends along its joint's local **+X** axis.
- Orientations compose as `parent * base_rotation * rotation`.
  `base_rotation` defines the joint's rest frame relative to its parent;
  `rotation` is the adjustable rotation within that frame.
- `base_position` and IK targets use world coordinates. Lengths, target positions,
  and tolerance must use the same distance units.
- Quaternions store `(x, y, z, w)`. Valid nonunit inputs are normalized when used.

## Joint limits

| Constraint | Allowed local rotation |
| --- | --- |
| `None` | Any rotation |
| `Hinge` | Rotation about the rest frame's X, Y, or Z axis; `-π <= min_rad <= max_rad <= π` |
| `Ball` | Swing within `max_cone_rad` of local +X, with unrestricted twist around +X; `0 <= max_cone_rad <= π` |

Equal hinge limits fix a joint at that angle. A zero ball cone still permits
twist. `apply_to_rotation` projects a quaternion into these limits; its caller
must supply valid limits. The scalar `apply`/`apply_to_angle` helpers only clamp
angles and do not perform quaternion projection.

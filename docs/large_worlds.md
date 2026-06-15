# Large Worlds (Double Precision) {#large-worlds}

Box3D can be built with double precision world positions for large worlds: simulations that range
far from the origin, where a single precision float can no longer resolve a position. At a
coordinate of 1e7 meters a float has a step of about one meter, so bodies snap to a coarse grid and
contacts jitter. Double precision keeps full sub-millimeter resolution out to planetary distances.

Only the world position boundary is double. Velocities, forces, shapes, contact manifolds, the
contact solver, and the broad-phase tree all stay float. This is the same boundary design Jolt
Physics uses for its big-world mode: doubles carry the absolute position, everything inside one
body's frame stays float, and the per-step motion the solver integrates is small and meters-scale
where float precision is ample. The cost is a few percent, not the 2x of an all-double build.

Double precision is **off by default**. With it off every double-precision type collapses to its
float counterpart through a typedef and the boundary helpers reduce to plain float operations, so a
float build behaves exactly as Box3D always has, with no measurable cost.

This implementation is inspired by [Jolt](https://jrouwe.github.io/JoltPhysics/index.html#big-worlds).

## Enabling it

Set the CMake option:

```cmake
set(BOX3D_DOUBLE_PRECISION ON)
```

The define is propagated to consumers as a `PUBLIC` compile definition, so anything that links Box3D
through CMake sees the same precision mode in the headers and cannot mismatch. For non-CMake
consumers there is a link-time guard: a float application linked against a double-precision library
(or the reverse) fails to link on the first Box3D call rather than miscompiling silently. At runtime
`b3IsDoublePrecision()` reports which mode the library was built in, for bindings and diagnostics.

## The two world-position types

Double precision adds two types and leaves the existing math types alone:

- `b3Position` — a world position. Three doubles in large world mode, an alias for `b3Vec3`
  otherwise.
- `b3WorldTransform` — a world transform: a `b3Position` translation and a float `b3Quat` rotation.
  An alias for `b3Transform` otherwise.

`b3Vec3`, `b3Quat`, `b3Transform`, and `b3AABB` stay float in both modes. `b3Transform` remains the
type for local and relative frames; `b3WorldTransform` is used for world space. Rotations are float
quaternions in both modes, so the trig and the cross-platform determinism properties are unchanged.

The public API uses these types wherever it accepts or returns a world position: `b3BodyDef.position`,
`b3Body_GetPosition` / `b3Body_GetTransform`, `b3Body_SetTransform`, `b3Body_GetWorldPoint` /
`b3Body_GetLocalPoint`, `b3Body_GetWorldCenterOfMass`, the explosion and ray-cast origins, contact
and ray-cast result points, and the body move event. With double precision off these are all the
float types they have always been, so existing code compiles unchanged.

With double precision **on**, `b3Position` and `b3Vec3` are distinct structs by design. Code that
passed a `b3Vec3` where a world position is now required no longer compiles, which is the intended
cost: enabling large world mode is a deliberate source migration, and the compiler points at every
site that needs a conversion. Helpers cover the boundary:

```c
b3Position p = b3MakePosition( v );      // float vector -> world position
b3Vec3     v = b3ToVec3( p );            // world position -> float vector (lossy far from origin)
b3Vec3     d = b3PositionDelta( a, b );  // a - b, demoted to float (the precision boundary)
b3Position q = b3OffsetPosition( p, d ); // p + d
float      x = b3RoundDownFloat( p.x );  // conservative narrowing, pair with b3RoundUpFloat to
                                         // build a float box that always contains double bounds
```

## Operating range

Full simulation correctness holds everywhere `b3Position` can represent. Body integration, the
contact solver, joints, and continuous collision all run in float relative to each body's own
moving frame, so a stack settles and a bullet is caught the same way at 1e7 as at the origin.

The practical limit comes from the broad phase, which stays float and stores conservative
(outward-rounded) float bounds. Far from the origin the float bound quantization grows: about one
meter at 1e7, sixteen meters at 1e8. Overlapping shapes always still produce a pair, so correctness
is preserved, but beyond roughly 1e7 to 1e8 meters the broad phase reports extra false pairs and
loses some margin hysteresis, which costs performance. Stay within about ±1e7 to ±1e8 meters.

## Queries far from the origin

The simulation is precise far from the origin; the spatial query helpers are not yet. They are a
documented float carve-out in this first release: each one takes its world input, demotes it to
float, and runs in float world space, so it loses sub-meter resolution beyond about 1e7 meters.

- `b3World_CastRay` / `b3World_CastRayClosest` take a `b3Position` origin (so callers pass world
  positions naturally) but traverse the tree and intersect shapes in float. A cast that grazes a
  shape by less than a coordinate ULP far from the origin can miss it. Hit points come back as
  `b3Position`.
- `b3World_OverlapShape`, `b3World_CastShape`, `b3World_CastMover`, `b3World_CollideMover`, and
  `b3Shape_RayCast` operate in float world space.
- `b3World_Explode` resolves the per-shape impulse in float around the explosion position.

This is fine near the origin and degrades gracefully far from it. The character controller
(`b3World_CastMover` / `b3World_CollideMover`) is the case to watch: a mover driven at 1e7 gets
float world coordinates and silently loses subpixel motion, so a future origin-relative query API is
the real fix for distant characters.

## Debug drawing

`b3DebugDraw` carries a `drawOrigin` world position. `b3World_Draw` demotes every shape transform and
contact point to float relative to it, so a host that keeps `drawOrigin` near the camera draws a
distant scene crisply instead of snapping it to the coarse float grid around the absolute origin. It
defaults to zero, which reproduces absolute coordinates and leaves a near-origin scene unchanged. The
sample app sets it from the active scene, and the Large World sample uses it to render a stack at 1e7
with no jitter.

## Determinism

Both precision modes are internally deterministic and reproduce across worker counts. The numerics
differ between modes — double precision accumulates body positions in double, so a body settles and
sleeps on a slightly different step and the state hash differs — so a double-precision build is not
bit-identical to a float build. The `DeterminismTest` carries a separate set of expected values for
each mode.

## SIMD

Double precision is orthogonal to SIMD. The wide types used in the broad phase and mesh collision
stay 4-wide float, and the contact solver is untouched in both modes. There is no double-precision
SIMD path and none is needed: the hot interior never sees an absolute world coordinate.

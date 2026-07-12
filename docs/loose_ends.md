# Loose Ends

## User Data

Bodies, shapes, and joints allow you to attach user data as a `void*`. This is
useful when you receive a body, shape, or joint id from an event or query and
need to map it back to a game object.

A common pattern is storing a game-entity pointer on the body:

```c
GameEntity* entity = GameCreateEntity();
b3BodyDef bodyDef = b3DefaultBodyDef();
bodyDef.userData = entity;
entity->bodyId = b3CreateBody(myWorldId, &bodyDef);
```

The circular reference lets you go from entity to body and back. Some typical
uses:

- Applying damage to an entity from a contact event.
- Triggering a scripted event when a body enters a region.
- Cleaning up game state when a joint is destroyed.

Keep the type consistent. If one body stores a `GameEntity*`, all bodies should.
Mixing pointer types and casting without a discriminant leads to crashes.

Setters and getters:

```c
// Body
b3Body_SetUserData(bodyId, ptr);
void* ptr = b3Body_GetUserData(bodyId);

// Shape
b3Shape_SetUserData(shapeId, ptr);
void* ptr = b3Shape_GetUserData(shapeId);

// Joint
b3Joint_SetUserData(jointId, ptr);
void* ptr = b3Joint_GetUserData(jointId);

// World
b3World_SetUserData(worldId, ptr);
void* ptr = b3World_GetUserData(worldId);
```

## Coordinate Systems

Box3D uses a right-handed coordinate system. Positive Y is up by default,
meaning the default gravity vector points in the negative Y direction
(`{0, -9.8f, 0}`). Nothing in the engine hard-codes this — gravity is just a
`b3Vec3` set on the world and you can orient it however your application
requires.

Use MKS units: meters, kilograms, seconds, and radians. The solver is tuned for
objects in the range of roughly 0.1 to 10 meters. Very small or very large
objects relative to this range can degrade numerical stability. If your content
is authored at a different unit scale, apply a single conversion factor at the
boundary between your asset pipeline and Box3D, not scattered throughout the
simulation code.

Bodies in Box3D have 6 degrees of freedom. Position is a `b3Vec3`, orientation
is a `b3Quat`, angular velocity and torque are `b3Vec3` values. The rotational
inertia tensor is a `b3Matrix3`.

## Debug Drawing

Implement the function pointers in `b3DebugDraw` to get detailed drawing of the
Box3D world. The struct lives in `types.h` and has slots for:

- `DrawShapeFcn` — draws a shape; receives a user-shape object created by
  `b3WorldDef::createDebugShape` and the current transform and color.
- `DrawSegmentFcn` — draws a line segment.
- `DrawTransformFcn` — draws a coordinate frame.
- `DrawPointFcn` — draws a point.
- `DrawBoundsFcn` — draws an AABB as a wireframe box.
- `DrawBoxFcn` — draws an oriented box by extents and transform.
- `DrawStringFcn` — draws a world-space label.

Category flags on the struct control what gets drawn:

| Flag | What it shows |
|---|---|
| `drawShapes` | Shape geometry |
| `drawJoints` | Joint frames and constraints |
| `drawJointExtras` | Extra joint information (limits, motors) |
| `drawBounds` | Shape AABBs |
| `drawMass` | Center-of-mass marker and mass value for dynamic bodies |
| `drawBodyNames` | Body name strings |
| `drawContacts` | Contact points and anchors |
| `drawContactNormals` | Contact normal directions |
| `drawContactForces` | Normal impulse magnitudes |
| `drawGraphColors` | Constraint-graph color assignment |
| `drawContactFeatures` | Raw contact feature indices |
| `drawIslands` | Island bounding boxes |

Call the draw function after stepping the world:

```c
b3DebugDraw draw = b3DefaultDebugDraw();
draw.DrawShapeFcn  = MyDrawShape;
draw.DrawSegmentFcn = MyDrawSegment;
// ... fill remaining callbacks ...
draw.drawShapes = true;
draw.drawingBounds = myViewAABB;

b3World_Draw(worldId, &draw, UINT64_MAX);
```

The `maskBits` argument is tested against each shape's `categoryBits` in the
broad-phase tree. Only shapes where `(maskBits & shape->categoryBits) != 0` are
visited. Pass `UINT64_MAX` to draw everything, or use a subset of your category
bits to limit drawing to specific layers (e.g. skip debris for a clean overhead
view).

The shape draw path uses a two-callback model set on `b3WorldDef`:
`createDebugShape` is called once when a shape is first drawn, giving you a
chance to upload GPU geometry; `destroyDebugShape` is called when the shape is
modified or destroyed so you can release it. The `DrawShapeFcn` callback
receives the opaque handle returned by `createDebugShape`, which enables
efficient multi-pass rendering without re-uploading geometry every frame.

The samples application demonstrates a complete `b3DebugDraw` implementation.

## Limitations

Box3D uses several approximations to simulate rigid body physics efficiently.
As a v0.1 engine it is still maturing, so expect rougher edges than Box2D.

Current limitations:

1. Extreme mass ratios between connected bodies can cause joint stretching and
   contact overlap.
2. Soft constraints improve robustness but allow a small amount of flexing in
   joints and contacts.
3. Continuous collision handles fast-moving dynamic bodies against static
   geometry (and bullets against dynamic bodies), but not general
   dynamic-versus-dynamic continuous collision.
4. Continuous collision does not propagate through joints, so fast-moving
   articulated chains may momentarily stretch.
5. The integrator is semi-implicit Euler, which gives first-order accuracy.
   Projectile arcs are approximate. Increasing `subStepCount` in
   `b3World_Step` improves accuracy at a proportional cost.
6. Constraint resolution uses an iterative Gauss-Seidel solver for real-time
   performance. Collisions are not perfectly rigid and contacts are not
   pixel-accurate. More sub-steps tighten the result.
7. Mesh and height-field shapes are static-only. They do not participate in
   dynamic-versus-dynamic contact.
8. The compound shape type is static-body only and immutable after creation.
9. The character mover API is experimental.

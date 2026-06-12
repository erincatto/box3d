# Hello Box3D {#hello}
The Box3D distribution includes a Hello World unit test written in C. The test
creates a large static ground box and a small dynamic box. This code does not
contain any graphics. All you will see is text output in the console showing
the box's position over time.

This is a good example of how to get up and running with Box3D.

## Creating a World
Every Box3D program begins with the creation of a world object.
The world is the physics hub that manages memory, objects, and simulation.
The world is represented by an opaque handle called `b3WorldId`.

It is easy to create a Box3D world. First, create the world definition:

```c
b3WorldDef worldDef = b3DefaultWorldDef();
```

The world definition is a temporary object you can create on the stack. The function
`b3DefaultWorldDef()` populates the world definition with default values. This is necessary
because C does not have constructors and zero-initializing `b3WorldDef` is not appropriate.

Box3D has no built-in concept of *up*. The gravity vector is a `b3Vec3` and can point in
any direction. Convention in Box3D examples uses +Y as the up axis. The default gravity
is already `{0, -10, 0}`, but it can be set explicitly:

```c
worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
```

Now create the world:

```c
b3WorldId worldId = b3CreateWorld(&worldDef);
```

World creation copies all the data it needs from the definition, so the definition can
go out of scope immediately afterward.

## Creating a Ground Box
Bodies are built using the following steps:
1. Define a body with position, type, etc.
2. Use the world id to create the body.
3. Build a hull shape with the desired extents.
4. Create the shape on the body.

For step 1, create the ground body definition and set its initial position:

```c
b3BodyDef groundBodyDef = b3DefaultBodyDef();
groundBodyDef.position = (b3Vec3){ 0.0f, -10.0f, 0.0f };
```

For step 2, use the world id to create the ground body. Bodies are static by default,
meaning they have zero mass, never move, and do not collide with other static bodies.

```c
b3BodyId groundId = b3CreateBody(worldId, &groundBodyDef);
```

Notice that `worldId` is passed by value. Ids are small structures and are always passed
by value.

For steps 3 and 4, build a box hull and attach it. Box3D uses convex hulls for box
shapes. The `b3MakeBoxHull` helper takes three **half-extents** (hx, hy, hz), so the
ground slab below is 100 units wide in X, 20 units tall in Y, and 100 units deep in Z:

```c
b3BoxHull groundBox = b3MakeBoxHull(50.0f, 10.0f, 50.0f);

b3ShapeDef groundShapeDef = b3DefaultShapeDef();
b3CreateHullShape(groundId, &groundShapeDef, &groundBox.base);
```

The `.base` field holds the `b3HullData` that `b3CreateHullShape` expects. Box3D copies
the hull data into a shared internal database, so `groundBox` does not need to outlive
the call. Do not call `b3DestroyHull` on a `b3BoxHull`; it is stack-allocated.

Box3D is tuned for meters, kilograms, and seconds, so the extents above are in meters.
The engine works best when objects are sized like real-world objects (a barrel is roughly
1 m tall). Simulating glaciers or dust particles would push the limits of single-precision
floating point.

Every shape must have a parent body, even static shapes. You can attach multiple shapes
to one body. A shape's world transform is inherited from its parent body; there is no
independent shape transform.

## Creating a Dynamic Body
Creating a dynamic body follows the same steps. The key difference is setting the body
type to `b3_dynamicBody` and giving the shape a non-zero density.

```c
b3BodyDef bodyDef = b3DefaultBodyDef();
bodyDef.type = b3_dynamicBody;
bodyDef.position = (b3Vec3){ 0.0f, 4.0f, 0.0f };
b3BodyId bodyId = b3CreateBody(worldId, &bodyDef);
```

> **Caution**:
> You must set the body type to `b3_dynamicBody` if you want the body to
> move in response to forces such as gravity.

Create a unit cube hull and a shape definition with density and friction:

```c
b3BoxHull dynamicBox = b3MakeCubeHull(1.0f);

b3ShapeDef shapeDef = b3DefaultShapeDef();
shapeDef.density = 1.0f;
shapeDef.baseMaterial.friction = 0.3f;

b3CreateHullShape(bodyId, &shapeDef, &dynamicBox.base);
```

`b3MakeCubeHull(r)` is a convenience that produces a cube with half-extent `r` on all
three axes, equivalent to `b3MakeBoxHull(r, r, r)`.

> **Caution**:
> A dynamic body should have at least one shape with a non-zero density.
> Otherwise you will get unexpected behavior.

That completes initialization. We are now ready to simulate.

## Simulating the World
Box3D uses a numerical integrator that advances the simulation by discrete time steps.
A fixed time step of 1/60 seconds (60 Hz) is recommended for most games. Avoid tying
the time step to your frame rate; a variable time step produces variable results that
are hard to debug.

```c
float timeStep = 1.0f / 60.0f;
```

In addition to integration, Box3D uses a constraint solver. Box3D advances through the
time step in several *sub-steps*, giving each constraint multiple chances to react. Four
sub-steps is the suggested value:

```c
int subStepCount = 4;
```

At 60 Hz with 4 sub-steps the constraints run at 240 Hz internally. More sub-steps
improve accuracy at the cost of performance.

The simulation loop calls `b3World_Step` once per game tick:

```c
for (int i = 0; i < 90; ++i)
{
    b3World_Step(worldId, timeStep, subStepCount);

    b3Vec3 position = b3Body_GetPosition(bodyId);
    b3Quat rotation = b3Body_GetRotation(bodyId);

    printf("%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\n",
           position.x, position.y, position.z,
           rotation.v.x, rotation.v.y, rotation.v.z, rotation.s);
}
```

`b3Body_GetPosition` returns a `b3Vec3` with the body origin in world space.
`b3Body_GetRotation` returns a `b3Quat` — a unit quaternion stored as a vector part
`q.v` (x, y, z) and a scalar part `q.s`. There is no single angle to extract as there
was in 2D; orientation in 3D requires the full quaternion. To convert to an axis-angle
representation use `b3GetAxisAngle`:

```c
float angle;
b3Vec3 axis = b3GetAxisAngle(&angle, rotation);
```

`angle` is in radians and `axis` is the unit rotation axis. Use
`b3MakeMatrixFromQuat` when you need a 3×3 rotation matrix, for example to feed a
renderer.

The output should show the box falling from y = 4 and coming to rest on the ground at
approximately y = 1 (the box half-height sits at y = 0 + 1 after the ground surface
at y = 0):

```
0.00 4.00 0.00 ...
0.00 3.99 0.00 ...
0.00 3.98 0.00 ...
...
0.00 1.25 0.00 ...
0.00 1.13 0.00 ...
0.00 1.01 0.00 ...
```

For advice on managing a fixed simulation rate alongside a variable render rate, see
[Fix Your Timestep!](https://gafferongames.com/post/fix_your_timestep/).

## Multithreading (optional)
By default Box3D runs single-threaded. The `b3DefaultWorldDef` leaves `workerCount`
at 1 and the task callbacks null, which is fine for getting started. When performance
matters, Box3D can drive a task system. Supply `workerCount` plus `enqueueTask`,
`finishTask`, and `userTaskContext` on the world definition before calling
`b3CreateWorld`. See the Foundations page for details.

## Cleanup
When you are done with the simulation, destroy the world:

```c
b3DestroyWorld(worldId);
```

This efficiently destroys all bodies, shapes, and joints in the simulation.

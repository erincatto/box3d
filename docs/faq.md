# FAQ

## What is Box3D?
Box3D is a feature rich 3D rigid body physics engine, written in C17 by Erin Catto. It has been used in games and game engines
as a 3D counterpart to the well-established Box2D engine.

Box3D uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License) and can be used free of charge. Credit
should be included if possible. Support is [appreciated](https://github.com/sponsors/erincatto).

## What platforms does Box3D support?
Box3D is developed using C17. It is portable and can be compiled for any platform with a conforming C17 compiler.

Erin Catto maintains the C version. Community ports and bindings for other languages are not officially supported.

## Who makes it?
Erin Catto is the creator and primary author of Box3D. It is an open source project, and accepts community feedback via
[GitHub Issues](https://github.com/erincatto/box3d/issues) and
[GitHub Discussions](https://github.com/erincatto/box3d/discussions).

## How do I get help?
You should read the documentation and the rest of this FAQ first. Also, you should study the examples included in the source
distribution. Then you can visit the [Discord](https://discord.gg/NKYgCBP) to ask any remaining questions.

Please do not message or email Erin Catto directly for support. It is best to ask questions on the Discord server so that
everyone can benefit from the discussion.

## Documentation

### Why isn't a feature documented?
If you grab the latest code from the git main branch you will likely find features that are not documented in the manual.
New features are added to the manual after they are mature and a new point release is imminent. However, all major features
added to Box3D are accompanied by example code in the samples application to test the feature and show the intended usage.

## Prerequisites

### Programming
You should have a working knowledge of C before you use Box3D. You should understand functions, structures, and pointers.
There are plenty of resources on the web for learning C. You should also understand your development environment:
compilation, linking, and debugging.

### Math and Physics
You should have a basic knowledge of rigid bodies, force, torque, and impulses. In 3D you will also encounter quaternions
for orientation and 3x3 inertia tensors for rotational dynamics. If you come across a math or physics concept you don't
understand, please read about it on Wikipedia. Visit this [page](https://box2d.org/publications/) if you want a deeper
knowledge of the algorithms used in Box3D.

## API

### What units does Box3D use?
Box3D is tuned for meters-kilograms-seconds (MKS). This is recommended as the unit system for your game. However, you may
use different units if you are careful. Call `b3SetLengthUnitsPerMeter()` at startup to change the length unit.

### What coordinate system does Box3D use?
Box3D has no built-in notion of up. The gravity vector in `b3WorldDef` can point in any direction. The default is
`(0, -10, 0)` (negative Y is down) but you can set it to whatever suits your application.

### Why don't you use this awesome language?
Box3D is designed to be portable and easy to wrap with other languages, so I decided to use C17. I used C17 to get
support for atomics.

### Can I use Box3D in a DLL?
Yes. See the CMake option `BUILD_SHARED_LIBS`.

### Is Box3D thread-safe?
No. Box3D will likely never be fully thread-safe from the outside. Box3D has a large API and trying to make such an API
thread-safe would have a large performance and complexity impact. However, you can call read-only functions from multiple
threads. For example, all the spatial query functions are read-only.

Box3D does use multithreading internally during `b3World_Step`. You supply your own task system via the `enqueueTask` and
`finishTask` callbacks in `b3WorldDef`.

## Build Issues

### Why doesn't my code compile and/or link?
There are many reasons why a build can go bad. Here are a few that have come up:
* Using old Box3D headers with new code
* Not linking the Box3D library with your application
* Using old project files that don't include some new source files

## Rendering

### What are Box3D's rendering capabilities?
Box3D is only a physics engine. How you draw stuff is up to you.

### But the samples application draws stuff
Visualization is very important for debugging collision and physics. The samples application helps test Box3D and gives
you examples of how to use Box3D. The samples are not part of the Box3D library.

### How do I draw shapes?
Fill out a `b3DebugDraw` struct with your drawing callbacks and call `b3World_Draw(worldId, &draw, maskBits)`.
The mask bits let you filter which shape categories are drawn.

## Accuracy
Box3D uses approximate methods for a few reasons.
* Performance
* Some differential equations don't have known solutions
* Some constraints cannot be determined uniquely

What this means is that constraints are not perfectly rigid and sometimes you will see some bounce even when the restitution
is zero. Box3D uses [Gauss-Seidel](https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method) to approximately solve
constraints. Box3D also uses [Semi-implicit Euler](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method) to
approximately solve the differential equations. Box3D also does not have exact collision between dynamic shapes. Slow
moving shapes may have small overlap for a few time steps. In extreme stacking scenarios, shapes may have sustained
overlap.

## Making Games

### Tile / Voxel Based Environments
Using many boxes for terrain may not work well because box-like characters can get snagged on internal corners. Box3D
provides capsules and convex hulls that may work better for characters. Consider the character mover API
(`b3World_CastMover`, `b3World_CollideMover`) for smooth first/third person movement.

### Asteroid Type Coordinate Systems
Box3D does not have any support for coordinate frame wrapping. You would likely need to customize Box3D for this purpose.

## Determinism

### Is Box3D deterministic?
For the same input Box3D will reproduce any simulation. Box3D does not use any random numbers nor base any computation
on random events (such as timers, etc).

Box3D is also deterministic under multithreading. A simulation using two threads will give the same result as eight threads.

Box3D inherits cross-platform determinism from its design: floating-point contraction is disabled (`-ffp-contract=off`)
and IEEE 754 arithmetic is relied upon consistently.

However, Box3D does not have rollback determinism. There is no mechanism to set a world back to a prior state and then
resume simulation expecting identical results. Box3D caches a lot of internal state to improve simulation stability and
performance.

### But I really want determinism
This naturally leads to the question of fixed-point math. Box3D does not support fixed-point math. Fixed-point math is
slower and more tedious to develop, and I have chosen not to use it.

## What are the common mistakes made by new users?
* Using non-metric units instead of meters
* Expecting Box3D to give pixel-perfect results
* Testing their code in release mode (always use Debug for testing — it enables assertions and validation)
* Not learning C before using Box3D
* Confusing b2 (Box2D) and b3 (Box3D) symbols when reading documentation

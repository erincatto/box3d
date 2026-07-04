# Foundations

Box3D provides minimal base functionality for allocation hooks and vector math. The C interface
allows most runtime data and types to be defined internally in the `src` folder.

## Assertions

Box3D will assert on bad input. This includes things like sending in NaN or infinity for values.
It will assert if you use negative values for things that should only be positive, such as density.

Box3D will also assert if an internal bug is detected. For this reason, it is advisable to build
Box3D from source. The library compiles in about a second.

You may wish to capture assertions in your application. In that case use `b3SetAssertFcn()`. This
lets you override the debugger break and/or perform your own error handling.

## Allocation

Box3D uses memory efficiently and minimizes per-frame allocations by pooling memory. The engine
quickly adapts to the simulation size. After the first step or two of simulation there should be
no further per-frame allocations.

As bodies, shapes, and joints are created and destroyed, their memory is recycled. Internally all
this data is stored in contiguous arrays. When an object is destroyed, the array element is marked
empty. When an object is created it fills an empty slot via an efficient free list.

Once the internal memory pools are initially filled, the only allocations should be for sleeping
islands, since their data is copied out of the main simulation. Those allocations are generally
infrequent.

You can provide a custom allocator using `b3SetAllocator()` and query the total bytes currently
allocated using `b3GetByteCount()`.

## Version

The `b3Version` structure holds the current version so you can query it at run-time using
`b3GetVersion()`.

```c
b3Version version = b3GetVersion();
printf("Box3D version %d.%d.%d\n", version.major, version.minor, version.revision);
```

## Vector Math
Box3D includes a vector math library covering types `b3Vec3`, `b3Quat`, `b3Transform`,
`b3Matrix3`, and `b3AABB`. The library is designed to suit the internal needs of Box3D
and its interface. All members are exposed, so you can use them freely in your application.

### b3Vec3
Three-component float vector with fields `x`, `y`, `z`. Useful constants and operations:

```c
b3Vec3 a = {1.0f, 0.0f, 0.0f};    // inline init
b3Vec3 z = b3Vec3_zero;            // {0,0,0}
b3Vec3 c = b3Add(a, b);            // component-wise add
b3Vec3 d = b3Sub(a, b);            // subtract
b3Vec3 e = b3MulSV(2.0f, a);       // scalar * vector
float  f = b3Dot(a, b);            // dot product
b3Vec3 g = b3Cross(a, b);          // cross product
float  h = b3Length(a);            // Euclidean length
b3Vec3 n = b3Normalize(a);         // unit vector
b3Vec3 p = b3Perp(a);              // any perpendicular unit vector
b3Vec3 q = b3Lerp(a, b, 0.5f);     // linear interpolation
```

### b3Quat
Unit quaternion representing orientation. Stored as a vector part `q.v` (x, y, z) and a
scalar part `q.s`. The identity quaternion is `b3Quat_identity`.

Useful operations:

```c
// Construct from axis (must be unit) and angle in radians
b3Quat q = b3MakeQuatFromAxisAngle(axis, radians);

// Rotate a vector
b3Vec3 r = b3RotateVector(q, v);

// Inverse-rotate a vector (equivalent to rotating by the conjugate)
b3Vec3 s = b3InvRotateVector(q, v);

// Compose two rotations: apply q2 first, then q1
b3Quat qc = b3MulQuat(q1, q2);

// Conjugate (same as inverse for a unit quaternion)
b3Quat qi = b3Conjugate(q);

// Extract axis-angle
float angle;
b3Vec3 axis = b3GetAxisAngle(&angle, q);

// Total rotation angle (ignoring axis)
float totalAngle = b3GetQuatAngle(q);

// Convert to rotation matrix
b3Matrix3 m = b3MakeMatrixFromQuat(q);

// Normalized linear interpolation
b3Quat qi = b3NLerp(q1, q2, alpha);
```

Because orientation in 3D is three-dimensional, there is no single scalar angle as there was
in 2D. Always work with the full quaternion or the derived matrix.

### b3Transform
A rigid transform: a position vector `t.p` (`b3Vec3`) combined with an orientation `t.q`
(`b3Quat`). The identity transform is `b3Transform_identity`.

```c
// Apply transform to a point in the transform's local frame -> world frame
b3Vec3 world = b3TransformPoint(t, localPoint);

// Inverse: world frame -> local frame
b3Vec3 local = b3InvTransformPoint(t, worldPoint);

// Compose: t_child expressed in t_parent's frame
b3Transform combined = b3MulTransforms(t_parent, t_child);

// Relative transform: t_b expressed in t_a's frame
b3Transform rel = b3InvMulTransforms(t_a, t_b);

// Invert a transform
b3Transform inv = b3InvertTransform(t);
```

### b3Matrix3

3×3 matrix stored as three column vectors `cx`, `cy`, `cz`. Primarily used for inertia
tensors and rotation matrices. Useful operations include `b3MulMV` (matrix-vector multiply),
`b3MulMM` (matrix-matrix multiply), `b3Transpose`, `b3InvertMatrix`, and `b3MakeMatrixFromQuat`.

### b3AABB

Axis-aligned bounding box with `lowerBound` and `upperBound` as `b3Vec3`. Helpers include
`b3AABB_Overlaps`, `b3AABB_Contains`, `b3AABB_ContainsPoint`, `b3AABB_Union`,
`b3AABB_Center`, `b3AABB_Extents`, `b3AABB_Inflate`, and `b3AABB_Transform`.

## Multithreading {#multi}

Box3D has been optimized for multithreading. Multithreading is not required and by default Box3D will run single-threaded. If performance is important for your application, you should consider using the multithreading features.

### Internal scheduler

Box3D has a built-in task scheduler that creates threads. You can use the built-in scheduler by setting the worker count in the world definition. This example shows how to use 4 workers. In this case Box3D will create 3 threads, and count the thread that calls `b3World_Step` as the fourth worker. I recommend to use the core count of the CPU as the worker count, not counting hyper-threads or efficiency cores.

```c
b3WorldDef worldDef = b3DefaultWorldDef();
worldDef.workerCount = 4;
```

### External scheduler

You can optionally connect your own task scheduler if you want more control.  Multithreading is established for each Box3D world you create and must be hooked up to the world definition. See `b3TaskCallback()`, `b3EnqueueTaskCallback()`, and `b3FinishTaskCallback()` for more details. Also see `b3WorldDef::workerCount`, `b3WorldDef::enqueueTask`, and `b3WorldDef::finishTask`.

```c
// Implement b3EnqueueTaskCallback
void* MyEnqueueTask(b3TaskCallback* task, void* taskContext,
                    void* userContext, const char* taskName)
{
    MyTask* t = AllocTask();
    t->task        = task;
    t->taskContext = taskContext;
    Scheduler* myScheduler = (Scheduler*)userContact;
    SubmitToThreadPool(myScheduler, t);
    return t;
}

// Implement b3FinishTaskCallback
void MyFinishTask(void* userTask, void* userContext)
{
    MyTask* t = (MyTask*)userTask;
    WaitForCompletion(t);
    FreeTask(t);
}

b3WorldDef worldDef = b3DefaultWorldDef();
worldDef.enqueueTask = MyEnqueueTask;
worldDef.finishTask = MyFinishTask;
worldDef.userTaskContext = myScheduler;
worldDef.workerCount = GetMyWorkerCount();
```

### Threading model

The multithreading design for Box3D is focused on [data parallelism](https://en.wikipedia.org/wiki/Data_parallelism).
The goal is to use multiple cores to finish the world simulation as fast as possible.
Box3D multithreading is not designed for [task parallelism](https://en.wikipedia.org/wiki/Task_parallelism).
Often in games you have a render thread or an audio thread doing work in isolation from the
main thread. Those are examples of task parallelism.

So when you design your game loop, you should let Box3D *go wide* and use multiple cores to
finish its work quickly, without other threads interacting with the Box3D world at the same time.

It is expected that the thread that calls `b3World_Step` participates in making progress. Do not call
`b3World_Step` and park that fiber. Tasks will only be enqueued on the thread that calls `b3World_Step`.
`MyFinishTask` must block until the task has completed. Ideally your scheduler is helping make progress
when `MyFinishTask` is called.

### Avoiding race conditions

In a multithreaded environment you must be careful to avoid [race conditions](https://en.wikipedia.org/wiki/Race_condition).
Modifying the world while it is simulating will lead to unpredictable behavior and is never safe.
It is also not safe to read data from a Box3D world while it is simulating. Box3D may move data
structures to improve cache performance, so you could easily read garbage.

> **Caution**:
> Do not perform read or write operations on a Box3D world during `b3World_Step()`.
> Do not write to the Box3D world from multiple threads. Any operation that wakes
> a body is not thread-safe.

It *is safe* to do ray-casts, shape-casts, and overlap tests from multiple threads outside of
`b3World_Step()`. Generally any read-only operation is safe to do multithreaded outside of
`b3World_Step()`. This can be very useful if you have multithreaded game logic.

## Multithreading Multiple Worlds

Some applications may wish to create multiple Box3D worlds and simulate them on different threads.
This works fine because Box3D has very limited use of globals.

There are a few caveats:

- You will get a race condition if you create or destroy Box3D worlds from multiple threads. Use a mutex to guard those operations.
- If you simulate multiple Box3D worlds simultaneously, they should probably not share a task system. Otherwise you risk preemption between worlds competing for the same workers.
- Any callbacks you hook up to Box3D must be thread-safe, including memory allocators.
- All the limitations for single-world simulation still apply.

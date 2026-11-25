# Known Limitations {#known-limitations}

Box3D makes trade-offs for performance and stability. The behaviors on this page are known and
intentional. They are not bugs, so please don't report them as issues. Each section describes the
symptom, why it happens, and what to do about it.

## Contact recycling drifts on slowly rotating platforms

Dynamic bodies stacked on a large rotating platforms can drift. One example is crates on the deck
of a ship. You can prevent this by disabling recycling on the platform.

```c
platformBodyDef.enableContactRecycling = false;
```

See also: [Contact Recycling](simulation.md#contact-recycling).

## Conflicting constraints

Avoid creating conflicting constraints. For example, the thighs of a ragdoll can collide with each
other in most setups because they are not connected to each other with a joint that would disable
the collision. If the thigh collision overlaps close to the pivot then this will lead to
jitter. This can be avoided by shrinking the collision.

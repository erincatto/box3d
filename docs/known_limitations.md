# Known Limitations {#known-limitations}

Box3D makes trade-offs for performance and stability. The behaviors on this page are known and
intentional. They are not bugs, so please don't report them as issues. Each section describes the
symptom, why it happens, and what to do about it.

## Cargo drifts on a rotating body

**Symptom:** a body resting on a rotating body, such as a crate on a turning platform or a rolling
ship deck, slowly slides across it even though friction should hold it in place. The drift grows
with the rotation rate and with the distance from the rotation axis. There is none at the axis.

**Cause:** contact recycling. A recycled contact keeps the normal and anchors it was computed with,
fixed in world space, and only updates the separation. Two bodies that rotate together without
moving relative to each other keep their contact recycled until either body has turned about 10
degrees (`B3_CONTACT_RECYCLE_ANGULAR_DISTANCE`). Meanwhile friction holds the cargo at a point that
lags behind where the carrier actually is.

**Workaround:** disable contact recycling on the carrier or on the cargo. A contact is only recycled
when both bodies have recycling enabled.

```c
bodyDef.enableContactRecycling = false;
```

or at runtime with `b3Body_EnableContactRecycling`. Setting the world recycle distance to zero with
`b3World_SetContactRecycleDistance` also removes the drift, but costs performance and stability
everywhere.

**Samples:** _Issues / Rolling Deck_ and _Issues / Turning Platform_ show the drift side by side with
recycling on and off.

See also: [Contact Recycling](simulation.md#contact-recycling).

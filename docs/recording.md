# Recording and Replay

Deterministic recording, replay, and world snapshots are planned features for Box3D but are not yet
available. Box3D's sister engine Box2D ships a full recording and snapshot API; a 3D equivalent
will follow once the simulation format stabilizes.

In the meantime:

- Box3D is deterministic given the same inputs and the same build. See the **Determinism** section
  of the Simulation page for details on what "same inputs" means across platforms and thread counts.
- There is no API to serialize or restore a world state in the current release.
- There is no replay viewer in the current samples application.

Check the [Box3D repository](https://github.com/erincatto/box3d) for updates on this feature.

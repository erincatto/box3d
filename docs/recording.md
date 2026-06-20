# Recording and Replay {#recording}

Box3D can record a simulation into a memory buffer and replay it later, reproducing the original
run exactly. A recording is a snapshot of the world at the moment recording starts followed by a
log of every world-mutating API call after that. Replaying it re-runs the engine from the
snapshot over those same calls, so the bodies follow the same paths and end up in the same state.

The main use is for debugging. You can record a session, save the buffer to a file, then load it
in the sample app and view the Box3D state as the simulation progresses.

You own the recording buffer. Box3D records into it and grows it as needed; you save it to disk
and free it. The library does no file I/O of its own for recording, beyond two optional
convenience helpers (`b3SaveRecordingToFile` / `b3LoadRecordingFromFile`).

A recording can start before the first step, capturing the whole session, or mid-session for the
"a bug appeared after 30 seconds and only then did I turn recording on" case. Either way the
buffer opens with a snapshot of the world as it stands when recording starts, so there is one
code path for both.

A recording is self-contained. Shape geometry that cannot ride along as plain data, hulls,
meshes, heightfields, and compounds, is interned once into a registry stored in the recording, so
the file carries everything needed to rebuild the world. No external mesh assets are required to
replay.

## Recording

Create a recording buffer, start recording the world into it, then run the simulation as usual.
Start before the first step to capture everything.

```c
b3WorldId worldId = b3CreateWorld( &worldDef );

b3Recording* recording = b3CreateRecording( 0 );    // 0 = default capacity (64 KiB)
b3World_StartRecording( worldId, recording );        // snapshots the world, then logs calls

// ... create bodies, step the world as usual ...

b3World_StopRecording( worldId );
```

`b3CreateRecording` takes a byte capacity to pre-size the buffer. The buffer still grows on
demand, so any value is safe; pre-sizing just avoids reallocations during a long session. Pass
`0` for the default.

`b3World_StartRecording` must be called at a step boundary. It serializes a snapshot of the
current world as the seed, so it works whether you call it before any bodies exist or deep into
a running simulation. It has no effect if the world is already recording. The buffer is reset on
each call, so a single `b3Recording` can be reused for several sessions.

When you are done, save the buffer and free it. File I/O is yours to do; the convenience helper
writes the raw bytes:

```c
b3SaveRecordingToFile( recording, "session.b3rec" );   // or fwrite the bytes yourself:
// const uint8_t* data = b3Recording_GetData( recording );
// int size = b3Recording_GetSize( recording );

b3DestroyRecording( recording );
```

Stopping is optional: `b3DestroyWorld` detaches an active recording for you. The recording buffer
outlives the world, so you can still save it after the world is gone. You can also keep
simulating after `b3World_StopRecording` without recording, and reuse the same handle for a fresh
recording with another `b3World_StartRecording`.

## Replay

The simplest way to check a recording is the headless validator. It re-runs the engine over the
recorded bytes and confirms every recorded id and per-step state reproduces.

```c
const uint8_t* data = b3Recording_GetData( recording );
int size = b3Recording_GetSize( recording );

bool ok = b3ValidateReplay( data, size, 1 );
// ok == false means replay diverged from the recording.
```

`b3ValidateReplay` runs the replay on a single worker; pass `1` for the `workerCount`. To exercise
multi-worker replay, use the player and `b3RecPlayer_SetWorkerCount` (see below).

To replay a recording from disk, load it into a buffer first:

```c
b3Recording* loaded = b3LoadRecordingFromFile( "session.b3rec" );
bool ok = b3ValidateReplay( b3Recording_GetData( loaded ), b3Recording_GetSize( loaded ), 1 );
b3DestroyRecording( loaded );
```

For stepping through a recording frame by frame, for example to drive a viewer or inspect the
world between steps, use the player handle. The player copies the bytes it is given, so you can
free the source buffer immediately after creating it.

```c
b3RecPlayer* player = b3RecPlayer_Create( data, size, 1 );
b3WorldId worldId = b3RecPlayer_GetWorldId( player );

while ( b3RecPlayer_StepFrame( player ) )
{
    // The replay world now holds the state after b3RecPlayer_GetFrame( player ) steps.
    // Read it with the normal b3Body_Get* and b3World_* functions, or draw it.
}

b3RecPlayer_Restart( player );   // rewind to frame 0 in place; the world id stays the same
b3RecPlayer_Destroy( player );
```

`b3RecPlayer_Create` returns `NULL` if the bytes are malformed or fail the layout gate (see the
determinism contract below). `b3RecPlayer_IsAtEnd` reports when the recording is exhausted, and
`b3RecPlayer_HasDiverged` reports whether a recorded state hash failed to reproduce.
`b3RecPlayer_GetDivergeFrame` returns the first frame that diverged, or `-1`. Divergence is
non-fatal during playback so the viewer can keep playing and show where the run starts to differ.

`b3RecPlayer_GetInfo` returns a `b3RecPlayerInfo` read once at open: frame count, recorded time
step and sub-step count, the length scale in effect when recorded, and the accumulated world
bounds, enough to frame and label the recording before the first step.

### Seeking and keyframes

`b3RecPlayer_SeekFrame` jumps to any frame. A forward seek steps op by op; a backward seek
restores the nearest keyframe and re-steps the remaining gap. A keyframe is a periodic snapshot
the player keeps so backward seeking does not replay from the start. `b3RecPlayer_SetKeyframePolicy`
trades memory for seek speed: it caps the bytes spent on kept snapshots and sets the finest
spacing in frames. The spacing widens automatically as the ring evicts to stay under budget, so
the effective backward-seek granularity is reported by `b3RecPlayer_GetKeyframeInterval`.

### Worker count

`b3RecPlayer_Create` takes a worker count for the replay world; pass `1` to match a serial
recording. `b3RecPlayer_SetWorkerCount` changes it on the live player and is reused whenever the
player rebuilds its world on restart or a backward seek. Replaying at a different worker count
than was recorded re-partitions the constraint graph, so the state-hash check becomes a
cross-thread determinism test.

## The seed snapshot

A recording is seeded by a snapshot: a serialized image of the world's simulation state at a step
boundary. It captures everything the engine needs to continue the simulation: bodies, shapes,
joints, contacts with their warm-start impulses, the island and sleep partition, the broad-phase
trees, the id pools, and the interned shape geometry. It does not capture host wiring (worker
count, task callbacks, user data, the friction and restitution mixers); that is rebuilt or
reinstalled at restore.

This machinery is currently internal, used only to seed and replay recordings. Box3D does not yet
expose a standalone save-state / restore API.

## Debug shapes for replay

The player's replay world is created internally, so a renderer has no chance to build per-shape
draw resources as shapes appear. `b3RecPlayer_SetDebugShapeCallbacks` wires host callbacks into
the replay world for exactly that: one is called when a replayed shape is added (returning a user
draw handle), one when it is removed. Call it once right after `b3RecPlayer_Create` and re-read
the world id afterward, since installing the callbacks rebuilds the world and rewinds to frame 0.
The callbacks persist across restart and backward seeks. The 3D sample needs this or the replay
world draws nothing.

## Viewing a recording

The samples app has a **Replay** category with a **Viewer**. Open a recording from its file row,
choose a keyframe budget and minimum interval in the load dialog, then play, pause, single-step,
seek with the scrubber, restart, and change the replay worker count. A `****DIVERGED****` marker
appears, and the scrubber marks the diverge frame, if a recorded state hash fails to reproduce,
which is a real determinism break, not a viewer bug. The info panel lists the recorded bodies,
shapes, joints, and per-frame spatial queries for inspection.

You can also record any sample: open the **Recording** controls in the sample's panel, set a file
name, and press **Record (restart)** to capture from the start or **Record Now** to capture from
the current step. **Stop** saves the buffer to the file.

## Determinism contract

Replay reproduces the original run exactly only when the replaying build matches the recording
build in the ways that affect the math. Some of this the format enforces on load, and the rest is
your responsibility:

- **Struct layout** is enforced. A recording opens by deserializing a snapshot, which is a raw
  struct image, so the reader's build must have identical struct layouts. The image carries a
  layout hash and `b3RecPlayer_Create` / `b3ValidateReplay` reject a recording whose hash differs
  rather than producing a silently wrong replay. A recording therefore does not replay across a
  build whose internal layout changed, including a change in SIMD width.
- **Pointer width**, **endianness**, and the format version are enforced the same way.
- **Floating-point environment** must match. Box3D builds with `-ffp-contract=off` so fused
  multiply-add does not change results. Building with `-ffast-math` is unsupported.

See the **Determinism** section of the Simulation page for what "same inputs" means across
platforms and thread counts.

## Spatial queries

Overlap and cast queries issued during a recorded step (ray casts, shape casts, overlap tests,
and the character mover casts) are recorded too. On replay each query is re-issued against the
replayed world and its results are compared against what was recorded, so a query that returns
different hits is flagged like any other divergence. The player exposes the recorded queries of
the most recently replayed frame through `b3RecPlayer_GetFrameQueryCount`,
`b3RecPlayer_GetFrameQuery`, and `b3RecPlayer_GetFrameQueryHit`, and
`b3RecPlayer_DrawFrameQueries` draws them layered on top of the world; call it after
`b3World_Draw`.

## Limitations

- **Recordings require a matching struct layout.** A recording is seeded by a raw struct-image
  snapshot gated on an exact layout hash, so it will not load into a build whose internal layout
  changed, even if the float environment and architecture match.
- **User data is not preserved.** `userData` pointers are host addresses with no meaning in the
  replay process, so they are written as zero. Code that keys off user data during replay will
  not see the original pointers.
- **Host callbacks are not captured.** The friction and restitution mixers, preSolve, and
  custom-filter callbacks are host functions, so they are not part of the recording. The default
  mixers are pure functions of their inputs and reproduce exactly. A session that installed a
  custom mixer, or a preSolve or custom-filter callback that changes the simulation, will diverge
  on replay because the internally created replay world runs without them.

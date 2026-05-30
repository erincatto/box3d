# Samples port roadmap: GLFW + custom-GL → sokol_app + render3d

Living roadmap for moving box3d's C++ samples off the old GLFW + GLAD +
custom-GL renderer onto the render3d renderer (sokol_app + sokol_gfx + ImGui),
hand-copied into `samples/`. Update the status table as phases land.

## Where this stands

- **Scaffolding (done).** `samples/gfx/` + `samples/shaders/` + `samples/host/`
  + `extern/sokol/` build green; a placeholder `main.cpp` opens a sokol_app
  window and draws a fixed scene with 0 sokol validation errors.
- **Tracer bullet (this phase).** Retarget the sample infrastructure
  (`sample.{h,cpp}`, `main.cpp`) + shims, and **3 representative samples**:
  `sample_stacking`, `sample_character`, `sample_collision`. The other 15
  `sample_*.cpp` stay out of the CMake build and port mechanically afterward.
  Chosen to cover every hard path once:
  - stacking — baseline: `SetView`, adapter `b3World_Draw`, `DrawTextLine`, UI.
  - character — third-person follow, cursor lock, polled WASD, picking, basis.
  - collision — heavy immediate draws, `DrawWorldString`, picking, `DrawFace`.

## Architecture of the retarget

### Driver loop (`main.cpp`)
sokol_app host driving a `SampleManager`. Callbacks:
- `OnInit`: `InitRenderer` → `InitUI(env, DrawUI, show)` → `InitAdapter` →
  `SampleManager::Startup(w, h)`.
- `OnEvent`: Esc quits; `HandleEvent` (ImGui gate); else feed camera
  (`Camera::OnEvent`) + translate sapp events into `SampleManager::Keyboard` /
  `MouseDown/Up/Move`. A key-state cache is fed here for polled-key samples.
- `OnFrame`: `ResetFrameArena` → `SampleManager::Step` (physics + sample
  `Render` doing `b3World_Draw` through the adapter + `DrawTextLine`) → build
  `FrameInput` from the camera → `RenderFrame` → `StartUIFrame` (fires `DrawUI`
  = picker panel + `m_sample->UpdateUI()`) → `RenderUI` → `sg_commit`. Keeps
  the `--frames N` exit-with-error-count harness.
- `OnCleanup`: `SampleManager::Shutdown` → `ShutdownUI` → `ShutdownRenderer`.

### Box3D debug draw via the adapter
The old sample-side `b3DebugDraw` callbacks (`DrawShapeCallback` →
`DrawDebugShape(scene,...)`, etc.) are deleted. `gfx/debug_adapter.h` already
implements the full table into the geometry registry + Draw* path. Per
`render3d/docs/HOST_INTEGRATION.md`:
- world create: `b3DefaultWorldDef` → `AttachToWorldDef(&def)` → `b3CreateWorld`.
- per frame: `MakeDebugDraw(&dd)` → set `drawingBounds` from the camera frustum
  → `ApplyGuiFlags(&dd)` → `b3World_Draw`.
- teardown: `ResetAdapterPool()` then `b3DestroyWorld`.
Draw flags are single-source-of-truth via `GetGuiDraw()`; the UI edits them and
`SampleContext::Save/Load` reads/writes through it.

### Shims (`sample_draw.{h,cpp}`)
Bridges the old sample spelling to the new API so sample bodies need minimal
edits:
- `Vec4 MakeColor(b3HexColor)` — the single 0xRRGGBB → linear Vec4 conversion.
- `DrawWorldString(b3Vec3, b3HexColor, fmt, ...)` — formats then
  `gfx/text.h DrawString` (renderer projects via the latched camera; no camera
  arg).
- `DrawFace` → wireframe `DrawTriangle` (debug-vis tradeoff: wireframe).
- Scene-less overloads of the old primitive draws
  (`DrawSphere/DrawCapsule/DrawPoint/DrawLine/...`) adapting box3d shape structs
  → `gfx/draw.h`.
- Key-state cache for polled-key samples (replaces `glfwGetKey`).

### Base class (`sample.{h,cpp}`)
Host `Camera` replaces the old one; drop `GLFWwindow*` and `Scene*`. Manager
loses every `GLFWwindow*` param. `DrawTextLine` → `DrawScreenStringFormat`.
Picking uses `Camera::BuildPickRay`. `ToggleThirdPerson` → `sapp_lock_mouse`.

## Status

| Phase | State |
|---|---|
| Scaffolding (renderer/shaders/host/placeholder main) | done |
| Tracer bullet: infra + 3 samples | done |
| Remaining 15 `sample_*.cpp` | not started |
| Delete old renderer/scene/camera/font/geo_buffer/ssao_buffer | not started |
| B1: committed shader headers, `BOX3D_BUILD_SHADERS` gate | not started |

Tracer bullet landed 2026-05-30: `samples.exe` builds green (VS2026 / D3D11,
Debug) and runs `--frames` with 0 sokol validation errors across a 9-sample
sweep spanning all three categories (incl. the third-person Mover). Added
`--sample N` to the host to target a sample headlessly.

### What the tracer bullet established (reuse for the remaining 15)

- **Draw compat layer.** `sample_draw.{h,cpp}` keeps the old call spelling
  (`DrawSphere(m_scene, …)`, `DrawWorldString(m_camera, …)`, `MakeColor`) with a
  vestigial `Scene*`/`Camera*` arg the renderer ignores. `draw_bridge.{h,cpp}` is
  the one TU that includes `gfx/draw.h` (C-linkage names can't be overloaded in a
  sample TU). New samples just swap includes; the draw calls port unchanged.
- **Per-sample edits are mechanical:** include block → `sample.h` +
  `sample_draw.h` + `gfx/keycodes.h` (drop `camera/renderer/scene.h`, glad,
  GLFW); `GLFW_MOD_*`→`MOD_*`, `GLFW_MOUSE_BUTTON_1`→`MOUSE_LEFT`,
  `GLFW_KEY_*`→`KEY_*`, `GLFW_PRESS`→`ACTION_PRESS`; `glfwGetKey`→`IsKeyDown`,
  `glfwSetInputMode`→`sapp_lock_mouse`; draw-flag writes
  (`m_context->debugDraw.X`) → `GetGuiDraw()->X` (then include
  `gfx/debug_adapter.h`).
- **Gotchas for the next batch:** the trimmed `SampleContext` has no
  `debugDraw`/`window`/`scene`/`arena`. `shared/` (human, utils/Random*) and
  box3d's `include/` are on the samples target's include dirs explicitly (the
  transitive usage requirements don't cross the subdir boundary here). The
  ImPlot Frame Time chart is gated behind `BOX3D_USE_IMPLOT` (off) pending an
  ImPlot re-add — `build/_deps/implot-src` is already fetched if wired back.

## Deferred

- The 15 other samples (mechanical, follow the stacking pattern; character-like
  and collision-like ones reuse the tracer-bullet solutions).
- Deleting the legacy GLFW/GLAD/custom-GL sources once all 18 samples build on
  the new path.
- B1 so a fresh clone builds without fetching sokol-shdc.

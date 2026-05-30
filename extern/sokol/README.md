# Vendored sokol headers

Upstream: <https://github.com/floooh/sokol>
Pinned commit: `b210caee10d6834081cb46c617c3ff715a92be26`
License: zlib (see `LICENSE`)

These are the only sokol headers render3d actually `#include`s:

| File              | Upstream path             | Used for                                  |
| ----------------- | ------------------------- | ----------------------------------------- |
| `sokol_gfx.h`     | `sokol_gfx.h`             | Graphics backend (D3D11 / Metal / GL 4.3) |
| `sokol_app.h`     | `sokol_app.h`             | Windowing and input                       |
| `sokol_glue.h`    | `sokol_glue.h`            | Bridges sokol_app and sokol_gfx           |
| `sokol_log.h`     | `sokol_log.h`             | Default logger callback                   |
| `sokol_imgui.h`   | `util/sokol_imgui.h`      | Dear ImGui renderer/event handler         |

## Local patches

`sokol_gfx.h` carries a local patch (search the file for `Local patch:`) wiring
`sg_push_debug_group` / `sg_pop_debug_group` on the D3D11 backend to
`ID3DUserDefinedAnnotation::BeginEvent` / `EndEvent`. Upstream sokol_gfx
silently no-ops these on every backend except Metal, so D3D11 captures in
RenderDoc / PIX / Nsight had no hierarchical event regions. The patch:

- includes `<d3d11_1.h>`,
- adds `ID3DUserDefinedAnnotation* annotation` to `_sg_d3d11_backend_t`,
- QueryInterfaces it in `_sg_d3d11_setup_backend`, releases in
  `_sg_d3d11_discard_backend`,
- implements `_sg_d3d11_push_debug_group` / `_sg_d3d11_pop_debug_group`
  (UTF-8 -> UTF-16 conversion, stack buffer, 256 char cap),
- routes the public `_sg_push_debug_group` / `_sg_pop_debug_group`
  dispatcher to them under `SOKOL_D3D11`.

`sokol_app.h` carries a local patch (search the file for `Local patch:`) in
`_sapp_win32_create_window` that changes how `desc.width` / `desc.height`
are interpreted on Win32 when `desc.high_dpi == true`. Upstream always
multiplies the requested size by the monitor's `window_scale`, treating
`desc.width` as 100%-DPI logical pixels — so callers that want a specific
framebuffer size on a high-DPI display have to pre-divide by the DPI scale
themselves. The patch makes the high-DPI path interpret `desc.width` /
`desc.height` as physical/framebuffer pixels directly; the `high_dpi=false`
branch keeps upstream behavior so DPI-unaware apps are unaffected.

When bumping either header, re-apply the patch and rebuild. Each patch is
small enough to re-do by hand each time; if either grows, consider
upstreaming.

## Bumping

1. Pull the new headers from upstream at the desired commit:

   ```
   git clone --depth 1 https://github.com/floooh/sokol /tmp/sokol-src
   (cd /tmp/sokol-src && git fetch --depth 1 origin <sha> && git checkout <sha>)
   cp /tmp/sokol-src/sokol_{gfx,app,glue,log}.h extern/sokol/
   cp /tmp/sokol-src/util/sokol_imgui.h         extern/sokol/
   cp /tmp/sokol-src/LICENSE                     extern/sokol/
   ```

2. Update the pinned commit at the top of this file.
3. Rebuild + run the smoke test on all three OSes.
4. Commit as `deps: bump sokol to <sha> (<reason>)`.

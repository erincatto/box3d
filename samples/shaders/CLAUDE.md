# Shaders

This directory contains all GPU shader code for the renderer. Authored in
GLSL, cross-compiled to per-backend variants (HLSL for D3D11, MSL for Metal,
GLSL 410 for desktop GL) by **sokol-shdc**.

Read this file before writing or editing any `.glsl` file. The root
`CLAUDE.md` covers project-wide conventions; this file covers shader-specific
rules that don't apply elsewhere.

## Toolchain

- Author in **GLSL 4.30 core** dialect, with sokol-shdc directives.
- sokol-shdc generates a single C header per shader file, containing the
  bytecode/source for every enabled backend plus pipeline reflection.
- Generated headers live in `build/shaders/` and are **not committed**.
  They are produced by a CMake custom command at build time.
- Shader edits trigger the custom command automatically; run
  `cmake --build build --target shaders` to force a rebuild.

Backends configured in `cmake/Shaders.cmake`:
- `hlsl5` for D3D11 (Windows)
- `metal_macos` for Metal (macOS)
- `glsl430` for GL 4.3 core (Linux). 4.3 is the floor sokol_gfx requires
  for storage buffers; we use storage buffers for per-instance data.

If sokol-shdc rejects a shader, it's a hard build break - there is no
fallback path. Fix the shader, don't disable a backend.

## File layout

One shader program per `.glsl` file. The filename is the program name.

```
shaders/
  CLAUDE.md                this file
  sky.glsl                          procedural Preetham sky
  common/                           shared @include fragments
    pbr.glsl                        PBR helpers (BRDF, IBL evaluation)
    preetham.glsl                   Preetham sky radiance function
    xe_gtao_common.glsl             XeGTAO helper block (PrefilterDepth,
                                    MainPass, Denoise primitives)
  shapes/
    cube.glsl                       instanced cube
    sphere.glsl                     ray-cast sphere impostor
    capsule.glsl                    ray-cast capsule impostor
    geom.glsl                       hulls + meshes + heightfields
    edge.glsl                       per-shape edge overlay
    shadow_caster_cube.glsl         depth-only shadow caster,
    shadow_caster_sphere.glsl       per-shape, one per primitive class
    shadow_caster_capsule.glsl
    shadow_caster_geom.glsl
  ibl/                              one-time cubemap / LUT builders
    sky_to_cube.glsl                rasterize sky into a cubemap face
    prefilter.glsl                  GGX prefilter the IBL cubemap
    brdf_lut.glsl                   split-sum integration LUT
  post/
    depth_only_cube.glsl            GTAO/shadow prepass per primitive
    depth_only_sphere.glsl          class - outputs linear view-Z
    depth_only_capsule.glsl
    depth_only_geom.glsl
    gtao_prefilter_depth.glsl       XeGTAO compute
    gtao_main_pass.glsl
    gtao_denoise.glsl
    mask_sphere.glsl                hover/select highlight mask,
    mask_capsule.glsl               uint output, GREATER depth test
    mask_hull.glsl
    highlight_outline.glsl          edge-detect + premult-alpha composite
    tonemap.glsl                    AgX
  overlays/
    line.glsl                       shader-expanded lines
    point.glsl                      shader-expanded points
```

Each shader currently inlines its own UBO declarations (`ub_frame`,
`ub_pass`, `ub_draw`) - there is no shared `frame_ubo.glsl` /
`pass_ubo.glsl` block to `@include`. The three shared blocks that do
exist are `common/pbr.glsl`, `common/preetham.glsl`, and
`common/xe_gtao_common.glsl`; the lit shape shaders `@include` PBR and
the sky shaders include Preetham. Shadow-cascade sampling is inlined
per shape shader (`sampleShadowPCF` / `sampleCascade` /
`sampleShadowCascaded` in each of cube/sphere/capsule/geom) - keep
those four implementations in sync if you change the algorithm.

A shader file has, in order:
1. `@ctype` directives mapping GLSL types to C types (mat4 -> Mat4,
   vec4 -> Vec4, vec3 -> b3Vec3, vec2 -> b3Vec2).
2. `@include` of any common blocks needed.
3. `@vs` block - vertex stage.
4. `@fs` block - fragment stage.
5. `@program <name> <vs> <fs>` declaration.

Don't put multiple programs in one file. Don't share `@vs` or `@fs` blocks
across programs except via `@include`.

## Naming inside shaders

- Uniform blocks: `ub_frame`, `ub_pass`, `ub_draw`. Lowercase, prefixed `ub_`.
- Samplers: `smp_<purpose>` (e.g. `smp_shadow`, `smp_blue_noise`).
- Textures: `tex_<purpose>` (e.g. `tex_depth`, `tex_normal`, `tex_shadow`).
- Vertex inputs: `in_<name>` (e.g. `in_pos`, `in_normal`, `in_inst_xform0`).
- Varyings (vs->fs): `v_<name>` (e.g. `v_view_pos`, `v_world_normal`).
- Fragment outputs: `out_<name>` (e.g. `out_color`, `out_normal`, `out_id`).
- Locals: `snake_case`. Constants: `SCREAMING_SNAKE_CASE`.

## Uniform organization

Three UBO frequencies, **never** mixed:

```glsl
// ub_frame - updated once per frame, bound at slot 0
layout(binding=0) uniform ub_frame {
    mat4 view;
    mat4 proj;
    mat4 view_proj;
    mat4 inv_view_proj;
    vec4 camera_pos;        // .xyz = pos, .w = time
    vec4 viewport;          // .xy = size px, .zw = 1/size
    vec4 sun_dir;           // .xyz = world-space dir TO sun, .w = unused
    vec4 sun_color;         // .rgb = color * intensity, .a = ambient strength
};

// ub_pass - updated once per render pass, bound at slot 1
layout(binding=1) uniform ub_pass {
    // per-pass data: shadow matrices, post-process knobs, etc.
};

// ub_draw - updated per draw call, bound at slot 2
layout(binding=2) uniform ub_draw {
    // small per-draw data; instance arrays go in storage buffers, not UBOs
};
```

Per-instance data lives in **storage buffers**, indexed by `gl_InstanceIndex`.
Don't pack instance data into UBOs - they're size-limited and the wrong tool.

A UBO declared at `binding=N` is **stage-specific** in sokol-shdc - declaring
the same UBO in both `@vs` and `@fs` collides. Place each UBO in the stage
that reads it; if a value is needed in both stages, either replicate it at
a different binding, precompute on the C side and split into VS/FS-only
UBOs, or pass via a varying. The three-tier scheme above describes update
frequencies, not a guarantee that every shader exposes all three to both
stages.

## Coordinate spaces

Be explicit about which space a value lives in. Suffix variable names:
`*_world`, `*_view`, `*_clip`, `*_ndc`, `*_screen`. Mixing spaces silently
is the single most common shader bug.

- **Reverse-Z**: clip-space Z is 1 at near, 0 at far. Depth comparison is
  `GREATER`. Far-plane clears to 0, not 1.
- **View space**: right-handed, Y-up, Z out of screen (camera looks down -Z).
- **NDC**: `[-1, 1]` for X and Y; Z is `[0, 1]` (reverse-Z).
- **UV**: `[0, 1]`, top-left origin in sampling. sokol-shdc handles the
  GL/D3D/Metal Y-flip difference automatically when sampling render targets;
  don't flip manually.

When reconstructing world position from depth, sample reverse-Z and
transform through `inv_view_proj` (declared in the per-shader UBO that
needs it - see `sky.glsl`, `overlays/line.glsl`, the lit shape
shaders). There is no shared math helper for this - it's inlined where
needed.

## Flat shading

Compute view-space normal from `dFdx`/`dFdy` of view-space position:

```glsl
vec3 dx = dFdx(v_view_pos);
vec3 dy = dFdy(v_view_pos);
vec3 n_view = normalize(cross(dx, dy));
```

This works for any triangle mesh and gives free flat shading without
duplicating vertices. Impostors compute their analytic normal directly
and skip this.

Do **not** pass per-vertex normals as varyings unless the shape needs
smooth shading (which, in this renderer, none of them do).

## Impostor shaders

Sphere and capsule impostors all follow the same structure:

1. Vertex shader emits a bounding proxy (cube for spheres, OBB for capsules).
2. Fragment shader reconstructs the view ray from `gl_FragCoord` and
   `inv_view_proj`.
3. Ray-cast against the analytic primitive in **world space** (not view
   space - keeps the math simpler when transforming ray origins).
4. On miss: `discard`.
5. On hit: write `gl_FragDepth` from the hit point in clip space, output
   the analytic surface normal in view space.

Reference math: Inigo Quilez, https://iquilezles.org/articles/intersectors/.

`gl_FragDepth` writes disable early-Z. That's fine for impostors but it
means the impostor pipelines don't get the hi-Z optimization. Don't write
`gl_FragDepth` from triangle-mesh shaders - let the rasterizer do it.

When writing `gl_FragDepth`, write the **reverse-Z** value (1 at near,
0 at far). The impostor shaders show the pattern: compute clip-space Z
from a world-space hit point via the view-proj matrix, then
`gl_FragDepth = clip.z / clip.w`.

## Shadow sampling

The four lit shape shaders (`cube.glsl`, `sphere.glsl`, `capsule.glsl`,
`geom.glsl`) each inline their own `sampleShadowPCF` /
`sampleCascade` / `sampleShadowCascaded` functions. They read
`cascade_matrices[3]`, `cascade_far_view_z`, and the shadow-atlas
sampler from the per-shape UBO + per-pass binding. The four
implementations are intentionally identical - if you change the
algorithm (kernel, bias formula, cascade-blend logic), update all four
at once. A future cleanup would lift them into a shared
`common/shadow.glsl` block; until then, treat the four copies as one
unit.

Don't sample shadow maps from any other shader without going through
the same helper trio - non-standard sample patterns (PCSS,
contact-hardening) belong inside `sampleShadowPCF` behind a flag, not
forked into new call sites.

## Things to NOT do

- Don't use `#ifdef <BACKEND>` to branch on backend. sokol-shdc handles
  cross-compilation; backend-specific shader code means we've lost.
- Don't sample a depth target with a regular sampler when you need raw
  depth values. Use the depth-specific sampler binding declared in the
  pass - drivers compare-sample by default on depth textures otherwise.
- Don't use derivatives (`dFdx`/`dFdy`/`fwidth`) in non-uniform control
  flow. They produce undefined results inside `if`/`discard` branches.
  Compute derivatives at top level, then branch.
- Don't write to `gl_FragDepth` conditionally (some pixels write, others
  don't). Either every pixel writes it or none do - otherwise the early-Z
  decision is per-draw, not per-pixel, and you get the worst of both.
- Don't use `texture()` without an explicit LOD in fragment shaders that
  contain `discard` or `gl_FragDepth` writes after the sample. The
  derivative for mipmap selection becomes undefined. Use `textureLod()`
  with LOD 0 if mipmapping isn't needed.
- Don't pack different update frequencies into one UBO. Use the three-tier
  scheme above.
- Don't store full 4x4 instance transforms. 3x4 affine; the bottom row is
  always `(0, 0, 0, 1)` and we don't need perspective from instance data.
- Don't encode normals as `vec3` in storage buffers if you're tight on
  bandwidth - octahedral encoding to two `f16`s saves the byte. There's
  no shared helper yet; if a second use case comes up, lift it into
  `common/`. Don't do this preemptively; profile first.
- Don't write debug colors (red for "this branch ran") and leave them in.
  Use the debug view-mode mechanism instead - it's a uniform switch, not
  edited shader code.
- Don't add new uniform blocks without updating the C-side reflection.
  sokol-shdc generates `_uniformblock_size()` and `_uniformblock_slot()`
  helpers; use them, don't hardcode sizes or slot indices.

## Debugging shaders

- The **debug view-modes** machinery is controlled by
  `fi.debug_view_mode` and runtime keys 0-6. A single uniform selects:
  lit, view-space distance, CSM cascade index, view-space normal, raw
  GTAO AO. This is faster than RenderDoc for 90% of issues.
- Use **RenderDoc** on Linux/Windows or **Xcode GPU frame capture** on
  macOS for the other 10%. Both work with sokol's backends.
- When a shader looks wrong, check coordinate spaces first, depth
  reverse-Z second, derivatives in branches third. In that order.
- sokol's validation layer catches most binding mistakes. If something
  renders black, check the validation log before reading shader code.

## When adding a new shader

1. Decide which file it belongs in (or create a new one if it's a new
   pipeline). One program per file.
2. Add `@ctype` directives matching the C-side struct layouts.
3. Pick the right UBO tier for each uniform - frame, pass, or draw.
4. Author the shader; build with `cmake --build build --target shaders`
   and watch sokol-shdc output for warnings (it's strict, that's good).
5. Make sure at least one scene in `main.cpp`'s `k_sceneTable` exercises
   the new pipeline - the smoke test cycles every scene, so a scene
   that touches the shader is render3d's regression net for sokol
   validation errors. If no existing scene covers it, add a new
   `src/app/*_scene.{cpp,h}` (mirror the `pbr_scene` / `gtao_scene`
   pattern) and wire it through `SceneKind`.
6. Update this file if the shader introduces a new convention worth
   documenting.

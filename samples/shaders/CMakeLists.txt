# Shader compilation via sokol-shdc.

# Each .glsl file in shaders/ becomes a single C header in
# generated/<name>.glsl.h that holds source/bytecode for every enabled backend
# (D3D11, Metal, GL 4.5 core). The generated headers are committed so a fresh
# clone builds without sokol-shdc. BOX3D_BUILD_SHADERS (default ON) recompiles
# them in place when a .glsl changes; off builds straight from the committed
# headers and fetches no tooling.

set(SHADER_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated"
    CACHE INTERNAL "Directory holding generated shader headers")

if(BOX3D_BUILD_SHADERS)
    if(NOT SOKOL_SHDC_EXECUTABLE OR NOT EXISTS "${SOKOL_SHDC_EXECUTABLE}")
        message(FATAL_ERROR
            "SOKOL_SHDC_EXECUTABLE is not set or does not exist: "
            "${SOKOL_SHDC_EXECUTABLE}")
    endif()
    file(MAKE_DIRECTORY "${SHADER_OUTPUT_DIR}")

    # Shared @include fragments - sokol-shdc's @include resolves relative to
    # the input file, so shaders include these via "../common/<name>.glsl".
    # Tracked as DEPENDS on every shader so an edit to a shared file triggers
    # a rebuild of everything that might include it, and listed in the shaders
    # target's SOURCES so they show up in IDE project trees - sokol-shdc never
    # sees them as direct inputs, so they would be invisible otherwise.
    # CONFIGURE_DEPENDS re-globs when a new common file appears, at the cost of
    # a configure-step stat each build.
    file(GLOB SHADER_COMMON_INCLUDES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/common/*.glsl")
endif()

# Reset the accumulators on every configure so stale entries from a prior
# `cmake --fresh`-skipped run don't carry over.
set_property(GLOBAL PROPERTY SHADER_OUTPUTS "")
set_property(GLOBAL PROPERTY SHADER_INPUTS  "")

# Backends: hlsl5 (D3D11), metal_macos (Metal), glsl430 (GL 4.5).
set(SHADER_LANGUAGES "hlsl5:metal_macos:glsl430")

# Registers a custom_command per .glsl input so only changed shaders re-run
# sokol-shdc on incremental builds. Skipped when BOX3D_BUILD_SHADERS is off -
# the committed header stands in for the input then. Either way the output
# path is recorded for the include dir and the IDE source list. After every
# shader is declared the trailing block produces the `shaders` target (the
# build entry point) and the `shaders` INTERFACE library consumers link to.
function(add_shader name input)
    if(NOT IS_ABSOLUTE "${input}")
        set(input "${CMAKE_CURRENT_SOURCE_DIR}/${input}")
    endif()
    set(output "${SHADER_OUTPUT_DIR}/${name}.glsl.h")

    if(BOX3D_BUILD_SHADERS)
        add_custom_command(
            OUTPUT  "${output}"
            COMMAND "${SOKOL_SHDC_EXECUTABLE}"
                    --input  "${input}"
                    --output "${output}"
                    --slang  "${SHADER_LANGUAGES}"
                    # --reflection
            DEPENDS "${input}" ${SHADER_COMMON_INCLUDES}
            COMMENT "sokol-shdc ${name}.glsl"
            VERBATIM
        )
        set_source_files_properties("${output}" PROPERTIES GENERATED TRUE)
    endif()

    set_property(GLOBAL APPEND PROPERTY SHADER_OUTPUTS "${output}")
    set_property(GLOBAL APPEND PROPERTY SHADER_INPUTS "${input}")
endfunction()

# shapes
add_shader(cube "shapes/cube.glsl")
add_shader(sphere "shapes/sphere.glsl")
add_shader(capsule "shapes/capsule.glsl")
add_shader(geom "shapes/geom.glsl")
add_shader(edge "shapes/edge.glsl")
add_shader(shadow_caster_cube "shapes/shadow_caster_cube.glsl")
add_shader(shadow_caster_geom "shapes/shadow_caster_geom.glsl")
add_shader(shadow_caster_sphere "shapes/shadow_caster_sphere.glsl")
add_shader(shadow_caster_capsule "shapes/shadow_caster_capsule.glsl")

# ibl
add_shader(sky "ibl/sky.glsl")
add_shader(brdf_lut "ibl/brdf_lut.glsl")
add_shader(sky_to_cube "ibl/sky_to_cube.glsl")
add_shader(prefilter "ibl/prefilter.glsl")

# post
add_shader(depth_only_cube "post/depth_only_cube.glsl")
add_shader(depth_only_sphere "post/depth_only_sphere.glsl")
add_shader(depth_only_capsule "post/depth_only_capsule.glsl")
add_shader(depth_only_geom "post/depth_only_geom.glsl")
add_shader(gtao_prefilter_depth "post/gtao_prefilter_depth.glsl")
add_shader(gtao_main_pass "post/gtao_main_pass.glsl")
add_shader(gtao_denoise "post/gtao_denoise.glsl")
add_shader(tonemap "post/tonemap.glsl")
add_shader(mask_sphere "post/mask_sphere.glsl")
add_shader(mask_capsule "post/mask_capsule.glsl")
add_shader(mask_hull "post/mask_hull.glsl")
add_shader(highlight_outline "post/highlight_outline.glsl")

# overlay
add_shader(overlay_line "overlays/line.glsl")
add_shader(overlay_point "overlays/point.glsl")

get_property(_outputs GLOBAL PROPERTY SHADER_OUTPUTS)
get_property(_inputs  GLOBAL PROPERTY SHADER_INPUTS)

if(BOX3D_BUILD_SHADERS)
    # DEPENDS drives the build (run sokol-shdc when the .glsl inputs change).
    # SOURCES is purely an IDE hint - Visual Studio and Xcode show listed files
    # in the project tree even though nothing here is compiled. We list the leaf
    # .glsl shaders, the common/ @include fragments, and the generated .glsl.h.
    add_custom_target(shaders
        DEPENDS ${_outputs}
        SOURCES ${_inputs} ${SHADER_COMMON_INCLUDES} ${_outputs}
    )

    # IDE grouping. TREE mirrors the on-disk shaders layout.
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
        FILES ${_inputs} ${SHADER_COMMON_INCLUDES})
    source_group("Generated" FILES ${_outputs})

    # The Visual Studio generator emits a build-stamp file named
    # CMakeFiles/<target> for each custom target. Left ungrouped it floats at
    # the project root as a stray "shaders" node with a can't-find-file icon -
    # file it under the build-rule machinery.
    source_group("CMake Rules" FILES "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/shaders")
else()
    # No codegen: the samples build straight from the committed headers.
    add_custom_target(shaders SOURCES ${_inputs} ${_outputs})
endif()

# shader_gen is the INTERFACE library consumers link to. It forwards the include
# path for the generated headers (SYSTEM so /WX doesn't trip on shdc output) and
# pulls in the shaders target so any link forces the headers to exist first.
add_library(shader_gen INTERFACE)
add_dependencies(shader_gen shaders)
target_include_directories(shader_gen SYSTEM INTERFACE "${SHADER_OUTPUT_DIR}")

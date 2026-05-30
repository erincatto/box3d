# Box3D samples: sokol_app + sokol_gfx host driving render3d's renderer.
#
# Scaffolding milestone. Builds the ported renderer (gfx/), its shaders
# (shaders/), the sokol_app host glue (host/), and a single placeholder
# sample in main.cpp. The legacy GLFW + GLAD + custom GL renderer sources
# (renderer.cpp, scene.cpp, shader.cpp, font.cpp, the sample_*.cpp set, ...)
# stay on disk but are not built here yet; they get retargeted in later
# phases. box3d itself is the in-tree target, not a FetchContent copy.

include(FetchContent)

# render3d's gfx/CMakeLists references these per-target interfaces. Box3D
# applies its own warning policy globally, so keep them empty for now.
add_library(render3d_warnings INTERFACE)
add_library(render3d_sanitizers INTERFACE)

# --- Dependencies: imgui core + the sokol-shdc binary. -----------------------
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.8
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_Declare(sokol_tools_bin
    GIT_REPOSITORY https://github.com/floooh/sokol-tools-bin.git
    GIT_TAG        1a9a4e54090fec42c5d13169b638f09f25474953
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(imgui sokol_tools_bin)

# imgui static lib: the five core sources. sokol_imgui is the backend (no
# glfw/opengl3 backend sources), so it is not listed here.
add_library(imgui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/imgui_demo.cpp"
)
target_include_directories(imgui SYSTEM PUBLIC "${imgui_SOURCE_DIR}")
set_target_properties(imgui PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# --- sokol: vendored headers INTERFACE target + per-platform backend. --------
set(sokol_SOURCE_DIR "${CMAKE_SOURCE_DIR}/extern/sokol")
add_library(sokol INTERFACE)
target_include_directories(sokol SYSTEM INTERFACE "${sokol_SOURCE_DIR}")
if(WIN32)
    target_compile_definitions(sokol INTERFACE SOKOL_D3D11)
    target_link_libraries(sokol INTERFACE
        d3d11 dxgi dxguid
        user32 gdi32 shell32 ole32)
elseif(APPLE)
    target_compile_definitions(sokol INTERFACE SOKOL_METAL)
    target_link_libraries(sokol INTERFACE
        "-framework Foundation"
        "-framework Cocoa"
        "-framework QuartzCore"
        "-framework Metal"
        "-framework MetalKit")
elseif(UNIX)
    target_compile_definitions(sokol INTERFACE SOKOL_GLCORE)
    find_package(OpenGL REQUIRED)
    find_package(Threads REQUIRED)
    find_package(X11 REQUIRED)
    if(NOT X11_Xi_FOUND OR NOT X11_Xcursor_FOUND)
        message(FATAL_ERROR
            "sokol_app needs Xi and Xcursor; install libxi-dev and libxcursor-dev.")
    endif()
    target_link_libraries(sokol INTERFACE
        OpenGL::GL Threads::Threads ${CMAKE_DL_LIBS} m
        X11::X11 X11::Xi X11::Xcursor)
else()
    message(FATAL_ERROR "Unsupported platform for sokol backend selection.")
endif()

# sokol-shdc executable path, consumed by shaders/CMakeLists.txt.
if(WIN32)
    set(_shdc_subdir win32)
    set(_shdc_exe    sokol-shdc.exe)
elseif(APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
        set(_shdc_subdir osx_arm64)
    else()
        set(_shdc_subdir osx)
    endif()
    set(_shdc_exe sokol-shdc)
else()
    set(_shdc_subdir linux)
    set(_shdc_exe    sokol-shdc)
endif()
set(SOKOL_SHDC_EXECUTABLE
    "${sokol_tools_bin_SOURCE_DIR}/bin/${_shdc_subdir}/${_shdc_exe}"
    CACHE FILEPATH "Path to sokol-shdc binary")
unset(_shdc_subdir)
unset(_shdc_exe)

# --- Renderer + shaders (ported verbatim from render3d). ---------------------
add_subdirectory(shaders)
add_subdirectory(gfx)

# --- Samples executable: sokol_app host + placeholder sample. ----------------
set(SAMPLE_FILES
    main.cpp
    host/camera.cpp
    host/camera.h
    host/gui.cpp
    host/gui.h
    host/imgui_impl.cpp
    host/sokol_app_impl.c
)

add_executable(samples ${SAMPLE_FILES})

if(APPLE)
    # sokol_app's Cocoa backend is Objective-C; compile its impl TU as OBJC
    # with ARC, matching render3d's app/CMakeLists.
    set_source_files_properties(host/sokol_app_impl.c PROPERTIES LANGUAGE OBJC)
    set_source_files_properties(host/sokol_app_impl.c PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
endif()

# Include root is the samples dir so "gfx/..." and "host/..." prefixes resolve.
target_include_directories(samples PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(samples PRIVATE gfx imgui sokol)

set_target_properties(samples PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SAMPLE_FILES})

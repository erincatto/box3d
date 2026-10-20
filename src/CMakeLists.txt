include(GNUInstallDirs)

set(BOX3D_SOURCE_FILES
	aabb.c
	aabb.h
	algorithm.h
	arena_allocator.c
	arena_allocator.h
	bitset.c
	bitset.h
	block_allocator.c
	block_allocator.h
	body.c
	body.h
	broad_phase.c
	broad_phase.h
	capsule.c
	compound.c
	compound.h
	constraint_graph.c
	constraint_graph.h
	contact_solver_wide.inl
	contact_solver.c
	contact_solver.h
	contact.c
	contact.h
	container.h
	convex_manifold_wide.inl
	convex_manifold.c
	core.c
	core.h
	ctz.h
	distance_joint.c
	distance.c
	dynamic_tree.c
	dynamic_tree.h
	height_field.c
	hull.c
	hull.h
	id_pool.c
	id_pool.h
	island.c
	island.h
	joint.c
	joint.h
	manifold.c
	manifold.h
	math_functions.c
	math_internal.h
	mesh_contact.c
	mesh.c
	motor_joint.c
	mover.c
	name_cache.c
	name_cache.h
	parallel_for.c
	parallel_for.h
	parallel_joint.c
	physics_world.c
	physics_world.h
	platform.h
	prismatic_joint.c
	qsort.h
	rapidhash.h
	recording_ops.inl
	recording.c
	recording.h
	replay.c
	replay.h
	revolute_joint.c
	scheduler.c
	scheduler.h
	sensor.c
	sensor.h
	shape.c
	shape.h
	simd_avx2.h
	simd_neon.h
	simd_scalar.h
	simd_sse2.h
	simd_wide.h
	simd.c
	simd.h
	solver_set.c
	solver_set.h
	solver.c
	solver.h
	sphere.c
	spherical_joint.c
	table.c
	table.h
	timer.c
	triangle_manifold.c
	types.c
	verstable.h
	weld_joint.c
	wheel_joint.c
	wide4.c
	wide8.c
	world_snapshot.c
	world_snapshot.h

	box3d.natvis
)

set(BOX3D_INCLUDE_FILES
	../include/box3d/base.h
	../include/box3d/box3d.h
	../include/box3d/collision.h
	../include/box3d/config.h
	../include/box3d/constants.h
	../include/box3d/id.h
	../include/box3d/math_functions.h
	../include/box3d/types.h
)

# Hide internal functions
# https://gcc.gnu.org/wiki/Visibility
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

add_library(box3d ${BOX3D_SOURCE_FILES} ${BOX3D_INCLUDE_FILES})

target_include_directories(box3d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(CMAKE_DEBUG_POSTFIX "d")

# Box3D uses C17 for _Static_assert and anonymous unions
set_target_properties(box3d PROPERTIES
	C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS YES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
	DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
)

if (BOX3D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(box3d PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

if (BOX3D_PROFILE)
	target_compile_definitions(box3d PRIVATE BOX3D_PROFILE)
	set(TRACY_ENABLE ON CACHE BOOL "Enable profiling" FORCE)
	set(TRACY_MANUAL_LIFETIME ON CACHE BOOL "Enable manual lifetime for Tracy" FORCE)

	FetchContent_Declare(
		tracy
		GIT_REPOSITORY https://github.com/wolfpld/tracy.git
		GIT_TAG v0.14.1
		GIT_SHALLOW TRUE
		GIT_PROGRESS TRUE
	)
	FetchContent_MakeAvailable(tracy)

	target_link_libraries(box3d PUBLIC TracyClient)
endif()

if(BOX3D_VALIDATE)
	# Only effective when NDEBUG is undefined, see base.h
	message(STATUS "Box3D validation ON (debug builds)")
	target_compile_definitions(box3d PRIVATE BOX3D_VALIDATE)
endif()

target_compile_definitions(box3d PRIVATE ${BOX3D_SIMD_DEFINITIONS})
target_compile_options(box3d PRIVATE ${BOX3D_SIMD_OPTIONS})

if (BOX3D_DOUBLE_PRECISION)
	message(STATUS "Box3D double precision ON")
	# PUBLIC: consumers must see the same precision mode in the headers or they cannot match the ABI
	target_compile_definitions(box3d PUBLIC BOX3D_DOUBLE_PRECISION)
endif()

# Enable asserts in release with debug info
target_compile_definitions(box3d PUBLIC "$<$<CONFIG:RELWITHDEBINFO>:B3_ENABLE_ASSERT>")

if (MSVC)
	message(STATUS "Box3D on MSVC")
	if (BUILD_SHARED_LIBS)
		# this is needed by DLL users to import Box3D symbols
		target_compile_definitions(box3d INTERFACE BOX3D_DLL)
	endif()

	# Visual Studio won't load the natvis unless it is in the project
	target_sources(box3d PRIVATE box3d.natvis)

	if (CMAKE_GENERATOR MATCHES "Visual Studio")
		target_compile_options(box3d PRIVATE /MP)
	endif()

	if (CMAKE_C_COMPILER_ID STREQUAL "Clang")
		# clang-cl: /Wall maps to -Weverything which is far too aggressive
		target_compile_options(box3d PRIVATE /W4 -Wmissing-prototypes)
	else()
		# All Warnings with some disabled
		# 4820 - padding between members
		# 5045 - Spectre mitigation
		# 4061 - not handling all cases for an enum in a switch block
		# 4711 - function inlined without being marked for inline
		# 4710 - inline functions that are not inlined
		target_compile_options(box3d PRIVATE /Wall /wd4820 /wd5045 /wd4061 /wd4711 /wd4710)
	endif()

	if (NOT BOX3D_DISABLE_SIMD AND NOT CMAKE_C_COMPILER_ID STREQUAL "Clang" AND CMAKE_C_COMPILER_ARCHITECTURE_ID MATCHES "^(x64|X86)$")
		set_source_files_properties(wide8.c PROPERTIES COMPILE_OPTIONS "/arch:AVX2")
	endif()

elseif (MINGW)
	message(STATUS "Box3D on MinGW")
	target_compile_options(box3d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
elseif (APPLE)
	message(STATUS "Box3D on Apple")
	target_compile_options(box3d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
elseif (EMSCRIPTEN)
	message(STATUS "Box3D on Emscripten")
elseif (UNIX)
	message(STATUS "Box3D using Unix")
	target_compile_options(box3d PRIVATE -Wmissing-prototypes -Wall -Wextra -pedantic -Wno-unused-value)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${BOX3D_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${BOX3D_INCLUDE_FILES})

add_library(box3d::box3d ALIAS box3d)

# libm is required on many Unix toolchains for sqrtf, etc.
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
  target_link_libraries(box3d PUBLIC m)
endif()

install(
  TARGETS box3d
  EXPORT box3dConfig
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  EXPORT box3dConfig
  NAMESPACE box3d::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box3d"
)

install(
  FILES ${BOX3D_INCLUDE_FILES}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/box3d
)

export(
  EXPORT box3dConfig
  NAMESPACE box3d::
  FILE "${PROJECT_BINARY_DIR}/box3dConfig.cmake"
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/box3dConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES "${PROJECT_BINARY_DIR}/box3dConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/box3d"
)

# Anchor every path to the .pc file itself so the install tree can be relocated,
# by cmake --install --prefix or by moving it afterwards
file(RELATIVE_PATH PKGCONFIG_PREFIX_REL "${CMAKE_INSTALL_FULL_LIBDIR}/pkgconfig" "${CMAKE_INSTALL_PREFIX}")
string(REGEX REPLACE "/$" "" PKGCONFIG_PREFIX_REL "${PKGCONFIG_PREFIX_REL}")
file(RELATIVE_PATH PKGCONFIG_LIBDIR_REL "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_FULL_LIBDIR}")
file(RELATIVE_PATH PKGCONFIG_INCLUDEDIR_REL "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_FULL_INCLUDEDIR}")

# The file has to carry every PUBLIC define the exported target carries, or consumers
# compile against different headers than the library was built with. Keep in sync
# with the defines above.
set(PKGCONFIG_CFLAGS "")
if (BOX3D_DOUBLE_PRECISION)
	string(APPEND PKGCONFIG_CFLAGS " -DBOX3D_DOUBLE_PRECISION")
endif()
if (WIN32 AND BUILD_SHARED_LIBS)
	# base.h keys the dllimport off _WIN32, so MinGW needs this as much as MSVC
	string(APPEND PKGCONFIG_CFLAGS " -DBOX3D_DLL")
endif()
string(APPEND PKGCONFIG_CFLAGS "$<$<CONFIG:RELWITHDEBINFO>: -DB3_ENABLE_ASSERT>")

set(PKGCONFIG_LIBNAME "box3d$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>")

# Plain pkg-config --libs skips Libs.private, and Box3D builds static by default, so
# public link deps belong on the main line unless we are actually building a shared lib
set(PKGCONFIG_LIBS "")
set(PKGCONFIG_LIBS_PRIVATE "")
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
	if(BUILD_SHARED_LIBS)
		set(PKGCONFIG_LIBS_PRIVATE "-lm")
	else()
		set(PKGCONFIG_LIBS " -lm")
	endif()
endif()

configure_file(box3d.pc.in box3d.pc.gen @ONLY)

# The library name and the assert define vary by config, so finish the substitution
# per config instead of once at configure time
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/$<CONFIG>/box3d.pc"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/box3d.pc.gen"
)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/$<CONFIG>/box3d.pc"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
)

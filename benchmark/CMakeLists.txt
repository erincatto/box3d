# Benchmark app

set(BOX3D_BENCHMARK_FILES
	main.c
)

add_executable(benchmark ${BOX3D_BENCHMARK_FILES})

set_target_properties(benchmark PROPERTIES
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_EXTENSIONS NO
)

if (BOX3D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(benchmark PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

target_link_libraries(benchmark PRIVATE box3d shared)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX3D_BENCHMARK_FILES})
# Mesh drop soak. Uses internal threads, so it needs the static library.
if (NOT BUILD_SHARED_LIBS)
	add_executable(mesh_soak mesh_soak.c)

	set_target_properties(mesh_soak PROPERTIES
		C_STANDARD 17
		C_STANDARD_REQUIRED YES
		C_EXTENSIONS NO
	)

	if (BOX3D_COMPILE_WARNING_AS_ERROR)
		set_target_properties(mesh_soak PROPERTIES COMPILE_WARNING_AS_ERROR ON)
	endif()

	target_include_directories(mesh_soak PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
	target_link_libraries(mesh_soak PRIVATE box3d shared)
endif()

# Headless Ghost Collisions sample. Counts ghost manifolds per scene and body.
add_executable(ghost_scan ghost_scan.c)

set_target_properties(ghost_scan PROPERTIES
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_EXTENSIONS NO
)

if (BOX3D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(ghost_scan PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

target_link_libraries(ghost_scan PRIVATE box3d shared)

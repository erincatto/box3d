# Shared code for samples, benchmarks, and unit tests

set(BOX3D_SHARED_FILES
	benchmarks.c
	benchmarks.h
	determinism.c
	determinism.h
	human.c
	human.h
	overflow_color.c
	overflow_color.h
	stability.c
	stability.h
	utils.c
	utils.h
)

add_library(shared STATIC ${BOX3D_SHARED_FILES})

set_target_properties(shared PROPERTIES
	C_STANDARD 17
)

if (BOX3D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(shared PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

target_include_directories(shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(shared PRIVATE box3d)

# The SAT benchmark calls internal functions that a shared Box3D does not export
if (NOT BUILD_SHARED_LIBS)
	target_sources(shared PRIVATE sat_benchmark.c sat_benchmark.h)
	target_include_directories(shared PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
	target_compile_definitions(shared PUBLIC BOX3D_INTERNAL_BENCHMARKS)
	target_compile_definitions(shared PRIVATE ${BOX3D_SIMD_DEFINITIONS})
	target_compile_options(shared PRIVATE ${BOX3D_SIMD_OPTIONS})
endif()

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX3D_SHARED_FILES})
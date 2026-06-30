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

target_include_directories(shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(shared PRIVATE box3d)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX3D_SHARED_FILES})
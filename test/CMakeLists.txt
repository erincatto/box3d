# Box3D unit test app

set(BOX3D_TEST_FILES
    main.c
	test_allocator.c
    test_bitset.c
	test_body.c
	test_body_query.c
	test_collision.c
	test_compound.c
	test_container.c
    test_determinism.c
    test_distance.c
    test_height_field.c
    test_hull.c
    test_id.c
    test_joint.c
    test_large_world.c
    test_macros.h
    test_math.c
	test_mover.c
    test_recording.c
    test_shape.c
    test_table.c
    test_world.c
)

add_executable(test ${BOX3D_TEST_FILES})

set_target_properties(test PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

# Special access to Box3D internals for testing
target_include_directories(test PRIVATE ${CMAKE_SOURCE_DIR}/src)

if(BOX3D_VALIDATE)
	target_compile_definitions(test PRIVATE BOX3D_VALIDATE)
endif()

if(BOX3D_DISABLE_SIMD)
	target_compile_definitions(test PRIVATE BOX3D_DISABLE_SIMD)
endif()

target_link_libraries(test PRIVATE box3d shared)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${BOX3D_TEST_FILES})

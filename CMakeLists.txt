cmake_minimum_required(VERSION 3.22)
include(FetchContent)
include(CMakeDependentOption)

project(box3d
	VERSION 0.1.0
	DESCRIPTION "A 3D physics engine for games"
	HOMEPAGE_URL "https://box2d.org"
	LANGUAGES C CXX
)

# stuff to help debug cmake
# message(STATUS "cmake tool chain: ${CMAKE_TOOLCHAIN_FILE}")
# message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")
# message(STATUS "library postfix: ${CMAKE_DEBUG_POSTFIX}")
message(STATUS "CMake C compiler: ${CMAKE_C_COMPILER_ID}")
message(STATUS "CMake C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake host system processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

# static link to make leak checking easier (_crtBreakAlloc)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if (MSVC OR APPLE OR UNIX)
	option(BOX3D_SANITIZE "Enable sanitizers for some builds" OFF)
	set(BOX3D_SANITIZER_TYPE "address" CACHE STRING "Type of sanitizer to use (address, thread, undefined, memory)")
	set_property(CACHE BOX3D_SANITIZER_TYPE PROPERTY STRINGS "address" "thread" "undefined" "memory")
	
	if(BOX3D_SANITIZE)
		message(STATUS "Box3D Sanitize: ${BOX3D_SANITIZER_TYPE}")
		# sanitizers need to apply to all compiled libraries to work well
		if(MSVC)
			# address sanitizer only in the debug build
			add_compile_options("$<$<CONFIG:Debug>:/fsanitize=address>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL:NO>")
		elseif(APPLE)
			# more sanitizers on Apple clang
			if(BOX3D_SANITIZER_TYPE STREQUAL "thread")
				add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
				add_link_options(-fsanitize=thread)
			elseif(BOX3D_SANITIZER_TYPE STREQUAL "undefined")
				add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
				add_link_options(-fsanitize=undefined)
			else()
				# default to address sanitizer
				add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
				add_link_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
			endif()
		elseif(UNIX)
			# Linux/WSL2 sanitizer support
			if(BOX3D_SANITIZER_TYPE STREQUAL "thread")
				# Disable ASLR first to avoid crash on launch
				# echo 0 | sudo tee /proc/sys/kernel/randomize_va_space
				add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -fPIE)
				add_link_options(-fsanitize=thread -pie)
			elseif(BOX3D_SANITIZER_TYPE STREQUAL "undefined")
				add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
				add_link_options(-fsanitize=undefined)
			elseif(BOX3D_SANITIZER_TYPE STREQUAL "memory")
				add_compile_options(-fsanitize=memory -fno-omit-frame-pointer)
				add_link_options(-fsanitize=memory)
			else()
				# default to address sanitizer
				add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
				add_link_options(-fsanitize=address)
			endif()
		endif()
	else()
		if(MSVC)
			# enable hot reloading
			add_compile_options("$<$<CONFIG:Debug>:/ZI>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL>")
		endif()
	endif()
endif()

# Deterministic math
# https://box2d.org/posts/2024/08/determinism/
if (MINGW OR APPLE OR UNIX)
	add_compile_options(-ffp-contract=off)
endif()

option(BOX3D_DISABLE_SIMD "Disable SIMD math (slower)" OFF)
option(BOX3D_COMPILE_WARNING_AS_ERROR "Compile warnings as errors" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Needed for samples.exe to find box2d.dll
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

add_subdirectory(src)

# This hides samples, test, and doxygen from apps that use box3d via FetchContent
if(PROJECT_IS_TOP_LEVEL)
	option(BOX3D_SAMPLES "Build the Box3D samples" ON)
	option(BOX3D_BENCHMARKS "Build the Box3D benchmarks" OFF)
	option(BOX3D_DOCS "Build the Box3D documentation" OFF)
	option(BOX3D_PROFILE "Enable profiling with Tracy" OFF)
	option(BOX3D_VALIDATE "Enable heavy validation" ON)
	option(BOX3D_UNIT_TESTS "Build the Box3D unit tests" ON)

	# Off builds the samples from the committed shader headers with no
	# sokol-shdc fetch. On recompiles them in place when a .glsl changes.
	option(BOX3D_BUILD_SHADERS "Recompile shaders with sokol-shdc" ON)

	if(BOX3D_UNIT_TESTS OR BOX3D_SAMPLES OR BOX3D_BENCHMARKS)

		# Emscripten pthread support
		if(EMSCRIPTEN)
			set(EMSCRIPTEN_PTHREADS_COMPILER_FLAGS "-pthread -s USE_PTHREADS=1")
			set(EMSCRIPTEN_PTHREADS_LINKER_FLAGS "${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS} -s ALLOW_MEMORY_GROWTH")
			string(APPEND CMAKE_C_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
			string(APPEND CMAKE_CXX_FLAGS " ${EMSCRIPTEN_PTHREADS_COMPILER_FLAGS}")
			string(APPEND CMAKE_EXE_LINKER_FLAGS " ${EMSCRIPTEN_PTHREADS_LINKER_FLAGS}")
		endif()

		add_subdirectory(shared)
	endif()

	# Tests need static linkage because they test internal Box3D functions
	if(NOT BUILD_SHARED_LIBS AND BOX3D_UNIT_TESTS)
		message(STATUS "Adding Box3D unit tests")
		add_subdirectory(test)
		set_target_properties(test PROPERTIES XCODE_GENERATE_SCHEME TRUE)
	else()
		message(STATUS "Skipping Box3D unit tests")
	endif()

	if(BOX3D_SAMPLES)
		add_subdirectory(samples)

		# default startup project for Visual Studio
		if(MSVC)
			set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
			set_property(TARGET samples PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
		endif()

			set_target_properties(samples PROPERTIES
				XCODE_GENERATE_SCHEME TRUE
				XCODE_SCHEME_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
	endif()

	if(BOX3D_BENCHMARKS)
		add_subdirectory(benchmark)
		set_target_properties(benchmark PROPERTIES XCODE_GENERATE_SCHEME TRUE)
	endif()

	if(BOX3D_DOCS)
		# todo
		# add_subdirectory(docs)
	endif()
endif()

# https://stackoverflow.com/questions/72635402/how-to-set-working-directory-in-cmake-visual-studio-2022
# launch.vs.json
# "currentDir": "${workspaceRoot}"
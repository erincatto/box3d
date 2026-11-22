// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/base.h"

#include <stddef.h>

// clang-format off

#ifdef NDEBUG
	#define B3_DEBUG 0
#else
	#define B3_DEBUG 1
#endif

// Define platform
#if defined(_WIN32) || defined(_WIN64)
	#define B3_PLATFORM_WINDOWS
#elif defined( __ANDROID__ )
	#define B3_PLATFORM_ANDROID
#elif defined( __linux__ )
	#define B3_PLATFORM_LINUX
#elif defined( __APPLE__ )
	#include <TargetConditionals.h>
	#if defined( TARGET_OS_IPHONE ) && !TARGET_OS_IPHONE
		#define B3_PLATFORM_MACOS
	#else
		#define B3_PLATFORM_IOS
	#endif
#elif defined( __EMSCRIPTEN__ )
	#define B3_PLATFORM_WASM
#else
	#define B3_PLATFORM_UNKNOWN
#endif

// Define CPU
#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( _M_IX86 )
	#define B3_CPU_X86_X64
#elif defined( __aarch64__ ) || defined( _M_ARM64 ) || defined( __arm__ ) || defined( _M_ARM )
	#define B3_CPU_ARM
#elif defined( __EMSCRIPTEN__ )
	#define B3_CPU_WASM
#else
	#define B3_CPU_UNKNOWN
#endif

// Define SIMD
#if defined( BOX3D_DISABLE_SIMD )
	#define B3_SIMD_NONE
	//#pragma message("B3_SIMD_NONE")
#else
	#if defined( B3_CPU_X86_X64 )
		#define B3_SIMD_SSE2
		#if !defined( BOX3D_DISABLE_AVX2 )
			#define B3_SIMD_HAS_WIDTH_8
			#if defined( __AVX2__ ) || ( defined( B3_SIMD_WIDTH ) && B3_SIMD_WIDTH == 8 )
				#define B3_SIMD_AVX2
			#endif
		#endif
	#elif defined( B3_CPU_ARM )
		// ARMv7 Neon doesn't have divide or sqrt so cannot be used.
		#if defined( __aarch64__ ) || defined( _M_ARM64 )
			#define B3_SIMD_NEON
		#else
			#define B3_SIMD_NONE
		#endif
		//#pragma message("B3_SIMD_NEON")
	#elif defined( B3_CPU_WASM )
		#define B3_SIMD_SSE2
		//#pragma message("B3_SIMD_SSE2")
	#else
		#define B3_SIMD_NONE
		//#pragma message("B3_SIMD_NONE")
	#endif
#endif

#if !defined( B3_SIMD_NONE )
	#define B3_SIMD_ENABLED
#endif

#if defined( B3_SIMD_AVX2 ) && defined( __clang__ )
	#define B3_AVX2_BEGIN _Pragma( "clang attribute push( __attribute__( ( target( \"avx2\" ) ) ), apply_to = function )" )
	#define B3_AVX2_END _Pragma( "clang attribute pop" )
#elif defined( B3_SIMD_AVX2 ) && defined( __GNUC__ )
	#define B3_AVX2_BEGIN _Pragma( "GCC push_options" ) _Pragma( "GCC target( \"avx2\" )" )
	#define B3_AVX2_END _Pragma( "GCC pop_options" )
#else
	#define B3_AVX2_BEGIN
	#define B3_AVX2_END
#endif

// Define compiler
#if defined( __clang__ )
	#define B3_COMPILER_CLANG
#elif defined( __GNUC__ )
	#define B3_COMPILER_GCC
#elif defined( _MSC_VER )
	#define B3_COMPILER_MSVC
#endif

/// Tracy profiler instrumentation
/// https://github.com/wolfpld/tracy
#ifdef BOX3D_PROFILE
	#include <tracy/TracyC.h>
	#define b3TracyCZoneC( ctx, color, active ) TracyCZoneC( ctx, color, active )
	#define b3TracyCZoneNC( ctx, name, color, active ) TracyCZoneNC( ctx, name, color, active )
	#define b3TracyCZoneEnd( ctx ) TracyCZoneEnd( ctx )
	#define b3TracyCFrame TracyCFrameMark
#else
	#define b3TracyCZoneC( ctx, color, active )
	#define b3TracyCZoneNC( ctx, name, color, active )
	#define b3TracyCZoneEnd( ctx )
	#define b3TracyCFrame
#endif

// clang-format on

typedef struct b3AtomicInt
{
	int value;
} b3AtomicInt;

typedef struct b3AtomicU32
{
	uint32_t value;
} b3AtomicU32;

typedef struct b3AtomicI64
{
	// 64-bit atomic wants 8-byte alignment
	_Alignas( 8 ) int64_t value;
} b3AtomicI64;

// Minimum memory alignment used for all allocations
#define B3_ALIGNMENT 32

#define B3_RESTRICT restrict

// Returns the number of elements of an array
#define B3_ARRAY_COUNT( A ) (int)( sizeof( A ) / sizeof( A[0] ) )

// Used to prevent the compiler from warning about unused variables
#define B3_UNUSED( ... ) (void)sizeof( ( __VA_ARGS__, 0 ) )

// Use to validate definitions. Do not take my cookie.
#define B3_SECRET_COOKIE 1152023

#define B3_CHECK_DEF( DEF ) B3_ASSERT( DEF->internalValue == B3_SECRET_COOKIE )
#define B3_CHECK_JOINT_DEF( DEF ) B3_ASSERT( DEF->base.internalValue == B3_SECRET_COOKIE )

// Skip bad input even in release builds.
#define B3_CHECK_INPUT( CONDITION )                                                                                              \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( CONDITION ) == false )                                                                                            \
		{                                                                                                                        \
			b3Log( "invalid input: %s in %s\n", #CONDITION, __func__ );                                                          \
			B3_ASSERT( CONDITION );                                                                                              \
			return;                                                                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

#define B3_CHECK_INPUT_RETURN( CONDITION, RESULT )                                                                               \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( CONDITION ) == false )                                                                                            \
		{                                                                                                                        \
			b3Log( "invalid input: %s in %s\n", #CONDITION, __func__ );                                                          \
			B3_ASSERT( CONDITION );                                                                                              \
			return RESULT;                                                                                                       \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// These macros help avoid sizeof bugs
#define B3_ALLOC( T, N ) (T*)b3Alloc( N * sizeof( T ) );
#define B3_FREE( M, T, N ) b3Free( M, N * sizeof( T ) );

void* b3Alloc( size_t size );
void* b3AllocZero( size_t size );
void b3Free( void* mem, size_t size );
void* b3GrowAlloc( void* oldMem, int oldSize, int newSize );
void* b3GrowAllocZeroed( void* oldMem, int oldSize, int newSize );

#define B3_GROW( A, N, M ) b3GrowAlloc( ( A ), (int)( ( N ) * sizeof( *( A ) ) ), (int)( ( M ) * sizeof( *( A ) ) ) )
#define B3_GROW_ZERO( A, N, M ) b3GrowAllocZeroed( ( A ), (int)( ( N ) * sizeof( *( A ) ) ), (int)( ( M ) * sizeof( *( A ) ) ) )

B3_PRINTF_FORMAT( 1, 2 )
void b3Log( const char* format, ... );

typedef struct b3Mutex b3Mutex;
b3Mutex* b3CreateMutex( void );
void b3DestroyMutex( b3Mutex* m );
void b3LockMutex( b3Mutex* m );
void b3UnlockMutex( b3Mutex* m );

typedef struct b3Semaphore b3Semaphore;
b3Semaphore* b3CreateSemaphore( int initCount );
void b3DestroySemaphore( b3Semaphore* s );
void b3WaitSemaphore( b3Semaphore* s );
void b3SignalSemaphore( b3Semaphore* s );

typedef void b3ThreadFunction( void* context );
typedef struct b3Thread b3Thread;
// Name may be NULL, otherwise it is copied.
b3Thread* b3CreateThread( b3ThreadFunction* function, void* context, const char* name );
void b3JoinThread( b3Thread* t );

void b3StrCpy( char* dst, int size, const char* src );

uint64_t b3Hash64NonZero( const uint8_t* bytes, int n );

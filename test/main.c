// SPDX-FileCopyrightText: 2024 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "box3d/base.h"

#include <string.h>

#if defined( _WIN32 )
#include <crtdbg.h>

// int MyAllocHook(int allocType, void* userData, size_t size, int blockType, long requestNumber, const unsigned char* filename,
//	int lineNumber)
//{
//	if (size == 16416)
//	{
//		size += 0;
//	}
//
//	return 1;
// }
#endif

#ifdef TRACY_ENABLE
#include <tracy/TracyC.h>
#endif

extern int AllocatorTest( void );
extern int BitTest( void );
extern int BodyTest( void );
extern int BodyQueryTest( void );
extern int CollisionTest( void );
extern int CompoundTest( void );
extern int ContainerTest( void );
extern int DeterminismTest( void );
extern int DistanceTest( void );
extern int HeightFieldTest( void );
extern int HullTest( void );
extern int IdTest( void );
extern int JointTest( void );
extern int LargeWorldTest( void );
extern int ManifoldTest( void );
extern int MathTest( void );
extern int MoverTest( void );
extern int NameCacheTest( void );
extern int RecordingTest( void );
extern int ShapeTest( void );
extern int TableTest( void );
extern int WorldTest( void );

// Filter-aware test runner: skips tests that don't match the filter
#define MAYBE_RUN_TEST( T )                                                                                                      \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( filter != NULL && strcmp( filter, #T ) != 0 )                                                                       \
		{                                                                                                                        \
			printf( "test skipped: " #T "\n" );                                                                                  \
			break;                                                                                                               \
		}                                                                                                                        \
		RUN_TEST( T );                                                                                                           \
	}                                                                                                                            \
	while ( false )

int main( int argc, char** argv )
{
#if defined( _WIN32 )
	// Enable memory-leak reports
	//_CrtSetBreakAlloc(196);
	_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_FILE );
	_CrtSetReportFile( _CRT_WARN, _CRTDBG_FILE_STDERR );
	// Route CRT errors and assertions to stderr instead of a modal dialog so a headless run
	// (CI, redirected stdout) fails fast rather than blocking on Abort/Retry/Ignore.
	_CrtSetReportMode( _CRT_ERROR, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_FILE );
	_CrtSetReportFile( _CRT_ERROR, _CRTDBG_FILE_STDERR );
	_CrtSetReportMode( _CRT_ASSERT, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_FILE );
	_CrtSetReportFile( _CRT_ASSERT, _CRTDBG_FILE_STDERR );
	//_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
	//_CrtSetAllocHook(MyAllocHook);
#endif

#ifdef TRACY_ENABLE
	___tracy_startup_profiler();
#endif

	const char* filter = NULL;
	if ( argc > 1 )
	{
		filter = argv[1];
	}

	uint64_t ticks = b3GetTicks();

	printf( "Starting Box3D unit tests\n" );
	if ( filter != NULL )
	{
		printf( "Filter: %s\n", filter );
	}
	printf( "======================================\n" );

	MAYBE_RUN_TEST( AllocatorTest );
	MAYBE_RUN_TEST( BitTest );
	MAYBE_RUN_TEST( BodyTest );
	MAYBE_RUN_TEST( BodyQueryTest );
	MAYBE_RUN_TEST( CollisionTest );
	MAYBE_RUN_TEST( CompoundTest );
	MAYBE_RUN_TEST( ContainerTest );
	MAYBE_RUN_TEST( DeterminismTest );
	MAYBE_RUN_TEST( DistanceTest );
	MAYBE_RUN_TEST( HeightFieldTest );
	MAYBE_RUN_TEST( HullTest );
	MAYBE_RUN_TEST( IdTest );
	MAYBE_RUN_TEST( JointTest );
	MAYBE_RUN_TEST( LargeWorldTest );
	MAYBE_RUN_TEST( ManifoldTest );
	MAYBE_RUN_TEST( MathTest );
	MAYBE_RUN_TEST( MoverTest );
	MAYBE_RUN_TEST( NameCacheTest );
	MAYBE_RUN_TEST( RecordingTest );
	MAYBE_RUN_TEST( ShapeTest );
	MAYBE_RUN_TEST( TableTest );
	MAYBE_RUN_TEST( WorldTest );

	printf( "======================================\n" );
	printf( "All Box3D tests passed!\n" );

	float duration = b3GetMilliseconds( ticks );
	printf( "Test duration = %.2f s\n", 0.001f * duration );

#ifdef TRACY_ENABLE
	___tracy_shutdown_profiler();
#endif

#if defined( _WIN32 )
	if ( _CrtDumpMemoryLeaks() )
	{
		return 1;
	}
#endif

	return 0;
}

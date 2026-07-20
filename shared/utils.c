// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "utils.h"

#if defined( _WIN64 )
// Lower-case windows.h intentionally for cross compiling on mingw.
#include <windows.h>
#elif defined( __APPLE__ )
#include <unistd.h>
#elif defined( __linux__ )
#include <unistd.h>
#elif defined( __EMSCRIPTEN__ )
#include <unistd.h>
#endif

uint32_t g_randomSeed = RAND_SEED;

int GetNumberOfCores( void )
{
#if defined( _WIN64 )
	SYSTEM_INFO sysinfo;
	GetSystemInfo( &sysinfo );
	return sysinfo.dwNumberOfProcessors;
#elif defined( __APPLE__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#elif defined( __linux__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#elif defined( __EMSCRIPTEN__ )
	return (int)sysconf( _SC_NPROCESSORS_ONLN );
#else
	return 1;
#endif
}

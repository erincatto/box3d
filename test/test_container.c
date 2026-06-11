// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "container.h"
#include "core.h"
#include "test_macros.h"

#include <assert.h>
#include <stdint.h>

b3DeclareArrayNative( uint64_t );
b3DeclareArrayNative( int16_t );
b3DeclareArrayNative( uint8_t );

typedef struct Foo
{
	int a;
	float b;
} Foo;

b3DeclareArray( Foo );

typedef struct Bar
{
	b3Array( int ) a;
} Bar;

static int TestCreateDestroy( void )
{
	b3Array( int ) a;
	b3Array_Create( a );
	b3Array_Destroy( a );
	return 0;
}

static int TestAccess( void )
{
	b3Array( int ) a;
	b3Array_Create( a );
	b3Array_Push( a, 42 );
	int* element = b3Array_Get( a, 0 );
	ENSURE( *element == 42 );
	b3Array_Destroy( a );
	return 0;
}

static int TestIteration( void )
{
	b3Array( int ) a = { 0 };
	b3Array_Push( a, 1 );
	b3Array_Push( a, 2 );
	b3Array_Push( a, 3 );

	int sum = 0;
	for ( int i = 0; i < a.count; ++i )
	{
		sum += a.data[i];
	}

	ENSURE( sum == 6 );
	b3Array_Destroy( a );
	return 0;
}

static int TestArrayOfStruct( void )
{
	b3Array( Foo ) a;
	b3Array_Create( a );
	b3Array_Push( a, ( (Foo){ .a = 1, .b = 5.0f } ) );
	b3Array_Push( a, ( (Foo){ .a = 2, .b = 6.0f } ) );
	b3Array_Push( a, ( (Foo){ .a = 3, .b = 7.0f } ) );

	int sum1 = 0;
	float sum2 = 0.0f;
	for ( int i = 0; i < a.count; ++i )
	{
		sum1 += a.data[i].a;
		sum2 += a.data[i].b;
	}

	ENSURE( sum1 == 6 );
	ENSURE( sum2 == 18.0f );

	b3Array_Destroy( a );
	return 0;
}

static int TestStructWithArray( void )
{
	Bar a;
	b3Array_Create( a.a );
	b3Array_Push( a.a, 1 );
	b3Array_Push( a.a, 2 );
	b3Array_Push( a.a, 3 );

	int sum1 = 0;
	for ( int i = 0; i < a.a.count; ++i )
	{
		sum1 += a.a.data[i];
	}

	ENSURE( sum1 == 6 );

	b3Array_Destroy( a.a );
	return 0;
}

static int TestArrayEmplace( void )
{
	b3Array( int ) a = { 0 };

	int n = 100;
	for ( int i = 0; i < n; ++i )
	{
		int* j = b3Array_Emplace( a );
		*j = i;
	}

	int sum = 0;
	for ( int i = 0; i < a.count; ++i )
	{
		sum += a.data[i];
	}

	ENSURE( sum == n * (n - 1) / 2);

	b3Array_Destroy( a );
	return 0;
}

static int TestArrayRemove( void )
{
	b3Array( int16_t ) a = { 0 };

	int n = 100;
	b3Array_Reserve( a, n );
	ENSURE( a.capacity == n && a.count == 0 );

	for ( int i = 0; i < n; ++i )
	{
		b3Array_Push( a, i );
	}

	int sum = 0;
	for ( int i = 0; i < n; ++i )
	{
		int16_t* value = b3Array_Get( a, 0 );
		sum += *value;
		b3Array_RemoveSwap(a, 0);
	}

	ENSURE( sum == n * ( n - 1 ) / 2 );

	b3Array_Destroy( a );
	return 0;
}

static int TestArrayPop( void )
{
	b3Array( int ) a = { 0 };

	int n = 10;
	b3Array_Resize( a, n );
	ENSURE( a.capacity == n && a.count == n );

	for ( int i = 0; i < n; ++i )
	{
		a.data[i] = i;
	}

	int sum = 0;
	while (a.count > 0)
	{
		sum += b3Array_Pop( a );
	}

	ENSURE( sum == n * ( n - 1 ) / 2 );

	b3Array_Destroy( a );
	return 0;
}

int ContainerTest( void )
{
	RUN_SUBTEST( TestCreateDestroy );
	RUN_SUBTEST( TestAccess );
	RUN_SUBTEST( TestIteration );
	RUN_SUBTEST( TestArrayOfStruct );
	RUN_SUBTEST( TestStructWithArray );
	RUN_SUBTEST( TestArrayEmplace );
	RUN_SUBTEST( TestArrayRemove );
	RUN_SUBTEST( TestArrayPop );

	return 0;
}

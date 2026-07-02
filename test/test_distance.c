// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "math_internal.h"
#include "test_macros.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>

static int SegmentDistanceTest( void )
{
	b3Vec3 p1 = { -1.0f, -1.0f };
	b3Vec3 q1 = { -1.0f, 1.0f };
	b3Vec3 p2 = { 2.0f, 0.0f };
	b3Vec3 q2 = { 1.0f, 0.0f };

	b3SegmentDistanceResult result = b3SegmentDistance( p1, q1, p2, q2 );

	ENSURE_SMALL( result.fraction1 - 0.5f, FLT_EPSILON );
	ENSURE_SMALL( result.fraction2 - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( result.point1.x + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( result.point1.y, FLT_EPSILON );
	ENSURE_SMALL( result.point1.z, FLT_EPSILON );
	ENSURE_SMALL( result.point2.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( result.point2.y, FLT_EPSILON );
	ENSURE_SMALL( result.point2.z, FLT_EPSILON );

	return 0;
}

static int ShapeDistanceTest( void )
{
	b3Vec3 vas[] = { (b3Vec3){ -1.0f, -1.0f }, (b3Vec3){ 1.0f, -1.0f }, (b3Vec3){ 1.0f, 1.0f }, (b3Vec3){ -1.0f, 1.0f } };

	b3Vec3 vbs[] = {
		(b3Vec3){ 2.0f, -1.0f },
		(b3Vec3){ 2.0f, 1.0f },
	};

	b3DistanceInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), 0.0f };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), 0.0f };
	input.transform = b3Transform_identity;
	input.useRadii = false;

	b3SimplexCache cache = { 0 };
	b3DistanceOutput output = b3ShapeDistance( &input, &cache, NULL, 0 );

	ENSURE_SMALL( output.distance - 1.0f, FLT_EPSILON );

	return 0;
}

static int ShapeCastTest( void )
{
	b3Vec3 vas[] = { (b3Vec3){ -1.0f, -1.0f, 0.0f }, (b3Vec3){ 1.0f, -1.0f, 0.0f }, (b3Vec3){ 1.0f, 1.0f, 0.0f },
					 (b3Vec3){ -1.0f, 1.0f } };

	b3Vec3 vbs[] = {
		(b3Vec3){ 2.0f, -1.0f, 0.0f },
		(b3Vec3){ 2.0f, 1.0f, 0.0f },
	};

	b3ShapeCastPairInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), 0.0f };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), 0.0f };
	input.transform = b3Transform_identity;
	input.translationB = (b3Vec3){ -2.0f, 0.0f };
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	b3CastOutput output = b3ShapeCast( &input );

	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction - 0.5f, 0.005f );

	return 0;
}

// unit box with a segment resting 2mm from its +x face: inside the cast
// target distance, not overlapped
static b3ShapeCastPairInput EncroachCastInput( b3Vec3 translationB )
{
	static const b3Vec3 vas[] = { { -1.0f, -1.0f, 0.0f }, { 1.0f, -1.0f, 0.0f }, { 1.0f, 1.0f, 0.0f }, { -1.0f, 1.0f, 0.0f } };
	static const b3Vec3 vbs[] = {
		{ 1.002f, -1.0f, 0.0f },
		{ 1.002f, 1.0f, 0.0f },
	};

	b3ShapeCastPairInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), 0.0f };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), 0.0f };
	input.transform = b3Transform_identity;
	input.translationB = translationB;
	input.maxFraction = 1.0f;
	input.canEncroach = true;
	return input;
}

static int ShapeCastEncroachRecedeTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ 1.0f, 0.0f, 0.0f } );
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit == false );

	return 0;
}

static int ShapeCastEncroachSlideTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ 0.0f, 2.0f, 0.0f } );
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit == false );

	return 0;
}

static int ShapeCastEncroachClosingTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ -1.0f, 0.0f, 0.0f } );
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction, FLT_EPSILON );
	ENSURE_SMALL( output.normal.x - 1.0f, 0.001f );

	return 0;
}

static int ShapeCastEncroachGradedTest( void )
{
	// 5mm gap: closing consumes the gap down to the retained rest separation
	static const b3Vec3 vbs[] = {
		{ 1.005f, -1.0f, 0.0f },
		{ 1.005f, 1.0f, 0.0f },
	};

	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ -1.0f, 0.0f, 0.0f } );
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), 0.0f };
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );
	ENSURE_SMALL( output.fraction - 0.0025f, 0.0005f );
	ENSURE_SMALL( output.normal.x - 1.0f, 0.001f );

	return 0;
}

static int ShapeCastEncroachGrazeTest( void )
{
	// mostly tangential slide with a slight closing component: free
	static const b3Vec3 vbs[] = {
		{ 1.005f, -1.0f, 0.0f },
		{ 1.005f, 1.0f, 0.0f },
	};

	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ -0.001f, 2.0f, 0.0f } );
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), 0.0f };
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit == false );

	return 0;
}

static int ShapeCastStartTouchingDefaultTest( void )
{
	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ 1.0f, 0.0f, 0.0f } );
	input.canEncroach = false;
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );

	return 0;
}

static int ShapeCastEncroachOverlapTest( void )
{
	static const b3Vec3 vcs[] = {
		{ 0.5f, -1.0f, 0.0f },
		{ 0.5f, 1.0f, 0.0f },
	};

	b3ShapeCastPairInput input = EncroachCastInput( (b3Vec3){ 1.0f, 0.0f, 0.0f } );
	input.proxyB = (b3ShapeProxy){ vcs, ARRAY_COUNT( vcs ), 0.0f };
	b3CastOutput output = b3ShapeCast( &input );
	ENSURE( output.hit );

	return 0;
}

static int TimeOfImpactTest( void )
{
	b3Vec3 vas[] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };

	b3Vec3 vbs[] = {
		{ 2.0f, -1.0f },
		{ 2.0f, 1.0f },
	};

	b3TOIInput input;
	input.proxyA = (b3ShapeProxy){ vas, ARRAY_COUNT( vas ), 0.0f };
	input.proxyB = (b3ShapeProxy){ vbs, ARRAY_COUNT( vbs ), 0.0f };
	input.sweepA = (b3Sweep){ b3Vec3_zero, b3Vec3_zero, b3Vec3_zero, b3Quat_identity, b3Quat_identity };
	input.sweepB = (b3Sweep){ b3Vec3_zero, b3Vec3_zero, (b3Vec3){ -2.0f, 0.0f, 0.0f }, b3Quat_identity, b3Quat_identity };
	input.maxFraction = 1.0f;

	b3TOIOutput output = b3TimeOfImpact( &input );

	ENSURE( output.state == b3_toiStateHit );
	ENSURE_SMALL( output.fraction - 0.5f, 0.005f );

	return 0;
}

int DistanceTest( void )
{
	RUN_SUBTEST( SegmentDistanceTest );
	RUN_SUBTEST( ShapeDistanceTest );
	RUN_SUBTEST( ShapeCastTest );
	RUN_SUBTEST( ShapeCastEncroachRecedeTest );
	RUN_SUBTEST( ShapeCastEncroachSlideTest );
	RUN_SUBTEST( ShapeCastEncroachClosingTest );
	RUN_SUBTEST( ShapeCastEncroachGradedTest );
	RUN_SUBTEST( ShapeCastEncroachGrazeTest );
	RUN_SUBTEST( ShapeCastStartTouchingDefaultTest );
	RUN_SUBTEST( ShapeCastEncroachOverlapTest );
	RUN_SUBTEST( TimeOfImpactTest );

	return 0;
}

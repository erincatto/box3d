// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "mesh_drop.h"

#include "box3d/box3d.h"

#include <float.h>
#include <math.h>

#define WAVE_COUNT 40
#define WAVE_CELL_WIDTH 1.0f
#define WAVE_ROW_FREQUENCY 0.1f
#define WAVE_COLUMN_FREQUENCY 0.2f
#define WAVE_AMPLITUDE 0.5f

// Bumps about 20 cm across, close to the curvature of the bodies, on cells a few times smaller. The cell
// width is a power of two so the mesh builder steps to the same vertices the depth tracker computes.
#define ROUGH_WAVE_COUNT 640
#define ROUGH_CELL_WIDTH 0.03125f
#define ROUGH_ROW_FREQUENCY 4.7f
#define ROUGH_COLUMN_FREQUENCY 5.9f
#define ROUGH_AMPLITUDE 0.015f
#define DROP_SPACING 0.5f
#define DROP_HEIGHT 5.0f
#define BODY_RADIUS 0.05f
#define CAPSULE_HALF_LENGTH 0.2f

const char* g_meshDropShapeNames[e_meshDropShapeCount] = { "box", "capsule", "cylinder", "sphere" };

const float g_meshDropDepthLevels[MESH_DROP_DEPTH_LEVEL_COUNT] = { 4.0f, 8.0f, 16.0f, 32.0f };

// Same XorShift32 and mapping as the samples, but with local state so worlds can run on many threads
static float RandomFloatRange( uint32_t* state, float lo, float hi )
{
	uint32_t x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;

	int i = (int)( x % 32768 );
	float r = (float)( i & 32767 ) / 32767.0f;
	return ( hi - lo ) * r + lo;
}

static b3Vec3 RandomVec3Uniform( uint32_t* state, float lo, float hi )
{
	b3Vec3 v;
	v.x = RandomFloatRange( state, lo, hi );
	v.y = RandomFloatRange( state, lo, hi );
	v.z = RandomFloatRange( state, lo, hi );
	return v;
}

static uint32_t HashSeed( uint32_t x )
{
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x != 0 ? x : 1;
}

typedef struct WaveGrid
{
	int count;
	float cellWidth;
	float rowFrequency;
	float columnFrequency;
} WaveGrid;

static WaveGrid GetWaveGrid( bool rough )
{
	_Static_assert( ROUGH_WAVE_COUNT <= MESH_DROP_MAX_WAVE_COUNT && WAVE_COUNT <= MESH_DROP_MAX_WAVE_COUNT,
					"wave count exceeds the depth tracker tables" );

	if ( rough )
	{
		return (WaveGrid){ ROUGH_WAVE_COUNT, ROUGH_CELL_WIDTH, ROUGH_ROW_FREQUENCY, ROUGH_COLUMN_FREQUENCY };
	}

	return (WaveGrid){ WAVE_COUNT, WAVE_CELL_WIDTH, WAVE_ROW_FREQUENCY, WAVE_COLUMN_FREQUENCY };
}

static b3BoxHull MakeDropBox( void )
{
	// The soft contact solver cannot always resolve a smaller box at 4 sub-steps, and its size gets
	// close to the time of impact tolerances
	return b3MakeBoxHull( 0.02f, 0.2f, 0.04f );
}

MeshDropSettings GetDefaultMeshDropSettings( void )
{
	MeshDropSettings settings = {
		.shape = e_meshDropBox,
		.amplitude = WAVE_AMPLITUDE,
		.rough = false,
		.collide = false,
		.soloIndex = -1,
	};
	return settings;
}

float GetMeshDropDefaultAmplitude( bool rough )
{
	return rough ? ROUGH_AMPLITUDE : WAVE_AMPLITUDE;
}

uint32_t GetMeshDropSeed( uint32_t baseSeed, int generation )
{
	return HashSeed( HashSeed( baseSeed ) + (uint32_t)generation );
}

int GetMeshDropStepLimit( MeshDropShape shape )
{
	return shape == e_meshDropCapsule ? 3000 : 1000;
}

float GetMeshDropTunnelHeight( void )
{
	return -2.0f;
}

b3MeshData* CreateMeshDropMesh( MeshDropSettings settings )
{
	WaveGrid grid = GetWaveGrid( settings.rough );
	return b3CreateWaveMesh( grid.count, grid.count, grid.cellWidth, settings.amplitude, grid.rowFrequency,
							 grid.columnFrequency );
}

b3HullData* CreateMeshDropCylinder( void )
{
	return b3CreateCylinder( 0.4f, BODY_RADIUS, 0.0f, 6 );
}

b3BodyId CreateMeshDropGround( b3WorldId worldId, const b3MeshData* mesh, MeshDropSettings settings )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.filter.categoryBits = 1;
	b3CreateMeshShape( groundId, &shapeDef, mesh, b3Vec3_one );

	WaveGrid grid = GetWaveGrid( settings.rough );
	float extent = 0.5f * grid.count * grid.cellWidth;
	float halfHeight = 1.0f;

	b3Transform transforms[4] = {
		{ { 0.0f, halfHeight, -extent }, b3Quat_identity },
		{ { 0.0f, halfHeight, extent }, b3Quat_identity },
		{ { -extent, halfHeight, 0.0f }, b3Quat_identity },
		{ { extent, halfHeight, 0.0f }, b3Quat_identity },
	};

	for ( int i = 0; i < 4; ++i )
	{
		float hx = i < 2 ? extent : 0.1f;
		float hz = i < 2 ? 0.1f : extent;
		b3BoxHull wallBox = b3MakeTransformedBoxHull( hx, halfHeight, hz, transforms[i] );
		b3CreateHullShape( groundId, &shapeDef, &wallBox.base );
	}

	return groundId;
}

void CreateMeshDropBodies( b3WorldId worldId, MeshDropSettings settings, uint32_t seed, const b3HullData* cylinder,
						   b3BodyId* bodies )
{
	b3BoxHull box = MakeDropBox();
	b3Capsule capsule = { { 0.0f, -CAPSULE_HALF_LENGTH, 0.0f }, { 0.0f, CAPSULE_HALF_LENGTH, 0.0f }, BODY_RADIUS };
	b3Sphere sphere = { b3Vec3_zero, BODY_RADIUS };

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	bool round = settings.shape == e_meshDropCapsule || settings.shape == e_meshDropSphere;
	shapeDef.baseMaterial.rollingResistance = round ? 0.4f : 0.1f;

	if ( settings.collide == false )
	{
		shapeDef.filter.categoryBits = 2;
		shapeDef.filter.maskBits = 1;
	}

	uint32_t state = seed;
	int index = 0;
	for ( int i = 0; i < MESH_DROP_GRID_COUNT; ++i )
	{
		for ( int j = 0; j < MESH_DROP_GRID_COUNT; ++j )
		{
			// Draw for every body so a solo body gets the same velocity it has in the full drop
			b3Vec3 linearVelocity = RandomVec3Uniform( &state, -1.0f, 1.0f );
			b3Vec3 angularVelocity = RandomVec3Uniform( &state, -5.0f, 5.0f );

			if ( settings.soloIndex >= 0 && settings.soloIndex != index )
			{
				bodies[index] = b3_nullBodyId;
				index += 1;
				continue;
			}

			bodyDef.position = (b3Pos){ DROP_SPACING * ( i - 0.5f * MESH_DROP_GRID_COUNT ), DROP_HEIGHT,
										DROP_SPACING * ( j - 0.5f * MESH_DROP_GRID_COUNT ) };
			bodyDef.linearVelocity = linearVelocity;
			bodyDef.angularVelocity = angularVelocity;
			bodyDef.userData = (void*)(intptr_t)( index + 1 );
			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

			switch ( settings.shape )
			{
				case e_meshDropBox:
					b3CreateHullShape( bodyId, &shapeDef, &box.base );
					break;
				case e_meshDropCapsule:
					b3CreateCapsuleShape( bodyId, &shapeDef, &capsule );
					break;
				case e_meshDropCylinder:
					b3CreateHullShape( bodyId, &shapeDef, cylinder );
					break;
				default:
					b3CreateSphereShape( bodyId, &shapeDef, &sphere );
					break;
			}

			bodies[index] = bodyId;
			index += 1;
		}
	}
}

int GetMeshDropBodyIndex( void* userData )
{
	intptr_t value = (intptr_t)userData;
	return 1 <= value && value <= MESH_DROP_BODY_COUNT ? (int)value - 1 : -1;
}

void ResetMeshDropDepth( MeshDropDepth* depth, MeshDropSettings settings, const b3HullData* cylinder )
{
	// Same heights as b3CreateWaveMesh
	WaveGrid grid = GetWaveGrid( settings.rough );
	float omegaZ = 2.0f * B3_PI * grid.rowFrequency * grid.cellWidth;
	float omegaX = 2.0f * B3_PI * grid.columnFrequency * grid.cellWidth;
	for ( int i = 0; i <= grid.count; ++i )
	{
		depth->rowHeights[i] = b3ComputeCosSin( omegaX * i ).sine;
		depth->columnHeights[i] = b3ComputeCosSin( omegaZ * i ).sine;
	}

	depth->waveCount = grid.count;
	depth->cellWidth = grid.cellWidth;
	depth->waveAmplitude = settings.amplitude;
	depth->amplitude = fabsf( settings.amplitude );
	depth->probeCount = 0;
	depth->radius = 0.0f;

	switch ( settings.shape )
	{
		case e_meshDropBox:
		case e_meshDropCylinder:
		{
			b3BoxHull box = MakeDropBox();
			const b3HullData* hull = settings.shape == e_meshDropBox ? &box.base : cylinder;
			const b3Vec3* points = b3GetHullPoints( hull );
			int count = hull->vertexCount < MESH_DROP_MAX_PROBES ? hull->vertexCount : MESH_DROP_MAX_PROBES;
			for ( int i = 0; i < count; ++i )
			{
				depth->probes[i] = points[i];
			}
			depth->probeCount = count;
		}
		break;

		case e_meshDropCapsule:
			depth->probes[0] = (b3Vec3){ 0.0f, -CAPSULE_HALF_LENGTH, 0.0f };
			depth->probes[1] = (b3Vec3){ 0.0f, CAPSULE_HALF_LENGTH, 0.0f };
			depth->probeCount = 2;
			depth->radius = BODY_RADIUS;
			break;

		default:
			depth->probes[0] = b3Vec3_zero;
			depth->probeCount = 1;
			depth->radius = BODY_RADIUS;
			break;
	}

	float maxLength = 0.0f;
	for ( int i = 0; i < depth->probeCount; ++i )
	{
		maxLength = fmaxf( maxLength, b3Length( depth->probes[i] ) );
	}
	depth->boundingRadius = maxLength + depth->radius;

	for ( int i = 0; i < MESH_DROP_BODY_COUNT; ++i )
	{
		depth->bodyDepths[i] = -FLT_MAX;
	}

	depth->maxDepth = -FLT_MAX;
	depth->maxIndex = -1;
	depth->maxStep = 0;
	depth->stepCount = 0;
}

// Signed distance to the plane of the wave triangle under the point. Exact away from the folds, where
// it reads a little deep on a crest and shallow in a valley.
static float GetWaveDistance( const MeshDropDepth* depth, b3Vec3 p )
{
	float w = depth->cellWidth;
	float origin = -0.5f * depth->waveCount * w;
	float u = ( p.x - origin ) / w;
	float v = ( p.z - origin ) / w;
	int ix = b3ClampInt( (int)floorf( u ), 0, depth->waveCount - 1 );
	int iz = b3ClampInt( (int)floorf( v ), 0, depth->waveCount - 1 );

	float amplitude = depth->waveAmplitude;
	float ha = amplitude * depth->rowHeights[ix] * depth->columnHeights[iz];
	float hb = amplitude * depth->rowHeights[ix] * depth->columnHeights[iz + 1];
	float hc = amplitude * depth->rowHeights[ix + 1] * depth->columnHeights[iz + 1];
	float hd = amplitude * depth->rowHeights[ix + 1] * depth->columnHeights[iz];

	// The cell diagonal runs from a to c, matching b3CreateWaveMesh
	b3Vec3 a = { origin + ix * w, ha, origin + iz * w };
	b3Vec3 ac = { w, hc - ha, w };
	b3Vec3 side = v - iz >= u - ix ? (b3Vec3){ 0.0f, hb - ha, w } : (b3Vec3){ w, hd - ha, 0.0f };
	b3Vec3 normal = b3Normalize( b3Cross( side, ac ) );
	if ( normal.y < 0.0f )
	{
		normal = b3Neg( normal );
	}

	return b3Dot( normal, b3Sub( p, a ) );
}

void UpdateMeshDropDepth( b3WorldId worldId, MeshDropDepth* depth )
{
	depth->stepCount += 1;

	b3BodyEvents events = b3World_GetBodyEvents( worldId );
	for ( int i = 0; i < events.moveCount; ++i )
	{
		const b3BodyMoveEvent* event = events.moveEvents + i;
		int index = GetMeshDropBodyIndex( event->userData );
		if ( index < 0 )
		{
			continue;
		}

		b3WorldTransform transform = event->transform;
		if ( (float)transform.p.y - depth->boundingRadius > depth->amplitude )
		{
			continue;
		}

		float sink = -FLT_MAX;
		for ( int j = 0; j < depth->probeCount; ++j )
		{
			b3Pos point = b3TransformWorldPoint( transform, depth->probes[j] );
			b3Vec3 p = { (float)point.x, (float)point.y, (float)point.z };
			sink = fmaxf( sink, depth->radius - GetWaveDistance( depth, p ) );
		}

		if ( sink > depth->bodyDepths[index] )
		{
			depth->bodyDepths[index] = sink;
		}

		if ( sink > depth->maxDepth )
		{
			depth->maxDepth = sink;
			depth->maxIndex = index;
			depth->maxStep = depth->stepCount;
		}
	}
}

void GetMeshDropDepthCounts( const MeshDropDepth* depth, int counts[MESH_DROP_DEPTH_LEVEL_COUNT] )
{
	for ( int level = 0; level < MESH_DROP_DEPTH_LEVEL_COUNT; ++level )
	{
		float threshold = g_meshDropDepthLevels[level] * B3_LINEAR_SLOP;
		int count = 0;
		for ( int i = 0; i < MESH_DROP_BODY_COUNT; ++i )
		{
			count += depth->bodyDepths[i] > threshold ? 1 : 0;
		}
		counts[level] = count;
	}
}

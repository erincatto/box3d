// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "ghost_collisions.h"

#include "box3d/box3d.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GHOST_HALF_LENGTH 20.0f
#define GHOST_HALF_WIDTH 3.0f
#define GHOST_START_X -16.0f

// A 34 degree chamfer like the s&box beams
#define GHOST_CHAMFER_WIDTH 0.05f
#define GHOST_CHAMFER_DROP ( GHOST_CHAMFER_WIDTH / 1.5f )

// A conveyor side rail along x, the floor is on the -z side
#define GHOST_RAIL_HEIGHT 0.35f
#define GHOST_RAIL_THICKNESS 0.08f
#define GHOST_RAIL_SKIRT 0.4f

typedef struct PeelPreset
{
	int columnCount;
	float scaleX;
	float startZ;
} PeelPreset;

static const PeelPreset s_peelPresets[GHOST_PEEL_PRESET_COUNT] = {
	{ 4, 0.1f, -55.0f },
	{ 4, 0.1f, -54.0f },
	{ 4, 0.01f, -55.0f },
	{ 16, 0.01f, -54.0f },
};

const char* g_ghostSceneNames[e_ghostSceneCount] = {
	"Flat", "Concave", "Cliff", "Crest", "Bumps", "T-Junction", "Internal", "Chamfer", "Rail",
};

const char* g_ghostBodyNames[e_ghostBodyCount] = { "Box", "H Capsule", "V Capsule", "Sphere", "Cylinder" };

const char* g_ghostPeelPresetNames[GHOST_PEEL_PRESET_COUNT] = {
	"4.1:-55",
	"4.1:-54",
	"4.01:-55",
	"16.01:-54",
};

GhostSettings GetDefaultGhostSettings( void )
{
	GhostSettings settings = {
		.scene = e_ghostFlat,
		.body = e_ghostBox,
		.peelPreset = 0,
		.mirror = false,
		.heightField = false,
		.cellSize = 0.25f,
		.speed = 6.0f,
		.yaw = 0.0f,
		.friction = 0.0f,
		.fixedRotation = false,
		.speculative = true,
	};
	return settings;
}

typedef struct MeshBuilder
{
	b3Vec3* vertices;
	int vertexCount;
	int32_t* indices;
	int indexCount;
} MeshBuilder;

// The profile runs in +x with the solid below it. The mirror flips x in the data and the mesh
// scale flips it back, so the world geometry is the same either way.
static void AddGrid( MeshBuilder* builder, const b3Vec2* points, int pointCount, float halfWidth, int zCount, bool alternate,
					 bool mirror )
{
	float sign = mirror ? -1.0f : 1.0f;
	int stride = zCount + 1;
	int base = builder->vertexCount;

	builder->vertices = realloc( builder->vertices, ( builder->vertexCount + pointCount * stride ) * sizeof( b3Vec3 ) );
	for ( int i = 0; i < pointCount; ++i )
	{
		for ( int j = 0; j <= zCount; ++j )
		{
			float z = -halfWidth + 2.0f * halfWidth * (float)j / (float)zCount;
			builder->vertices[builder->vertexCount++] = (b3Vec3){ sign * points[i].x, points[i].y, z };
		}
	}

	int triangleCount = 2 * ( pointCount - 1 ) * zCount;
	builder->indices = realloc( builder->indices, ( builder->indexCount + 3 * triangleCount ) * sizeof( int32_t ) );
	for ( int i = 0; i + 1 < pointCount; ++i )
	{
		for ( int j = 0; j < zCount; ++j )
		{
			int a = base + i * stride + j;
			int b = a + 1;
			int c = a + stride + 1;
			int d = a + stride;

			int corners[6] = { a, b, c, c, d, a };

			// Alternate the diagonal like cooked level geometry
			if ( alternate && ( ( i + j ) & 1 ) )
			{
				int flipped[6] = { a, b, d, b, c, d };
				memcpy( corners, flipped, sizeof( corners ) );
			}

			for ( int k = 0; k < 6; k += 3 )
			{
				builder->indices[builder->indexCount++] = corners[k];
				builder->indices[builder->indexCount++] = mirror ? corners[k + 2] : corners[k + 1];
				builder->indices[builder->indexCount++] = mirror ? corners[k + 1] : corners[k + 2];
			}
		}
	}
}

static void AddStrip( MeshBuilder* builder, const b3Vec2* profile, int count, float cellSize, float zCell, bool mirror )
{
	int pointCount = 1;
	for ( int k = 0; k + 1 < count; ++k )
	{
		b3Vec2 a = profile[k];
		b3Vec2 b = profile[k + 1];
		float length = sqrtf( ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y ) );
		pointCount += b3MaxInt( 1, (int)ceilf( length / cellSize - 0.01f ) );
	}

	b3Vec2* points = malloc( pointCount * sizeof( b3Vec2 ) );
	int index = 0;
	points[index++] = profile[0];
	for ( int k = 0; k + 1 < count; ++k )
	{
		b3Vec2 a = profile[k];
		b3Vec2 b = profile[k + 1];
		float length = sqrtf( ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y ) );
		int n = b3MaxInt( 1, (int)ceilf( length / cellSize - 0.01f ) );
		for ( int s = 1; s <= n; ++s )
		{
			float t = (float)s / (float)n;
			points[index++] = (b3Vec2){ a.x + t * ( b.x - a.x ), a.y + t * ( b.y - a.y ) };
		}
	}

	int zCount = b3MaxInt( 1, (int)ceilf( 2.0f * GHOST_HALF_WIDTH / zCell - 0.01f ) );
	AddGrid( builder, points, pointCount, GHOST_HALF_WIDTH, zCount, true, mirror );
	free( points );
}

// The rail cross section is a profile in zy extruded along x. Each quad side is its edge direction along
// the section crossed with x, which is the outward normal for a section traversed with air on the left.
static void AddRail( MeshBuilder* builder, const b3Vec2* section, int count, float cellSize, bool mirror )
{
	float sign = mirror ? -1.0f : 1.0f;
	int xCount = b3MaxInt( 1, (int)ceilf( 2.0f * GHOST_HALF_LENGTH / cellSize - 0.01f ) );
	int stride = xCount + 1;

	int pointCount = 1;
	for ( int k = 0; k + 1 < count; ++k )
	{
		float dz = section[k + 1].x - section[k].x;
		float dy = section[k + 1].y - section[k].y;
		pointCount += b3MaxInt( 1, (int)ceilf( sqrtf( dz * dz + dy * dy ) / cellSize - 0.01f ) );
	}

	builder->vertices = realloc( builder->vertices, pointCount * stride * sizeof( b3Vec3 ) );
	int index = 0;
	for ( int k = 0; k < count; ++k )
	{
		int n = 1;
		if ( k + 1 < count )
		{
			float dz = section[k + 1].x - section[k].x;
			float dy = section[k + 1].y - section[k].y;
			n = b3MaxInt( 1, (int)ceilf( sqrtf( dz * dz + dy * dy ) / cellSize - 0.01f ) );
		}

		// The last point only closes the final segment
		int last = k + 1 < count ? n : 1;
		for ( int s = 0; s < last; ++s )
		{
			float t = k + 1 < count ? (float)s / (float)n : 0.0f;
			b3Vec2 a = section[k];
			b3Vec2 b = k + 1 < count ? section[k + 1] : a;
			float z = a.x + t * ( b.x - a.x );
			float y = a.y + t * ( b.y - a.y );
			for ( int j = 0; j <= xCount; ++j )
			{
				float x = -GHOST_HALF_LENGTH + 2.0f * GHOST_HALF_LENGTH * (float)j / (float)xCount;
				builder->vertices[index++] = (b3Vec3){ sign * x, y, z };
			}
		}
	}

	builder->vertexCount = index;

	int triangleCount = 2 * ( pointCount - 1 ) * xCount;
	builder->indices = realloc( builder->indices, 3 * triangleCount * sizeof( int32_t ) );
	builder->indexCount = 0;
	for ( int i = 0; i + 1 < pointCount; ++i )
	{
		for ( int j = 0; j < xCount; ++j )
		{
			int a = i * stride + j;
			int b = a + 1;
			int c = a + stride + 1;
			int d = a + stride;

			int corners[6] = { a, d, c, c, b, a };
			if ( ( i + j ) & 1 )
			{
				int flipped[6] = { a, d, b, d, c, b };
				memcpy( corners, flipped, sizeof( corners ) );
			}

			for ( int k = 0; k < 6; k += 3 )
			{
				builder->indices[builder->indexCount++] = corners[k];
				builder->indices[builder->indexCount++] = mirror ? corners[k + 2] : corners[k + 1];
				builder->indices[builder->indexCount++] = mirror ? corners[k + 1] : corners[k + 2];
			}
		}
	}
}

static float EvaluateRail( float z, float cellZ )
{
	if ( z < -0.5f * cellZ )
	{
		return 0.0f;
	}

	return z < 0.5f * cellZ ? GHOST_RAIL_HEIGHT : -GHOST_RAIL_SKIRT;
}

static void AddMesh( GhostCollisionsData* data, MeshBuilder* builder, bool frictionless )
{
	b3MeshDef meshDef = { 0 };
	meshDef.vertices = builder->vertices;
	meshDef.indices = builder->indices;
	meshDef.vertexCount = builder->vertexCount;
	meshDef.triangleCount = builder->indexCount / 3;
	meshDef.weldVertices = true;
	meshDef.weldTolerance = 0.001f;
	meshDef.identifyEdges = true;

	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );
	data->meshes[data->meshCount++] = mesh;

	b3Vec3 scale = data->settings.mirror ? (b3Vec3){ -1.0f, 1.0f, 1.0f } : b3Vec3_one;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	if ( frictionless )
	{
		shapeDef.baseMaterial.friction = 0.0f;
	}
	b3CreateMeshShape( data->groundId, &shapeDef, mesh, scale );

	builder->vertexCount = 0;
	builder->indexCount = 0;
}

// The bodies holding height field shapes must go first
static void DestroyGhostGroundData( GhostCollisionsData* data )
{
	for ( int i = 0; i < data->meshCount; ++i )
	{
		b3DestroyMesh( data->meshes[i] );
		data->meshes[i] = NULL;
	}
	data->meshCount = 0;

	for ( int i = 0; i < data->heightFieldCount; ++i )
	{
		b3DestroyHeightField( data->heightFields[i] );
		data->heightFields[i] = NULL;
	}
	data->heightFieldCount = 0;
}

static void PushProfile( GhostCollisionsData* data, float x, float y )
{
	if ( data->profileCount < GHOST_MAX_PROFILE_POINTS )
	{
		data->profile[data->profileCount++] = (b3Vec2){ x, y };
	}
}

// The profile is a function of x, a vertical step has two points at the same x
static float EvaluateProfile( const GhostCollisionsData* data, float x )
{
	const b3Vec2* profile = data->profile;
	int count = data->profileCount;
	for ( int k = 0; k + 1 < count; ++k )
	{
		if ( x <= profile[k + 1].x )
		{
			float dx = profile[k + 1].x - profile[k].x;
			float t = dx > 0.0f ? b3ClampFloat( ( x - profile[k].x ) / dx, 0.0f, 1.0f ) : 1.0f;
			return profile[k].y + t * ( profile[k + 1].y - profile[k].y );
		}
	}

	return profile[count - 1].y;
}

static int GetCellCount( float length, float cellSize )
{
	return b3MaxInt( 1, (int)ceilf( length / cellSize - 0.01f ) );
}

// Samples the profile on a grid spanning x0 to x1. The mirror samples in reverse and turns the body half
// way around y, so the world geometry is the same either way. All tiles of a scene share one height range
// so their quantized heights line up at a seam.
static void AddHeightField( b3WorldId worldId, GhostCollisionsData* data, float x0, float x1, int cellCountX, float halfWidth,
							int cellCountZ, bool frictionless )
{
	float minHeight = FLT_MAX;
	float maxHeight = -FLT_MAX;
	for ( int k = 0; k < data->profileCount; ++k )
	{
		minHeight = b3MinFloat( minHeight, data->profile[k].y );
		maxHeight = b3MaxFloat( maxHeight, data->profile[k].y );
	}

	bool mirror = data->settings.mirror;
	int countX = cellCountX + 1;
	int countZ = cellCountZ + 1;
	float cellX = ( x1 - x0 ) / (float)cellCountX;
	float cellZ = 2.0f * halfWidth / (float)cellCountZ;

	// The mirror turns the body half way around y, which also reverses z
	bool rail = data->settings.scene == e_ghostRail;
	float* heights = malloc( countX * countZ * sizeof( float ) );
	for ( int row = 0; row < countZ; ++row )
	{
		float z = mirror ? halfWidth - cellZ * (float)row : -halfWidth + cellZ * (float)row;
		for ( int column = 0; column < countX; ++column )
		{
			float x = mirror ? x1 - cellX * (float)column : x0 + cellX * (float)column;
			heights[row * countX + column] = rail ? EvaluateRail( z, cellZ ) : EvaluateProfile( data, x );
		}
	}

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.scale = (b3Vec3){ cellX, 1.0f, cellZ };
	def.countX = countX;
	def.countZ = countZ;
	def.globalMinimumHeight = minHeight;
	def.globalMaximumHeight = maxHeight;
	b3HeightFieldData* heightField = b3CreateHeightField( &def );
	free( heights );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	if ( mirror )
	{
		bodyDef.position = (b3Pos){ x1, 0.0f, halfWidth };
		bodyDef.rotation = (b3Quat){ { 0.0f, 1.0f, 0.0f }, 0.0f };
	}
	else
	{
		bodyDef.position = (b3Pos){ x0, 0.0f, -halfWidth };
	}

	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	if ( frictionless )
	{
		shapeDef.baseMaterial.friction = 0.0f;
	}
	b3CreateHeightFieldShape( bodyId, &shapeDef, heightField );

	data->heightFields[data->heightFieldCount] = heightField;
	data->heightBodyIds[data->heightFieldCount] = bodyId;
	data->heightFieldCount += 1;
}

void CreateGhostGround( b3WorldId worldId, GhostCollisionsData* data )
{
	if ( B3_IS_NON_NULL( data->groundId ) )
	{
		b3DestroyBody( data->groundId );
	}

	for ( int i = 0; i < data->heightFieldCount; ++i )
	{
		b3DestroyBody( data->heightBodyIds[i] );
	}

	DestroyGhostGroundData( data );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	data->groundId = b3CreateBody( worldId, &bodyDef );

	const GhostSettings* settings = &data->settings;
	float h = GHOST_HALF_LENGTH;
	data->profileCount = 0;
	data->halfWidth = GHOST_HALF_WIDTH;

	switch ( settings->scene )
	{
		case e_ghostConcave:
		{
			float slope = 8.0f * B3_PI / 180.0f;
			PushProfile( data, -h, h * tanf( slope ) );
			PushProfile( data, 0.0f, 0.0f );
			PushProfile( data, h, 0.0f );
		}
		break;

		case e_ghostCliff:
			// A height field cannot hold a vertical wall, so there the drop takes one cell
			PushProfile( data, -h, 3.0f );
			PushProfile( data, 0.0f, 3.0f );
			PushProfile( data, settings->heightField ? settings->cellSize : 0.0f, 0.0f );
			PushProfile( data, h, 0.0f );
			break;

		case e_ghostCrest:
			PushProfile( data, -h, 0.0f );
			PushProfile( data, -10.0f, 0.0f );
			PushProfile( data, 0.0f, 1.0f );
			PushProfile( data, 10.0f, 0.0f );
			PushProfile( data, h, 0.0f );
			break;

		case e_ghostBumps:
			// Gentle folds of a few degrees, the range that only flat edge suppression misses
			for ( int i = 0; i <= 80; ++i )
			{
				float x = -h + 0.5f * (float)i;
				float y = x < -10.0f ? 0.0f : 0.05f * ( 1.0f - cosf( B3_PI * x ) );
				PushProfile( data, x, y );
			}
			break;

		case e_ghostInternal:
			// PEEL's curved mesh: 32 rows 4 m apart, each step up 1.25 times the last, sliding toward +x
			for ( int i = 31; i >= 0; --i )
			{
				PushProfile( data, 62.0f - 4.0f * (float)i, 1.0f + 0.1f * powf( 1.25f, (float)i ) );
			}
			break;

		case e_ghostChamfer:
			// Flat tops at y = 0 with a chamfered notch every meter. A body crossing a notch overhangs both
			// chamfers while its bottom rests on the convex top edges.
			PushProfile( data, -h, 0.0f );
			for ( int i = 0; i < 30; ++i )
			{
				float x = -8.0f + (float)i;
				PushProfile( data, x, 0.0f );
				PushProfile( data, x + GHOST_CHAMFER_WIDTH, -GHOST_CHAMFER_DROP );
				PushProfile( data, x + 2.0f * GHOST_CHAMFER_WIDTH, -GHOST_CHAMFER_DROP );
				PushProfile( data, x + 3.0f * GHOST_CHAMFER_WIDTH, 0.0f );
			}
			PushProfile( data, h, 0.0f );
			break;

		case e_ghostRail:
			// Cross section in zy, from the floor up the inner wall, over the top and down the outer skirt
			PushProfile( data, -GHOST_HALF_WIDTH, 0.0f );
			PushProfile( data, 0.0f, 0.0f );
			PushProfile( data, 0.0f, GHOST_RAIL_HEIGHT );
			PushProfile( data, GHOST_RAIL_THICKNESS, GHOST_RAIL_HEIGHT );
			PushProfile( data, GHOST_RAIL_THICKNESS, -GHOST_RAIL_SKIRT );
			break;

		default:
			PushProfile( data, -h, 0.0f );
			PushProfile( data, h, 0.0f );
			break;
	}

	MeshBuilder builder = { 0 };
	b3Vec2 left[2] = { { -h, 0.0f }, { 0.0f, 0.0f } };
	b3Vec2 right[2] = { { 0.0f, 0.0f }, { h, 0.0f } };
	float cellSize = settings->cellSize;
	bool mirror = settings->mirror;
	float groundHalfWidth = GHOST_HALF_WIDTH;
	int cellCountZ = GetCellCount( 2.0f * groundHalfWidth, cellSize );

	if ( settings->heightField )
	{
		if ( settings->scene == e_ghostTJunction )
		{
			AddHeightField( worldId, data, -h, 0.0f, GetCellCount( h, cellSize ), groundHalfWidth,
							GetCellCount( 2.0f * groundHalfWidth, 0.5f * cellSize ), false );
			AddHeightField( worldId, data, 0.0f, h, GetCellCount( h, cellSize ), groundHalfWidth, cellCountZ, false );
		}
		else if ( settings->scene == e_ghostInternal )
		{
			// One grid line per profile row, as in the mesh
			PeelPreset preset = s_peelPresets[settings->peelPreset];
			int zCount = preset.columnCount - 1;
			data->halfWidth = 20.0f * preset.scaleX * (float)zCount;
			float x0 = data->profile[0].x;
			float x1 = data->profile[data->profileCount - 1].x;
			AddHeightField( worldId, data, x0, x1, data->profileCount - 1, data->halfWidth, zCount, true );

			b3BoxHull box = b3MakeOffsetBoxHull( 140.0f, 1.0f, 30.0f, ( b3Vec3 ){ 65.0f, -1.0f, 0.0f } );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3CreateHullShape( data->groundId, &shapeDef, &box.base );
		}
		else if ( settings->scene == e_ghostChamfer )
		{
			// One cell per chamfer puts every notch vertex on the grid
			AddHeightField( worldId, data, -h, h, GetCellCount( 2.0f * h, GHOST_CHAMFER_WIDTH ), groundHalfWidth, cellCountZ,
							false );
		}
		else
		{
			AddHeightField( worldId, data, -h, h, GetCellCount( 2.0f * h, cellSize ), groundHalfWidth, cellCountZ, false );
		}
	}
	else if ( settings->scene == e_ghostRail )
	{
		AddRail( &builder, data->profile, data->profileCount, cellSize, mirror );
		AddMesh( data, &builder, false );
	}
	else if ( settings->scene == e_ghostTJunction )
	{
		// Fine and coarse strips share only every other vertex along x = 0
		AddStrip( &builder, left, 2, cellSize, 0.5f * cellSize, mirror );
		AddStrip( &builder, right, 2, cellSize, cellSize, mirror );
		AddMesh( data, &builder, false );
	}
	else if ( settings->scene == e_ghostInternal )
	{
		// The mesh has no friction and the ground plane under it does, as in PEEL
		PeelPreset preset = s_peelPresets[settings->peelPreset];
		int zCount = preset.columnCount - 1;
		float halfWidth = 20.0f * preset.scaleX * (float)zCount;
		AddGrid( &builder, data->profile, data->profileCount, halfWidth, zCount, false, mirror );
		data->halfWidth = halfWidth;
		AddMesh( data, &builder, true );

		b3BoxHull box = b3MakeOffsetBoxHull( 140.0f, 1.0f, 30.0f, ( b3Vec3 ){ 65.0f, -1.0f, 0.0f } );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( data->groundId, &shapeDef, &box.base );
	}
	else
	{
		AddStrip( &builder, data->profile, data->profileCount, cellSize, cellSize, mirror );
		AddMesh( data, &builder, false );
	}

	free( builder.vertices );
	free( builder.indices );
}

void LaunchGhostBody( b3WorldId worldId, GhostCollisionsData* data )
{
	if ( B3_IS_NON_NULL( data->bodyId ) )
	{
		b3DestroyBody( data->bodyId );
	}

	const GhostSettings* settings = &data->settings;

	// PEEL drops its bodies from rest, sized in meters rather than tenths
	bool peel = settings->scene == e_ghostInternal;
	float boxHalf = peel ? 1.0f : 0.2f;
	float capsuleRadius = peel ? 1.0f : 0.2f;
	float capsuleHalf = peel ? 2.0f : 0.4f;
	float sphereRadius = peel ? 1.0f : 0.25f;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableContactRecycling = false;
	b3Quat yaw = b3MakeQuatFromAxisAngle( b3Vec3_axisY, settings->yaw * B3_PI / 180.0f );

	if ( peel )
	{
		bodyDef.position = (b3Pos){ s_peelPresets[settings->peelPreset].startZ, 80.0f, 0.0f };
		bodyDef.rotation = yaw;
	}
	else if ( settings->scene == e_ghostRail )
	{
		// Dropped across the rail top, pitched so a box face sweeps into the rail edge at an angle. The real
		// contact is edge on edge and perpendicular to the rail, a face normal there leans along the rail
		// and catches the body. Found in the Conveyor Circuit sample.
		b3Quat pitch = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 60.0f * B3_PI / 180.0f );
		bodyDef.position = (b3Pos){ GHOST_START_X, GHOST_RAIL_HEIGHT + 0.45f, -0.15f };
		bodyDef.rotation = b3MulQuat( pitch, yaw );
		bodyDef.linearVelocity = (b3Vec3){ settings->speed, 0.0f, 0.0f };
	}
	else
	{
		int segment = 0;
		for ( int k = 0; k + 1 < data->profileCount; ++k )
		{
			if ( data->profile[k].x <= GHOST_START_X && GHOST_START_X < data->profile[k + 1].x )
			{
				segment = k;
				break;
			}
		}

		b3Vec2 a = data->profile[segment];
		b3Vec2 b = data->profile[segment + 1];
		float length = sqrtf( ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y ) );
		b3Vec2 d = { ( b.x - a.x ) / length, ( b.y - a.y ) / length };
		b3Vec2 n = { -d.y, d.x };
		float y = a.y + ( GHOST_START_X - a.x ) * d.y / d.x;

		float lift = boxHalf;
		if ( settings->body == e_ghostSphere )
		{
			lift = sphereRadius;
		}
		else if ( settings->body == e_ghostVCapsule )
		{
			lift = capsuleHalf + capsuleRadius;
		}
		lift += 0.01f;

		bodyDef.position = (b3Pos){ GHOST_START_X + lift * n.x, y + lift * n.y, 0.0f };
		b3Quat slope = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, atan2f( d.y, d.x ) );
		bodyDef.rotation = b3MulQuat( slope, yaw );
		bodyDef.linearVelocity = (b3Vec3){ settings->speed * d.x, settings->speed * d.y, 0.0f };
	}

	bodyDef.motionLocks.angularX = settings->fixedRotation;
	bodyDef.motionLocks.angularY = settings->fixedRotation;
	bodyDef.motionLocks.angularZ = settings->fixedRotation;
	bodyDef.enableSleep = false;
	bodyDef.name = "slider";
	data->bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = settings->friction;
	shapeDef.enableSpeculativeContact = settings->speculative;

	switch ( settings->body )
	{
		case e_ghostBox:
		{
			b3BoxHull box = peel ? b3MakeBoxHull( 1.0f, 1.0f, 1.0f ) : b3MakeBoxHull( 0.4f, 0.2f, 0.4f );
			data->shapeId = b3CreateHullShape( data->bodyId, &shapeDef, &box.base );
		}
		break;

		case e_ghostHCapsule:
		{
			b3Capsule capsule = { { 0.0f, 0.0f, -capsuleHalf }, { 0.0f, 0.0f, capsuleHalf }, capsuleRadius };
			data->shapeId = b3CreateCapsuleShape( data->bodyId, &shapeDef, &capsule );
		}
		break;

		case e_ghostVCapsule:
		{
			b3Capsule capsule = { { 0.0f, -capsuleHalf, 0.0f }, { 0.0f, capsuleHalf, 0.0f }, capsuleRadius };
			data->shapeId = b3CreateCapsuleShape( data->bodyId, &shapeDef, &capsule );
		}
		break;

		case e_ghostSphere:
		{
			b3Sphere sphere = { b3Vec3_zero, sphereRadius };
			data->shapeId = b3CreateSphereShape( data->bodyId, &shapeDef, &sphere );
		}
		break;

		default:
			data->shapeId = b3CreateHullShape( data->bodyId, &shapeDef, peel ? data->peelCylinder : data->cylinder );
			break;
	}

	data->contactBounds = b3Shape_GetAABB( data->shapeId );
	data->runGhostCount = 0;
	data->runSteps = 0;
}

GhostCollisionsData CreateGhostCollisions( b3WorldId worldId, GhostSettings settings )
{
	GhostCollisionsData data = { 0 };
	data.settings = settings;
	data.cylinder = b3CreateCylinder( 0.4f, 0.4f, -0.2f, 16 );
	data.peelCylinder = b3CreateCylinder( 2.0f, 1.0f, -1.0f, 16 );
	data.groundId = b3_nullBodyId;
	data.bodyId = b3_nullBodyId;
	data.shapeId = b3_nullShapeId;

	CreateGhostGround( worldId, &data );
	LaunchGhostBody( worldId, &data );
	return data;
}

static float SegmentAngle( const GhostCollisionsData* data, int k )
{
	b3Vec2 a = data->profile[k];
	b3Vec2 b = data->profile[k + 1];
	return atan2f( b.x - a.x, a.y - b.y );
}

// The normal points from the ground to the body. A hull face can be the reference face, so any
// normal within the span of face normals under the body is valid. At a side edge of the ground the
// normal may also lean off that side, perpendicular to the edge.
static bool IsGhost( const GhostCollisionsData* data, b3Vec3 normal, float xMin, float xMax, float zMin, float zMax )
{
	float tolerance = 5.0f * B3_PI / 180.0f;
	float sinTolerance = sinf( tolerance );
	if ( b3AbsFloat( normal.z ) > sinTolerance )
	{
		// The manifold must reach the side it leans off
		float margin = 0.05f;
		bool atSide = normal.z < 0.0f ? zMin < -data->halfWidth + margin : zMax > data->halfWidth - margin;
		if ( atSide == false )
		{
			return true;
		}
	}

	// A normal leaning off a side has a short projection, which leaves little room to lean along the edge
	float planarLength = sqrtf( normal.x * normal.x + normal.y * normal.y );
	if ( planarLength < sinTolerance )
	{
		return false;
	}

	GhostBody body = data->settings.body;
	bool chamferHull = data->settings.scene == e_ghostChamfer && ( body == e_ghostBox || body == e_ghostCylinder );

	float lower = B3_PI;
	float upper = -B3_PI;
	int segmentCount = data->profileCount - 1;
	for ( int k = 0; k < segmentCount; ++k )
	{
		float x0 = b3MinFloat( data->profile[k].x, data->profile[k + 1].x );
		float x1 = b3MaxFloat( data->profile[k].x, data->profile[k + 1].x );
		if ( x1 < xMin || xMax < x0 )
		{
			continue;
		}

		// A flat hull bottom resting on the top edges must keep its own normal, so for a hull only the tops
		// count. A round body's normal comes from the closest ground feature, which can be a top edge.
		if ( chamferHull && ( data->profile[k].y < 0.0f || data->profile[k + 1].y < 0.0f ) )
		{
			continue;
		}

		float angle = SegmentAngle( data, k );
		lower = b3MinFloat( lower, angle );
		upper = b3MaxFloat( upper, angle );
	}

	float angle = atan2f( normal.y, normal.x );
	float deviation = b3MaxFloat( lower - angle, angle - upper );
	if ( deviation <= 0.0f )
	{
		return false;
	}

	// Measure the lean outside the span in 3D so a steep side normal is not over counted
	return deviation > 0.5f * B3_PI || planarLength * sinf( deviation ) > sinTolerance;
}

int UpdateGhostCollisions( b3WorldId worldId, GhostCollisionsData* data, GhostReport* reports, int capacity )
{
	b3Pos center = b3Body_GetWorldCenter( data->bodyId );
	bool done = center.x > GHOST_HALF_LENGTH - 1.0f || center.y < -3.0f;
	if ( data->settings.scene == e_ghostInternal )
	{
		// Run out onto the ground plane, or give up on a body knocked off the side
		done = center.x > 100.0f || center.y < -3.0f || data->runSteps > 900;
	}

	if ( done )
	{
		data->runCount += 1;
		LaunchGhostBody( worldId, data );
		center = b3Body_GetWorldCenter( data->bodyId );
	}

	data->runSteps += 1;

	b3AABB bounds = data->contactBounds;
	float xMin = (float)bounds.lowerBound.x - 0.1f;
	float xMax = (float)bounds.upperBound.x + 0.1f;
	data->contactBounds = b3Shape_GetAABB( data->shapeId );

	int ghostCount = 0;
	b3ContactData contacts[8];
	int contactCount = b3Body_GetContactData( data->bodyId, contacts, 8 );
	for ( int i = 0; i < contactCount; ++i )
	{
		bool bodyIsA = B3_ID_EQUALS( contacts[i].shapeIdA, data->shapeId );
		b3ShapeId otherId = bodyIsA ? contacts[i].shapeIdB : contacts[i].shapeIdA;
		b3ShapeType otherType = b3Shape_GetType( otherId );
		if ( otherType != b3_meshShape && otherType != b3_heightShape )
		{
			continue;
		}

		for ( int j = 0; j < contacts[i].manifoldCount; ++j )
		{
			const b3Manifold* manifold = contacts[i].manifolds + j;

			// Speculative points that never pushed are harmless
			float impulse = 0.0f;
			for ( int k = 0; k < manifold->pointCount; ++k )
			{
				impulse += manifold->points[k].totalNormalImpulse;
			}

			if ( manifold->pointCount == 0 || impulse <= 0.0f )
			{
				continue;
			}

			float zMin = FLT_MAX;
			float zMax = -FLT_MAX;
			for ( int k = 0; k < manifold->pointCount; ++k )
			{
				b3Vec3 anchor = bodyIsA ? manifold->points[k].anchorA : manifold->points[k].anchorB;
				float z = (float)center.z + anchor.z;
				zMin = b3MinFloat( zMin, z );
				zMax = b3MaxFloat( zMax, z );
			}

			b3Vec3 normal = bodyIsA ? b3Neg( manifold->normal ) : manifold->normal;
			if ( data->settings.scene == e_ghostRail )
			{
				// The rail is the same all along x, so no valid normal leans along it
				if ( b3AbsFloat( normal.x ) <= sinf( 5.0f * B3_PI / 180.0f ) )
				{
					continue;
				}
			}
			else if ( IsGhost( data, normal, xMin, xMax, zMin, zMax ) == false )
			{
				continue;
			}

			if ( ghostCount < capacity )
			{
				b3Vec3 anchor = bodyIsA ? manifold->points[0].anchorA : manifold->points[0].anchorB;
				reports[ghostCount] = (GhostReport){
					.point = b3OffsetPos( center, anchor ),
					.normal = normal,
					.separation = manifold->points[0].separation,
					.impulse = impulse,
				};
			}

			ghostCount += 1;
		}
	}

	data->runGhostCount += ghostCount;
	data->ghostCount += ghostCount;
	return ghostCount;
}

void DestroyGhostCollisions( GhostCollisionsData* data )
{
	DestroyGhostGroundData( data );
	b3DestroyHull( data->cylinder );
	b3DestroyHull( data->peelCylinder );
	data->cylinder = NULL;
	data->peelCylinder = NULL;
}

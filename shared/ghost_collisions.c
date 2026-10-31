// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "ghost_collisions.h"

#include "box3d/box3d.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define GHOST_HALF_LENGTH 20.0f
#define GHOST_HALF_WIDTH 3.0f
#define GHOST_START_X -16.0f

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
	"Flat", "Concave", "Cliff", "Crest", "Bumps", "T-Junction", "Seam", "Internal",
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

static void DestroyGhostMeshes( GhostCollisionsData* data )
{
	for ( int i = 0; i < data->meshCount; ++i )
	{
		b3DestroyMesh( data->meshes[i] );
		data->meshes[i] = NULL;
	}
	data->meshCount = 0;
}

static void PushProfile( GhostCollisionsData* data, float x, float y )
{
	if ( data->profileCount < GHOST_MAX_PROFILE_POINTS )
	{
		data->profile[data->profileCount++] = (b3Vec2){ x, y };
	}
}

void CreateGhostGround( b3WorldId worldId, GhostCollisionsData* data )
{
	if ( B3_IS_NON_NULL( data->groundId ) )
	{
		b3DestroyBody( data->groundId );
	}
	DestroyGhostMeshes( data );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	data->groundId = b3CreateBody( worldId, &bodyDef );

	const GhostSettings* settings = &data->settings;
	float h = GHOST_HALF_LENGTH;
	data->profileCount = 0;

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
			PushProfile( data, -h, 3.0f );
			PushProfile( data, 0.0f, 3.0f );
			PushProfile( data, 0.0f, 0.0f );
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

	if ( settings->scene == e_ghostTJunction )
	{
		// Fine and coarse strips share only every other vertex along x = 0
		AddStrip( &builder, left, 2, cellSize, 0.5f * cellSize, mirror );
		AddStrip( &builder, right, 2, cellSize, cellSize, mirror );
		AddMesh( data, &builder, false );
	}
	else if ( settings->scene == e_ghostSeam )
	{
		// Two mesh shapes meeting at x = 0, so the seam spans two contacts
		AddStrip( &builder, left, 2, cellSize, cellSize, mirror );
		AddMesh( data, &builder, false );
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
	b3Quat yaw = b3MakeQuatFromAxisAngle( b3Vec3_axisY, settings->yaw * B3_PI / 180.0f );

	if ( peel )
	{
		bodyDef.position = (b3Pos){ s_peelPresets[settings->peelPreset].startZ, 80.0f, 0.0f };
		bodyDef.rotation = yaw;
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
// normal within the span of face normals under the body is valid.
static bool IsGhost( const GhostCollisionsData* data, b3Vec3 normal, float xMin, float xMax )
{
	float tolerance = 5.0f * B3_PI / 180.0f;
	if ( b3AbsFloat( normal.z ) > sinf( tolerance ) )
	{
		return true;
	}

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

		float angle = SegmentAngle( data, k );
		lower = b3MinFloat( lower, angle );
		upper = b3MaxFloat( upper, angle );
	}

	float angle = atan2f( normal.y, normal.x );
	return angle < lower - tolerance || upper + tolerance < angle;
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
		if ( b3Shape_GetType( otherId ) != b3_meshShape )
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

			b3Vec3 normal = bodyIsA ? b3Neg( manifold->normal ) : manifold->normal;
			if ( IsGhost( data, normal, xMin, xMax ) == false )
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
	DestroyGhostMeshes( data );
	b3DestroyHull( data->cylinder );
	b3DestroyHull( data->peelCylinder );
	data->cylinder = NULL;
	data->peelCylinder = NULL;
}

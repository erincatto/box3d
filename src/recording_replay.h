// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b3RecPlayer b3RecPlayer;

// A single recorded callback hit, used both as reader scratch during a query replay and as the
// per-frame draw store. For collide-mover, one hit is one plane, with planeCount and userReturnB
// replicated across a shape's planes so the replay walker can re-group and compare per shape.
typedef struct b3RecRecordedHit
{
	b3ShapeId     id;
	b3Pos         point;
	b3Vec3        normal;
	float         fraction;
	uint64_t      userMaterialId;
	int           triangleIndex;
	int           childIndex;
	b3PlaneResult plane;       // collide-mover: this plane
	int           planeCount;  // collide-mover: planes in this hit's shape group (replicated)
	float         userReturnF; // cast queries
	bool          userReturnB; // overlap / collide-mover (per shape, replicated)
} b3RecRecordedHit;

// Recorded query kind, matching the public b3RecQueryType order.
typedef enum b3RecQueryKind
{
	B3_RECQ_OVERLAP_AABB,
	B3_RECQ_OVERLAP_SHAPE,
	B3_RECQ_CAST_RAY,
	B3_RECQ_CAST_SHAPE,
	B3_RECQ_CAST_RAY_CLOSEST,
	B3_RECQ_CAST_MOVER,
	B3_RECQ_COLLIDE_MOVER,
} b3RecQueryKind;

// Per-frame draw record for one query call. Self-contained (no aliased pointers) so the player's
// frameQueries array can grow with a plain memcpy. Geometry is origin relative except the overlap
// AABB, which is world space in 3D.
typedef struct b3RecDrawQuery
{
	int           kind;
	b3QueryFilter filter;
	b3AABB        aabb;                              // overlap AABB, world space
	b3Vec3        proxyPoints[B3_MAX_SHAPE_CAST_POINTS]; // overlap/cast shape proxy, origin relative
	int           proxyCount;
	float         proxyRadius;
	b3Capsule     mover;                             // cast/collide mover, origin relative
	b3Pos         origin;
	b3Vec3        translation;
	float         castFraction;                      // cast-mover result fraction
	b3RayResult   rayResult;                         // cast-ray-closest result
	b3ShapeId     shape;
	int           hitStart;                          // first hit in the player's frameHits store
	int           hitCount;
} b3RecDrawQuery;

// One slot in the preloaded geometry registry. Loaded from the trailing block before any
// ops run; live pointer built lazily on first shape create that references the id.
typedef struct b3RegistrySlot
{
	b3GeometryKind kind;
	int            byteCount;
	uint8_t*       bytes; // raw bytes from the file (always freed at teardown)
	void*          live;  // reconstructed live object, freed after b3DestroyWorld
} b3RegistrySlot;

// Reader state threaded through the replay loop and all dispatch functions
typedef struct b3RecReader
{
	const uint8_t* data;
	int            size;
	int            cursor;
	b3WorldId      replayWorldId;
	bool           ok;       // false on read overrun or id mismatch, fatal stop
	bool           diverged; // a StateHash failed, non-fatal

	// Player that owns this reader, or NULL during a headless b3ValidateReplay. Body
	// create/destroy and the bounds record fold back into it for the outliner and camera framing.
	b3RecPlayer*   owner;

	// Scratch for per-triangle materials in shape defs (grown on demand, freed at teardown)
	b3SurfaceMaterial* matScratch;
	int                matScratchCap;

	// Scratch for string reads: rotating slots, valid until the next 4 STR reads
	char strBufs[4][B3_NAME_LENGTH + 1];
	int  strNext;

	// Preloaded geometry registry
	b3RegistrySlot* slots;
	int             slotCount;

	// Scratch for recorded query hits; grown on demand, freed with the player.
	b3RecRecordedHit* hits;
	int               hitCap;

	// Scratch for a shape-proxy point cloud read from the stream. b3ShapeProxy holds the points
	// behind a pointer, so a decoded proxy borrows this until the next proxy read or teardown.
	b3Vec3* proxyScratch;
	int     proxyScratchCap;
} b3RecReader;

// Read primitives
uint8_t          b3RecR_U8( b3RecReader* rdr );
uint16_t         b3RecR_U16( b3RecReader* rdr );
uint32_t         b3RecR_U24( b3RecReader* rdr );
uint32_t         b3RecR_U32( b3RecReader* rdr );
uint64_t         b3RecR_U64( b3RecReader* rdr );
int32_t          b3RecR_I32( b3RecReader* rdr );
float            b3RecR_F32( b3RecReader* rdr );
double           b3RecR_F64( b3RecReader* rdr );
bool             b3RecR_BOOL( b3RecReader* rdr );
b3Vec3           b3RecR_VEC3( b3RecReader* rdr );
b3Quat           b3RecR_QUAT( b3RecReader* rdr );
b3Transform      b3RecR_TRANSFORM( b3RecReader* rdr );
b3Pos            b3RecR_POSITION( b3RecReader* rdr );
b3WorldTransform b3RecR_WORLDXF( b3RecReader* rdr );
b3Matrix3        b3RecR_MATRIX3( b3RecReader* rdr );
b3AABB           b3RecR_AABB( b3RecReader* rdr );
b3WorldId        b3RecR_WORLDID( b3RecReader* rdr );
b3BodyId         b3RecR_BODYID( b3RecReader* rdr );
b3ShapeId        b3RecR_SHAPEID( b3RecReader* rdr );
b3JointId        b3RecR_JOINTID( b3RecReader* rdr );
b3Sphere         b3RecR_SPHERE( b3RecReader* rdr );
b3Capsule        b3RecR_CAPSULE( b3RecReader* rdr );
uint32_t         b3RecR_GEOMID( b3RecReader* rdr );
b3Filter         b3RecR_FILTER( b3RecReader* rdr );
b3SurfaceMaterial b3RecR_MATERIAL( b3RecReader* rdr );
b3MassData       b3RecR_MASSDATA( b3RecReader* rdr );
b3MotionLocks    b3RecR_LOCKS( b3RecReader* rdr );
const char*      b3RecR_STR( b3RecReader* rdr );
b3ExplosionDef   b3RecR_EXPLOSIONDEF( b3RecReader* rdr );
b3BodyDef        b3RecR_BODYDEF( b3RecReader* rdr );
b3ShapeDef       b3RecR_SHAPEDEF( b3RecReader* rdr );
b3ParallelJointDef  b3RecR_PARALLELJOINTDEF( b3RecReader* rdr );
b3DistanceJointDef  b3RecR_DISTANCEJOINTDEF( b3RecReader* rdr );
b3FilterJointDef    b3RecR_FILTERJOINTDEF( b3RecReader* rdr );
b3MotorJointDef     b3RecR_MOTORJOINTDEF( b3RecReader* rdr );
b3PrismaticJointDef b3RecR_PRISMATICJOINTDEF( b3RecReader* rdr );
b3RevoluteJointDef  b3RecR_REVOLUTEJOINTDEF( b3RecReader* rdr );
b3SphericalJointDef b3RecR_SPHERICALJOINTDEF( b3RecReader* rdr );
b3WeldJointDef      b3RecR_WELDJOINTDEF( b3RecReader* rdr );
b3WheelJointDef     b3RecR_WHEELJOINTDEF( b3RecReader* rdr );
b3QueryFilter       b3RecR_QUERYFILTER( b3RecReader* rdr );
b3ShapeProxy        b3RecR_SHAPEPROXY( b3RecReader* rdr );
b3TreeStats         b3RecR_TREESTATS( b3RecReader* rdr );
b3RayResult         b3RecR_RAYRESULT( b3RecReader* rdr );
b3PlaneResult       b3RecR_PLANERESULT( b3RecReader* rdr );

// Grow the reader's hit scratch to at least n entries, preserving contents. n is bounded by the
// file size since every recorded hit consumes at least one byte.
void b3RecEnsureHits( b3RecReader* rdr, int n );

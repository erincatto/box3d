// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b3RecPlayer b3RecPlayer;

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

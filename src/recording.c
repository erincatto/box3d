// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "recording.h"

#include "body.h"
#include "compound.h"
#include "physics_world.h"
#include "world_snapshot.h"

#include "box3d/box3d.h"
#include "box3d/constants.h"

#include <limits.h>
#include <stddef.h>

// Buffer helpers

void b3RecBufAppend( b3RecBuffer* buf, const void* data, int size )
{
	if ( size <= 0 )
	{
		return;
	}

	if ( buf->countOnly )
	{
		buf->size += size;
		return;
	}

	if ( buf->size + size > buf->capacity )
	{
		int newCap = buf->capacity * 2;
		if ( newCap < buf->size + size + 64 )
		{
			newCap = buf->size + size + 64;
		}
		if ( buf->data == NULL )
		{
			buf->data = b3Alloc( (size_t)newCap );
		}
		else
		{
			buf->data = b3GrowAlloc( buf->data, buf->capacity, newCap );
		}
		buf->capacity = newCap;
	}

	memcpy( buf->data + buf->size, data, (size_t)size );
	buf->size += size;
}

void b3RecBufFree( b3RecBuffer* buf )
{
	if ( buf->data != NULL )
	{
		b3Free( buf->data, (size_t)buf->capacity );
		buf->data = NULL;
		buf->capacity = 0;
		buf->size = 0;
	}
}

// Write primitives

void b3RecW_U8( b3RecBuffer* buf, uint8_t v )
{
	b3RecBufAppend( buf, &v, 1 );
}

void b3RecW_U16( b3RecBuffer* buf, uint16_t v )
{
	uint8_t b[2] = { (uint8_t)v, (uint8_t)( v >> 8 ) };
	b3RecBufAppend( buf, b, 2 );
}

void b3RecW_U32( b3RecBuffer* buf, uint32_t v )
{
	uint8_t b[4] = { (uint8_t)v, (uint8_t)( v >> 8 ), (uint8_t)( v >> 16 ), (uint8_t)( v >> 24 ) };
	b3RecBufAppend( buf, b, 4 );
}

void b3RecW_U64( b3RecBuffer* buf, uint64_t v )
{
	uint8_t b[8] = { (uint8_t)v,          (uint8_t)( v >> 8 ),  (uint8_t)( v >> 16 ), (uint8_t)( v >> 24 ),
	                 (uint8_t)( v >> 32 ), (uint8_t)( v >> 40 ), (uint8_t)( v >> 48 ), (uint8_t)( v >> 56 ) };
	b3RecBufAppend( buf, b, 8 );
}

void b3RecW_I32( b3RecBuffer* buf, int32_t v )
{
	b3RecW_U32( buf, (uint32_t)v );
}

void b3RecW_F32( b3RecBuffer* buf, float v )
{
	uint32_t bits;
	memcpy( &bits, &v, 4 );
	b3RecW_U32( buf, bits );
}

void b3RecW_F64( b3RecBuffer* buf, double v )
{
	uint64_t bits;
	memcpy( &bits, &v, 8 );
	b3RecW_U64( buf, bits );
}

void b3RecW_BOOL( b3RecBuffer* buf, bool v )
{
	b3RecW_U8( buf, v ? 1u : 0u );
}

void b3RecW_VEC3( b3RecBuffer* buf, b3Vec3 v )
{
	b3RecW_F32( buf, v.x );
	b3RecW_F32( buf, v.y );
	b3RecW_F32( buf, v.z );
}

void b3RecW_QUAT( b3RecBuffer* buf, b3Quat v )
{
	b3RecW_F32( buf, v.v.x );
	b3RecW_F32( buf, v.v.y );
	b3RecW_F32( buf, v.v.z );
	b3RecW_F32( buf, v.s );
}

void b3RecW_TRANSFORM( b3RecBuffer* buf, b3Transform v )
{
	b3RecW_VEC3( buf, v.p );
	b3RecW_QUAT( buf, v.q );
}

// World position at full precision so recordings reproduce the simulation far from the origin.
// In the float build this is three floats, wire-identical to VEC3.
void b3RecW_POSITION( b3RecBuffer* buf, b3Pos v )
{
#if defined( BOX3D_DOUBLE_PRECISION )
	b3RecW_F64( buf, v.x );
	b3RecW_F64( buf, v.y );
	b3RecW_F64( buf, v.z );
#else
	b3RecW_F32( buf, v.x );
	b3RecW_F32( buf, v.y );
	b3RecW_F32( buf, v.z );
#endif
}

void b3RecW_WORLDXF( b3RecBuffer* buf, b3WorldTransform v )
{
	b3RecW_POSITION( buf, v.p );
	b3RecW_QUAT( buf, v.q );
}

void b3RecW_MATRIX3( b3RecBuffer* buf, b3Matrix3 v )
{
	b3RecW_VEC3( buf, v.cx );
	b3RecW_VEC3( buf, v.cy );
	b3RecW_VEC3( buf, v.cz );
}

void b3RecW_AABB( b3RecBuffer* buf, b3AABB v )
{
	b3RecW_VEC3( buf, v.lowerBound );
	b3RecW_VEC3( buf, v.upperBound );
}

void b3RecW_QUERYFILTER( b3RecBuffer* buf, b3QueryFilter v )
{
	b3RecW_U64( buf, v.categoryBits );
	b3RecW_U64( buf, v.maskBits );
}

// Variable length: count, then count points, then radius. The point cloud lives behind a pointer so
// it cannot ride along as POD.
void b3RecW_SHAPEPROXY( b3RecBuffer* buf, b3ShapeProxy v )
{
	int count = v.count;
	if ( count < 0 )
		count = 0;
	if ( count > B3_MAX_SHAPE_CAST_POINTS )
		count = B3_MAX_SHAPE_CAST_POINTS;
	b3RecW_I32( buf, count );
	for ( int i = 0; i < count; ++i )
	{
		b3RecW_VEC3( buf, v.points[i] );
	}
	b3RecW_F32( buf, v.radius );
}

void b3RecW_TREESTATS( b3RecBuffer* buf, b3TreeStats v )
{
	b3RecW_I32( buf, v.nodeVisits );
	b3RecW_I32( buf, v.leafVisits );
}

void b3RecW_RAYRESULT( b3RecBuffer* buf, b3RayResult v )
{
	b3RecW_SHAPEID( buf, v.shapeId );
	b3RecW_POSITION( buf, v.point );
	b3RecW_VEC3( buf, v.normal );
	b3RecW_U64( buf, v.userMaterialId );
	b3RecW_F32( buf, v.fraction );
	b3RecW_I32( buf, v.triangleIndex );
	b3RecW_I32( buf, v.childIndex );
	b3RecW_BOOL( buf, v.hit );
}

void b3RecW_PLANERESULT( b3RecBuffer* buf, b3PlaneResult v )
{
	b3RecW_VEC3( buf, v.plane.normal );
	b3RecW_F32( buf, v.plane.offset );
	b3RecW_VEC3( buf, v.point );
}

void b3RecW_WORLDID( b3RecBuffer* buf, b3WorldId v )
{
	b3RecW_U32( buf, b3StoreWorldId( v ) );
}

void b3RecW_BODYID( b3RecBuffer* buf, b3BodyId v )
{
	b3RecW_U64( buf, b3StoreBodyId( v ) );
}

void b3RecW_SHAPEID( b3RecBuffer* buf, b3ShapeId v )
{
	b3RecW_U64( buf, b3StoreShapeId( v ) );
}

void b3RecW_JOINTID( b3RecBuffer* buf, b3JointId v )
{
	b3RecW_U64( buf, b3StoreJointId( v ) );
}

// Pointer-free POD; pointerWidth in the header gates the layout on replay
void b3RecW_SPHERE( b3RecBuffer* buf, b3Sphere v )
{
	b3RecBufAppend( buf, &v, (int)sizeof( b3Sphere ) );
}

void b3RecW_CAPSULE( b3RecBuffer* buf, b3Capsule v )
{
	b3RecBufAppend( buf, &v, (int)sizeof( b3Capsule ) );
}

void b3RecW_GEOMID( b3RecBuffer* buf, uint32_t v )
{
	b3RecW_U32( buf, v );
}

void b3RecW_FILTER( b3RecBuffer* buf, b3Filter v )
{
	b3RecW_U64( buf, v.categoryBits );
	b3RecW_U64( buf, v.maskBits );
	b3RecW_I32( buf, v.groupIndex );
}

void b3RecW_MATERIAL( b3RecBuffer* buf, b3SurfaceMaterial v )
{
	b3RecW_F32( buf, v.friction );
	b3RecW_F32( buf, v.restitution );
	b3RecW_F32( buf, v.rollingResistance );
	b3RecW_VEC3( buf, v.tangentVelocity );
	b3RecW_U64( buf, v.userMaterialId );
	b3RecW_U32( buf, v.customColor );
}

void b3RecW_MASSDATA( b3RecBuffer* buf, b3MassData v )
{
	b3RecW_F32( buf, v.mass );
	b3RecW_VEC3( buf, v.center );
	b3RecW_MATRIX3( buf, v.inertia );
}

void b3RecW_LOCKS( b3RecBuffer* buf, b3MotionLocks v )
{
	b3RecW_BOOL( buf, v.linearX );
	b3RecW_BOOL( buf, v.linearY );
	b3RecW_BOOL( buf, v.linearZ );
	b3RecW_BOOL( buf, v.angularX );
	b3RecW_BOOL( buf, v.angularY );
	b3RecW_BOOL( buf, v.angularZ );
}

void b3RecW_STR( b3RecBuffer* buf, const char* s )
{
	if ( s == NULL )
	{
		b3RecW_U16( buf, 0xFFFFu );
		return;
	}
	int len = 0;
	while ( s[len] != '\0' && len < 65534 )
	{
		len++;
	}
	b3RecW_U16( buf, (uint16_t)len );
	if ( len > 0 )
	{
		b3RecBufAppend( buf, s, len );
	}
}

// Hand-written def helpers. Zero pointer and cookie fields before serializing.
// Readers call b3Default*Def() first to get the cookie, then overwrite fields.

void b3RecW_EXPLOSIONDEF( b3RecBuffer* buf, b3ExplosionDef v )
{
	b3RecW_U64( buf, v.maskBits );
	b3RecW_POSITION( buf, v.position );
	b3RecW_F32( buf, v.radius );
	b3RecW_F32( buf, v.falloff );
	b3RecW_F32( buf, v.impulsePerArea );
}

void b3RecW_BODYDEF( b3RecBuffer* buf, b3BodyDef v )
{
	b3RecW_I32( buf, (int32_t)v.type );
	b3RecW_POSITION( buf, v.position );
	b3RecW_QUAT( buf, v.rotation );
	b3RecW_VEC3( buf, v.linearVelocity );
	b3RecW_VEC3( buf, v.angularVelocity );
	b3RecW_F32( buf, v.linearDamping );
	b3RecW_F32( buf, v.angularDamping );
	b3RecW_F32( buf, v.gravityScale );
	b3RecW_F32( buf, v.sleepThreshold );
	b3RecW_STR( buf, v.name );
	// userData: not preserved
	b3RecW_U64( buf, 0u );
	b3RecW_LOCKS( buf, v.motionLocks );
	b3RecW_BOOL( buf, v.enableSleep );
	b3RecW_BOOL( buf, v.isAwake );
	b3RecW_BOOL( buf, v.isBullet );
	b3RecW_BOOL( buf, v.isEnabled );
	b3RecW_BOOL( buf, v.allowFastRotation );
	b3RecW_BOOL( buf, v.enableContactRecycling );
	// internalValue omitted
}

void b3RecW_SHAPEDEF( b3RecBuffer* buf, b3ShapeDef v )
{
	b3RecW_STR( buf, v.name );
	// userData: not preserved
	b3RecW_U64( buf, 0u );
	// Per-triangle materials: length-prefixed so the reader can rebuild the array.
	// Guard NULL so a default def (materialCount=0, materials=NULL) round-trips cleanly.
	int matCount = ( v.materials != NULL ) ? v.materialCount : 0;
	b3RecW_I32( buf, matCount );
	for ( int i = 0; i < matCount; ++i )
	{
		b3RecW_MATERIAL( buf, v.materials[i] );
	}
	b3RecW_MATERIAL( buf, v.baseMaterial );
	b3RecW_F32( buf, v.density );
	b3RecW_F32( buf, v.explosionScale );
	b3RecW_FILTER( buf, v.filter );
	b3RecW_BOOL( buf, v.enableCustomFiltering );
	b3RecW_BOOL( buf, v.isSensor );
	b3RecW_BOOL( buf, v.enableSensorEvents );
	b3RecW_BOOL( buf, v.enableContactEvents );
	b3RecW_BOOL( buf, v.enableHitEvents );
	b3RecW_BOOL( buf, v.enablePreSolveEvents );
	b3RecW_BOOL( buf, v.invokeContactCreation );
	b3RecW_BOOL( buf, v.updateBodyMass );
	// internalValue omitted
}

// Joint defs share a base. Body ids are written as packed ids for replay remapping.
static void b3RecW_JointBase( b3RecBuffer* buf, const b3JointDef* base )
{
	// userData: not preserved
	b3RecW_U64( buf, 0u );
	b3RecW_BODYID( buf, base->bodyIdA );
	b3RecW_BODYID( buf, base->bodyIdB );
	b3RecW_TRANSFORM( buf, base->localFrameA );
	b3RecW_TRANSFORM( buf, base->localFrameB );
	b3RecW_F32( buf, base->forceThreshold );
	b3RecW_F32( buf, base->torqueThreshold );
	b3RecW_F32( buf, base->constraintHertz );
	b3RecW_F32( buf, base->constraintDampingRatio );
	b3RecW_F32( buf, base->drawScale );
	b3RecW_BOOL( buf, base->collideConnected );
	// internalValue omitted
}

void b3RecW_PARALLELJOINTDEF( b3RecBuffer* buf, b3ParallelJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_F32( buf, v.hertz );
	b3RecW_F32( buf, v.dampingRatio );
	b3RecW_F32( buf, v.maxTorque );
}

void b3RecW_DISTANCEJOINTDEF( b3RecBuffer* buf, b3DistanceJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_F32( buf, v.length );
	b3RecW_BOOL( buf, v.enableSpring );
	b3RecW_F32( buf, v.lowerSpringForce );
	b3RecW_F32( buf, v.upperSpringForce );
	b3RecW_F32( buf, v.hertz );
	b3RecW_F32( buf, v.dampingRatio );
	b3RecW_BOOL( buf, v.enableLimit );
	b3RecW_F32( buf, v.minLength );
	b3RecW_F32( buf, v.maxLength );
	b3RecW_BOOL( buf, v.enableMotor );
	b3RecW_F32( buf, v.maxMotorForce );
	b3RecW_F32( buf, v.motorSpeed );
}

void b3RecW_FILTERJOINTDEF( b3RecBuffer* buf, b3FilterJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
}

void b3RecW_MOTORJOINTDEF( b3RecBuffer* buf, b3MotorJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_VEC3( buf, v.linearVelocity );
	b3RecW_F32( buf, v.maxVelocityForce );
	b3RecW_VEC3( buf, v.angularVelocity );
	b3RecW_F32( buf, v.maxVelocityTorque );
	b3RecW_F32( buf, v.linearHertz );
	b3RecW_F32( buf, v.linearDampingRatio );
	b3RecW_F32( buf, v.maxSpringForce );
	b3RecW_F32( buf, v.angularHertz );
	b3RecW_F32( buf, v.angularDampingRatio );
	b3RecW_F32( buf, v.maxSpringTorque );
}

void b3RecW_PRISMATICJOINTDEF( b3RecBuffer* buf, b3PrismaticJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_BOOL( buf, v.enableSpring );
	b3RecW_F32( buf, v.hertz );
	b3RecW_F32( buf, v.dampingRatio );
	b3RecW_F32( buf, v.targetTranslation );
	b3RecW_BOOL( buf, v.enableLimit );
	b3RecW_F32( buf, v.lowerTranslation );
	b3RecW_F32( buf, v.upperTranslation );
	b3RecW_BOOL( buf, v.enableMotor );
	b3RecW_F32( buf, v.maxMotorForce );
	b3RecW_F32( buf, v.motorSpeed );
}

void b3RecW_REVOLUTEJOINTDEF( b3RecBuffer* buf, b3RevoluteJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_F32( buf, v.targetAngle );
	b3RecW_BOOL( buf, v.enableSpring );
	b3RecW_F32( buf, v.hertz );
	b3RecW_F32( buf, v.dampingRatio );
	b3RecW_BOOL( buf, v.enableLimit );
	b3RecW_F32( buf, v.lowerAngle );
	b3RecW_F32( buf, v.upperAngle );
	b3RecW_BOOL( buf, v.enableMotor );
	b3RecW_F32( buf, v.maxMotorTorque );
	b3RecW_F32( buf, v.motorSpeed );
}

void b3RecW_SPHERICALJOINTDEF( b3RecBuffer* buf, b3SphericalJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_BOOL( buf, v.enableSpring );
	b3RecW_F32( buf, v.hertz );
	b3RecW_F32( buf, v.dampingRatio );
	b3RecW_QUAT( buf, v.targetRotation );
	b3RecW_BOOL( buf, v.enableConeLimit );
	b3RecW_F32( buf, v.coneAngle );
	b3RecW_BOOL( buf, v.enableTwistLimit );
	b3RecW_F32( buf, v.lowerTwistAngle );
	b3RecW_F32( buf, v.upperTwistAngle );
	b3RecW_BOOL( buf, v.enableMotor );
	b3RecW_F32( buf, v.maxMotorTorque );
	b3RecW_VEC3( buf, v.motorVelocity );
}

void b3RecW_WELDJOINTDEF( b3RecBuffer* buf, b3WeldJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_F32( buf, v.linearHertz );
	b3RecW_F32( buf, v.angularHertz );
	b3RecW_F32( buf, v.linearDampingRatio );
	b3RecW_F32( buf, v.angularDampingRatio );
}

void b3RecW_WHEELJOINTDEF( b3RecBuffer* buf, b3WheelJointDef v )
{
	b3RecW_JointBase( buf, &v.base );
	b3RecW_BOOL( buf, v.enableSuspensionSpring );
	b3RecW_F32( buf, v.suspensionHertz );
	b3RecW_F32( buf, v.suspensionDampingRatio );
	b3RecW_BOOL( buf, v.enableSuspensionLimit );
	b3RecW_F32( buf, v.lowerSuspensionLimit );
	b3RecW_F32( buf, v.upperSuspensionLimit );
	b3RecW_BOOL( buf, v.enableSpinMotor );
	b3RecW_F32( buf, v.maxSpinTorque );
	b3RecW_F32( buf, v.spinSpeed );
	b3RecW_BOOL( buf, v.enableSteering );
	b3RecW_F32( buf, v.steeringHertz );
	b3RecW_F32( buf, v.steeringDampingRatio );
	b3RecW_F32( buf, v.targetSteeringAngle );
	b3RecW_F32( buf, v.maxSteeringTorque );
	b3RecW_BOOL( buf, v.enableSteeringLimit );
	b3RecW_F32( buf, v.lowerSteeringLimit );
	b3RecW_F32( buf, v.upperSteeringLimit );
}

// Query recording. A query collects a variable number of hits through a user callback, so the count
// is not known until the callback stops firing. The record is built in a local buffer with a
// reserved hit-count slot, then committed whole under the lock so concurrent query threads never
// interleave records in the shared buffer.

int b3RecReserveU32( b3RecBuffer* buf )
{
	int offset = buf->size;
	uint8_t zero[4] = { 0, 0, 0, 0 };
	b3RecBufAppend( buf, zero, 4 );
	return offset;
}

void b3RecPatchU32( b3RecBuffer* buf, int offset, uint32_t v )
{
	B3_ASSERT( offset >= 0 && offset + 4 <= buf->size );
	uint8_t* p = buf->data + offset;
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)( v >> 8 );
	p[2] = (uint8_t)( v >> 16 );
	p[3] = (uint8_t)( v >> 24 );
}

void b3RecCommitRecord( b3Recording* rec, uint8_t opcode, const uint8_t* payload, int payloadSize )
{
	B3_ASSERT( payloadSize >= 0 && payloadSize < ( 1 << 24 ) );
	b3LockMutex( rec->lock );
	b3RecW_U8( &rec->buffer, opcode );
	uint8_t sz[3] = { (uint8_t)payloadSize, (uint8_t)( payloadSize >> 8 ), (uint8_t)( payloadSize >> 16 ) };
	b3RecBufAppend( &rec->buffer, sz, 3 );
	b3RecBufAppend( &rec->buffer, payload, payloadSize );
	b3UnlockMutex( rec->lock );
}

void b3RecQueryBegin( b3RecQueryWriter* w, void* context )
{
	w->buf = ( b3RecBuffer ){ 0 };
	w->userFcn.overlapFcn = NULL;
	w->userContext = context;
	w->hitCount = 0;
	w->countOffset = 0;
}

void b3RecQueryCommit( b3Recording* rec, uint8_t opcode, b3RecQueryWriter* w )
{
	b3RecCommitRecord( rec, opcode, w->buf.data, w->buf.size );
	b3RecBufFree( &w->buf );
}

bool b3RecOverlapTrampoline( b3ShapeId id, void* ctx )
{
	b3RecQueryWriter* w = (b3RecQueryWriter*)ctx;
	// The user fcn is NULL for an unfiltered mover cast: accept all, still record the decision so
	// replay reproduces the same per-shape accept stream.
	bool ret = w->userFcn.overlapFcn != NULL ? w->userFcn.overlapFcn( id, w->userContext ) : true;
	b3RecW_SHAPEID( &w->buf, id );
	b3RecW_BOOL( &w->buf, ret );
	w->hitCount++;
	return ret;
}

float b3RecCastTrampoline( b3ShapeId id, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId, int triangleIndex,
						   int childIndex, void* ctx )
{
	b3RecQueryWriter* w = (b3RecQueryWriter*)ctx;
	float ret = w->userFcn.castFcn( id, point, normal, fraction, userMaterialId, triangleIndex, childIndex, w->userContext );
	b3RecW_SHAPEID( &w->buf, id );
	b3RecW_POSITION( &w->buf, point );
	b3RecW_VEC3( &w->buf, normal );
	b3RecW_F32( &w->buf, fraction );
	b3RecW_U64( &w->buf, userMaterialId );
	b3RecW_I32( &w->buf, triangleIndex );
	b3RecW_I32( &w->buf, childIndex );
	b3RecW_F32( &w->buf, ret );
	w->hitCount++;
	return ret;
}

// 3D delivers every plane for one shape in a single call. Record the shape, its plane count, each
// plane, then the user return so replay reproduces the same per-shape batch. One hit per shape.
bool b3RecPlaneTrampoline( b3ShapeId id, const b3PlaneResult* planes, int planeCount, void* ctx )
{
	b3RecQueryWriter* w = (b3RecQueryWriter*)ctx;
	bool ret = w->userFcn.planeFcn( id, planes, planeCount, w->userContext );
	b3RecW_SHAPEID( &w->buf, id );
	b3RecW_I32( &w->buf, planeCount );
	for ( int i = 0; i < planeCount; ++i )
	{
		b3RecW_PLANERESULT( &w->buf, planes[i] );
	}
	b3RecW_BOOL( &w->buf, ret );
	w->hitCount++;
	return ret;
}

// Record framing

void b3RecBeginRecord( b3Recording* rec, uint8_t opcode )
{
	b3RecW_U8( &rec->buffer, opcode );
	rec->recordStart = rec->buffer.size;
	// Reserve 3 bytes for the u24 payload size, backpatched in b3RecEndRecord.
	uint8_t zero[3] = { 0, 0, 0 };
	b3RecBufAppend( &rec->buffer, zero, 3 );
}

void b3RecEndRecord( b3Recording* rec )
{
	int payloadSize = rec->buffer.size - rec->recordStart - 3;
	B3_ASSERT( payloadSize >= 0 && payloadSize < ( 1 << 24 ) );
	uint8_t* p = rec->buffer.data + rec->recordStart;
	p[0] = (uint8_t)payloadSize;
	p[1] = (uint8_t)( payloadSize >> 8 );
	p[2] = (uint8_t)( payloadSize >> 16 );
}

// Codegen pass 1b: arg writers
#define ARG( TAG, field ) b3RecW_##TAG( &rec->buffer, a->field );
#define B3_REC_OP( op, Name, RET, ... )                                                                                          \
	void b3RecWriteArgs_##Name( b3Recording* rec, const b3RecArgs_##Name* a )                                                    \
	{                                                                                                                            \
		__VA_ARGS__                                                                                                              \
	}
#include "recording_ops.inl"
#undef B3_REC_OP
#undef ARG

// Codegen: full writers
#define B3_REC_OP( op, Name, RET, ... )                                                                                          \
	void b3RecWrite_##Name( b3Recording* rec, const b3RecArgs_##Name* a )                                                        \
	{                                                                                                                            \
		b3RecBeginRecord( rec, (uint8_t)( op ) );                                                                                \
		b3RecWriteArgs_##Name( rec, a );                                                                                         \
		b3RecEndRecord( rec );                                                                                                   \
	}
#include "recording_ops.inl"
#undef B3_REC_OP

// Codegen: create-op writers that append the returned id inside the record
#define B3_REC_RETWRITE( op, Name, idType, idW )                                                                                 \
	void b3RecWriteRet_##Name( b3Recording* rec, const b3RecArgs_##Name* a, idType id )                                          \
	{                                                                                                                            \
		b3RecBeginRecord( rec, (uint8_t)( op ) );                                                                                \
		b3RecWriteArgs_##Name( rec, a );                                                                                         \
		idW( &rec->buffer, id );                                                                                                 \
		b3RecEndRecord( rec );                                                                                                   \
	}
#define B3_REC_RETWRITE_RET_NONE( op, Name )
#define B3_REC_RETWRITE_RET_BODYID( op, Name )  B3_REC_RETWRITE( op, Name, b3BodyId,  b3RecW_BODYID )
#define B3_REC_RETWRITE_RET_SHAPEID( op, Name ) B3_REC_RETWRITE( op, Name, b3ShapeId, b3RecW_SHAPEID )
#define B3_REC_RETWRITE_RET_JOINTID( op, Name ) B3_REC_RETWRITE( op, Name, b3JointId, b3RecW_JOINTID )
#define B3_REC_OP( op, Name, RET, ... ) B3_REC_RETWRITE_##RET( op, Name )
#include "recording_ops.inl"
#undef B3_REC_OP
#undef B3_REC_RETWRITE_RET_NONE
#undef B3_REC_RETWRITE_RET_BODYID
#undef B3_REC_RETWRITE_RET_SHAPEID
#undef B3_REC_RETWRITE_RET_JOINTID
#undef B3_REC_RETWRITE

// Geometry registry

// Fold a 32-bit hash into 64 bits. Collisions are caught by the byteCount+memcmp fallback.
uint64_t b3Hash64Blob( const uint8_t* bytes, int n )
{
	uint32_t h = b3Hash( B3_HASH_INIT, bytes, n );
	return (uint64_t)h | ( (uint64_t)b3NonZeroHash( ~h ) << 32 );
}

uint32_t b3InternGeometry( b3GeometryRegistry* reg, b3GeometryKind kind, uint64_t contentHash,
                           uint8_t* bytes, int byteCount )
{
	// Linear scan for a matching blob (TODO: replace with a hash map)
	for ( int i = 0; i < reg->count; ++i )
	{
		b3GeometryEntry* e = reg->entries + i;
		if ( e->contentHash == contentHash && e->byteCount == byteCount && memcmp( e->bytes, bytes, (size_t)byteCount ) == 0 )
		{
			// Duplicate: the caller transferred ownership; return existing id
			b3Free( bytes, (size_t)byteCount );
			return e->id;
		}
	}

	// New entry — grow the array if needed
	if ( reg->count >= reg->capacity )
	{
		int newCap = reg->capacity < 8 ? 8 : reg->capacity * 2;
		reg->entries = (b3GeometryEntry*)b3GrowAlloc( reg->entries,
		                                               reg->capacity * (int)sizeof( b3GeometryEntry ),
		                                               newCap * (int)sizeof( b3GeometryEntry ) );
		reg->capacity = newCap;
	}

	uint32_t id = (uint32_t)reg->count;
	b3GeometryEntry* entry = reg->entries + reg->count;
	entry->contentHash = contentHash;
	entry->id          = id;
	entry->kind        = kind;
	entry->byteCount   = byteCount;
	entry->bytes       = bytes; // take ownership
	reg->count++;
	return id;
}

void b3FreeRegistry( b3GeometryRegistry* reg )
{
	for ( int i = 0; i < reg->count; ++i )
	{
		b3Free( reg->entries[i].bytes, (size_t)reg->entries[i].byteCount );
	}
	if ( reg->entries != NULL )
	{
		b3Free( reg->entries, (size_t)( reg->capacity * (int)sizeof( b3GeometryEntry ) ) );
	}
	reg->entries  = NULL;
	reg->count    = 0;
	reg->capacity = 0;
}

// Write the trailing registry block: u32 entryCount then per-entry { u8 kind, u32 byteCount, bytes }.
void b3RecWriteRegistry( b3Recording* rec )
{
	b3RecW_U32( &rec->buffer, (uint32_t)rec->registry.count );
	for ( int i = 0; i < rec->registry.count; ++i )
	{
		b3GeometryEntry* e = rec->registry.entries + i;
		b3RecW_U8( &rec->buffer, (uint8_t)e->kind );
		b3RecW_U32( &rec->buffer, (uint32_t)e->byteCount );
		b3RecBufAppend( &rec->buffer, e->bytes, e->byteCount );
	}
}

// HeightField codec. Derives array sizes from the scalar block exactly as height_field.c does.
// Pack layout: [scalar block: offsetof(aabb)..end of struct][heights u16][materials u8][flags u8]
uint8_t* b3PackHeightField( const b3HeightField* hf, int* outByteCount )
{
	int heightCount  = hf->columnCount * hf->rowCount;
	int cellCount    = ( hf->columnCount - 1 ) * ( hf->rowCount - 1 );
	int triangleCount = 2 * cellCount;

	int scalarSize   = (int)( sizeof( b3HeightField ) - offsetof( b3HeightField, aabb ) );
	int heightBytes  = heightCount * (int)sizeof( uint16_t );
	int matBytes     = cellCount * (int)sizeof( uint8_t );
	int flagBytes    = triangleCount * (int)sizeof( uint8_t );
	int total        = scalarSize + heightBytes + matBytes + flagBytes;

	uint8_t* buf = (uint8_t*)b3Alloc( (size_t)total );

	// Scalar block (hash field is included as stored: it will be re-asserted on unpack)
	memcpy( buf, (const uint8_t*)hf + offsetof( b3HeightField, aabb ), (size_t)scalarSize );

	// Arrays
	memcpy( buf + scalarSize,                        hf->compressedHeights, (size_t)heightBytes );
	memcpy( buf + scalarSize + heightBytes,          hf->materialIndices,   (size_t)matBytes );
	memcpy( buf + scalarSize + heightBytes + matBytes, hf->flags,           (size_t)flagBytes );

	*outByteCount = total;
	return buf;
}

b3HeightField* b3UnpackHeightField( const uint8_t* bytes, int byteCount )
{
	(void)byteCount;

	b3HeightField* hf = (b3HeightField*)b3AllocZeroed( sizeof( b3HeightField ) );

	int scalarSize = (int)( sizeof( b3HeightField ) - offsetof( b3HeightField, aabb ) );
	memcpy( (uint8_t*)hf + offsetof( b3HeightField, aabb ), bytes, (size_t)scalarSize );

	// Pointers from the scalar block are stale; rebuild below.
	hf->compressedHeights = NULL;
	hf->materialIndices   = NULL;
	hf->flags             = NULL;

	int heightCount   = hf->columnCount * hf->rowCount;
	int cellCount     = ( hf->columnCount - 1 ) * ( hf->rowCount - 1 );
	int triangleCount = 2 * cellCount;

	int heightBytes  = heightCount * (int)sizeof( uint16_t );
	int matBytes     = cellCount * (int)sizeof( uint8_t );
	int flagBytes    = triangleCount * (int)sizeof( uint8_t );

	hf->compressedHeights = (uint16_t*)b3Alloc( (size_t)heightBytes );
	hf->materialIndices   = (uint8_t*)b3Alloc( (size_t)matBytes );
	hf->flags             = (uint8_t*)b3Alloc( (size_t)flagBytes );

	const uint8_t* src = bytes + scalarSize;
	memcpy( hf->compressedHeights, src, (size_t)heightBytes );
	src += heightBytes;
	memcpy( hf->materialIndices, src, (size_t)matBytes );
	src += matBytes;
	memcpy( hf->flags, src, (size_t)flagBytes );

	hf->version = B3_HEIGHT_FIELD_VERSION;

	// Recompute and assert hash matches to catch any pack/unpack mismatch.
	uint32_t expectedHash = hf->hash;
	hf->hash = 0;
	uint32_t h = B3_HASH_INIT;
	h = b3Hash( h, (const uint8_t*)hf->compressedHeights, heightCount * (int)sizeof( uint16_t ) );
	h = b3Hash( h, (const uint8_t*)hf->materialIndices,   cellCount * (int)sizeof( uint8_t ) );
	h = b3Hash( h, (const uint8_t*)hf->flags,             triangleCount * (int)sizeof( uint8_t ) );
	h = b3Hash( h, (const uint8_t*)hf + offsetof( b3HeightField, aabb ),
	            (int)( sizeof( b3HeightField ) - offsetof( b3HeightField, aabb ) ) );
	hf->hash = b3NonZeroHash( h );

	// TODO: verify hash recompute in Phase 2 (assert matches expectedHash)
	(void)expectedHash;

	return hf;
}

// Lifecycle

b3Recording* b3CreateRecording( int byteCapacity )
{
	b3Recording* rec = (b3Recording*)b3Alloc( sizeof( b3Recording ) );
	*rec = (b3Recording){ 0 };

	int initCap = byteCapacity > 0 ? byteCapacity : 65536;
	rec->buffer.data     = (uint8_t*)b3Alloc( (size_t)initCap );
	rec->buffer.capacity = initCap;
	rec->buffer.size     = 0;
	rec->lock = b3CreateMutex();
	return rec;
}

void b3DestroyRecording( b3Recording* recording )
{
	if ( recording == NULL )
	{
		return;
	}

	b3RecBufFree( &recording->buffer );
	b3FreeRegistry( &recording->registry );
	b3DestroyMutex( recording->lock );
	b3Free( recording, sizeof( b3Recording ) );
}

const uint8_t* b3Recording_GetData( const b3Recording* recording )
{
	return recording->buffer.data;
}

int b3Recording_GetSize( const b3Recording* recording )
{
	return recording->buffer.size;
}

void b3RecAccumulateBounds( b3Recording* rec, b3AABB bounds )
{
	rec->accumulatedBounds = rec->haveBounds ? b3AABB_Union( rec->accumulatedBounds, bounds ) : bounds;
	rec->haveBounds = true;
}

void b3StartRecordingIntoBuffer( b3World* world, b3Recording* recording )
{
	// Reset so a recording handle can be reused for a fresh session
	recording->buffer.size = 0;
	recording->recordStart = 0;
	recording->haveBounds  = false;
	b3FreeRegistry( &recording->registry );

	b3RecHeader hdr = { 0 };
	hdr.magic            = B3_REC_MAGIC;
	hdr.versionMajor     = B3_REC_VERSION_MAJOR;
	hdr.versionMinor     = B3_REC_VERSION_MINOR;
	hdr.pointerWidth     = (uint8_t)sizeof( void* );
	hdr.bigEndian        = 0;
	hdr.validationEnabled = B3_ENABLE_VALIDATION ? 1u : 0u;
	hdr.lengthScale      = b3GetLengthUnitsPerMeter();
	hdr.registryOffset   = 0;       // backpatched in b3StopRecordingInternal
	hdr.registryByteCount = 0;

	world->recording = recording;

	// Every recording is snapshot-seeded. The seed blob follows the header so replay restores in
	// place and the world id stays stable across a restart or backward scrub. An empty world still
	// serializes a valid blob, so there is no from-creation special case.
	b3RecBuffer snapBuf = { 0 };
	b3SerializeWorld( world, &snapBuf, recording );
	hdr.snapshotSize = (uint64_t)snapBuf.size;

	b3RecBufAppend( &recording->buffer, &hdr, (int)sizeof( hdr ) );
	b3RecBufAppend( &recording->buffer, snapBuf.data, snapBuf.size );
	b3RecBufFree( &snapBuf );

	// Anchor the recording with the current world state hash so replay can assert
	// determinism from the very first step.
	b3WorldId worldId = { (uint16_t)( world->worldId + 1 ), world->generation };
	b3RecArgs_StateHash stateHash = { worldId, b3HashWorldState( world ) };
	b3RecWrite_StateHash( recording, &stateHash );
}

void b3StopRecordingInternal( b3World* world )
{
	if ( world->recording == NULL )
	{
		return;
	}

	b3Recording* rec = world->recording;
	world->recording = NULL;

	// Write accumulated bounds so a viewer can frame the whole recorded motion
	b3RecArgs_RecordingBounds rb = { 0 };
	if ( rec->haveBounds )
	{
		rb.bounds = rec->accumulatedBounds;
	}
	b3RecWrite_RecordingBounds( rec, &rb );

	// End-of-stream marker; the buffer is now self-contained
	b3WorldId wid = { (uint16_t)( world->worldId + 1 ), world->generation };
	b3RecArgs_DestroyWorld a = { wid };
	b3RecWrite_DestroyWorld( rec, &a );

	// Write the trailing registry block
	int registryOffset = rec->buffer.size;
	b3RecWriteRegistry( rec );
	int registryByteCount = rec->buffer.size - registryOffset;

	// Backpatch registryOffset and registryByteCount into the header
	uint8_t* hdrBytes = rec->buffer.data;
	uint64_t regOff = (uint64_t)registryOffset;
	uint64_t regSz  = (uint64_t)registryByteCount;
	// Little-endian backpatch in place; offsetof keeps this correct if the header layout shifts
	uint8_t* pOff = hdrBytes + offsetof( b3RecHeader, registryOffset );
	uint8_t* pSz  = hdrBytes + offsetof( b3RecHeader, registryByteCount );
	for ( int i = 0; i < 8; ++i )
	{
		pOff[i] = (uint8_t)( regOff >> ( 8 * i ) );
		pSz[i]  = (uint8_t)( regSz  >> ( 8 * i ) );
	}
}

// Convenience file I/O

bool b3SaveRecordingToFile( const b3Recording* recording, const char* path )
{
	if ( recording == NULL || path == NULL )
	{
		return false;
	}

	FILE* f = fopen( path, "wb" );
	if ( f == NULL )
	{
		return false;
	}

	size_t written = fwrite( recording->buffer.data, 1, (size_t)recording->buffer.size, f );
	fclose( f );
	return (int)written == recording->buffer.size;
}

b3Recording* b3LoadRecordingFromFile( const char* path )
{
	if ( path == NULL )
	{
		return NULL;
	}

	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		return NULL;
	}

	if ( fseek( f, 0, SEEK_END ) != 0 )
	{
		fclose( f );
		return NULL;
	}

	long fileSize = ftell( f );
	// Anything smaller than the fixed header can't be a recording
	if ( fileSize < (long)sizeof( b3RecHeader ) || fileSize > INT_MAX )
	{
		fclose( f );
		return NULL;
	}
	fseek( f, 0, SEEK_SET );

	b3Recording* rec = b3CreateRecording( (int)fileSize );
	size_t readSize = fread( rec->buffer.data, 1, (size_t)fileSize, f );
	fclose( f );

	if ( (long)readSize != fileSize )
	{
		b3DestroyRecording( rec );
		return NULL;
	}

	// Validate magic so a wrong file fails at load rather than deep in the player
	b3RecHeader hdr;
	memcpy( &hdr, rec->buffer.data, sizeof( hdr ) );
	if ( hdr.magic != B3_REC_MAGIC )
	{
		b3DestroyRecording( rec );
		return NULL;
	}

	rec->buffer.size = (int)fileSize;
	return rec;
}

// Geometry interning helpers

uint32_t b3RecInternHull( b3Recording* rec, const b3HullData* hull )
{
	int byteCount = hull->byteCount;
	uint8_t* bytes = (uint8_t*)b3Alloc( (size_t)byteCount );
	memcpy( bytes, hull, (size_t)byteCount );
	uint64_t h = b3Hash64Blob( bytes, byteCount );
	return b3InternGeometry( &rec->registry, b3_geometryHull, h, bytes, byteCount );
}

uint32_t b3RecInternMesh( b3Recording* rec, const b3MeshData* mesh )
{
	int byteCount = mesh->byteCount;
	uint8_t* bytes = (uint8_t*)b3Alloc( (size_t)byteCount );
	memcpy( bytes, mesh, (size_t)byteCount );
	uint64_t h = b3Hash64Blob( bytes, byteCount );
	return b3InternGeometry( &rec->registry, b3_geometryMesh, h, bytes, byteCount );
}

uint32_t b3RecInternHeightField( b3Recording* rec, const b3HeightField* hf )
{
	int byteCount = 0;
	uint8_t* bytes = b3PackHeightField( hf, &byteCount );
	uint64_t h = b3Hash64Blob( bytes, byteCount );
	return b3InternGeometry( &rec->registry, b3_geometryHeightField, h, bytes, byteCount );
}

uint32_t b3RecInternCompound( b3Recording* rec, const b3Compound* compound )
{
	int byteCount = compound->byteCount;
	uint8_t* bytes = (uint8_t*)b3Alloc( (size_t)byteCount );
	memcpy( bytes, compound, (size_t)byteCount );
	// Null the tree node pointer in the copy so the canonical bytes are pointer-free.
	// b3ConvertBytesToCompound fixes it back on load via nodeOffset.
	( (b3Compound*)bytes )->tree.nodes = NULL;
	uint64_t h = b3Hash64Blob( bytes, byteCount );
	return b3InternGeometry( &rec->registry, b3_geometryCompound, h, bytes, byteCount );
}

uint64_t b3HashWorldState( b3World* world )
{
	uint64_t hash = B3_SNAP_FNV_INIT;
	const uint64_t prime = B3_SNAP_FNV_PRIME;

	int bodyCount = world->bodies.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b3Body* body = world->bodies.data + i;
		if ( body->id != i )
		{
			// Free or never-used slot
			continue;
		}

		b3BodySim* sim = b3GetBodySim( world, body );

		uint32_t bits;

#define B3_HASH_FLOAT( f )                                                                                                       \
	memcpy( &bits, &( f ), 4 );                                                                                                  \
	hash = ( hash ^ (uint64_t)bits ) * prime;

		hash = b3FnvMixPosition( hash, sim->transform.p );
		B3_HASH_FLOAT( sim->transform.q.v.x )
		B3_HASH_FLOAT( sim->transform.q.v.y )
		B3_HASH_FLOAT( sim->transform.q.v.z )
		B3_HASH_FLOAT( sim->transform.q.s )

		b3BodyState* state = b3GetBodyState( world, body );
		if ( state != NULL )
		{
			B3_HASH_FLOAT( state->linearVelocity.x )
			B3_HASH_FLOAT( state->linearVelocity.y )
			B3_HASH_FLOAT( state->linearVelocity.z )
			B3_HASH_FLOAT( state->angularVelocity.x )
			B3_HASH_FLOAT( state->angularVelocity.y )
			B3_HASH_FLOAT( state->angularVelocity.z )
		}

#undef B3_HASH_FLOAT
	}

	return hash;
}

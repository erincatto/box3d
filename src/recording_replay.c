// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "recording_replay.h"

#include "compound.h"
#include "physics_world.h"
#include "world_snapshot.h"

#include "box3d/box3d.h"

#include <stdio.h>
#include <string.h>

// Read primitives

static void b3RecRdrCheck( b3RecReader* rdr, int size )
{
	if ( size < 0 || (int64_t)rdr->cursor + (int64_t)size > (int64_t)rdr->size )
	{
		rdr->ok = false;
	}
}

static void b3RecRdrBlob( b3RecReader* rdr, void* out, int size )
{
	b3RecRdrCheck( rdr, size );
	if ( !rdr->ok )
	{
		memset( out, 0, (size_t)size );
		return;
	}
	memcpy( out, rdr->data + rdr->cursor, (size_t)size );
	rdr->cursor += size;
}

uint8_t b3RecR_U8( b3RecReader* rdr )
{
	b3RecRdrCheck( rdr, 1 );
	if ( !rdr->ok )
	{
		return 0;
	}
	return rdr->data[rdr->cursor++];
}

uint16_t b3RecR_U16( b3RecReader* rdr )
{
	b3RecRdrCheck( rdr, 2 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint16_t v = (uint16_t)rdr->data[rdr->cursor] | ( (uint16_t)rdr->data[rdr->cursor + 1] << 8 );
	rdr->cursor += 2;
	return v;
}

uint32_t b3RecR_U24( b3RecReader* rdr )
{
	b3RecRdrCheck( rdr, 3 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint32_t v = (uint32_t)rdr->data[rdr->cursor] | ( (uint32_t)rdr->data[rdr->cursor + 1] << 8 ) |
	             ( (uint32_t)rdr->data[rdr->cursor + 2] << 16 );
	rdr->cursor += 3;
	return v;
}

uint32_t b3RecR_U32( b3RecReader* rdr )
{
	b3RecRdrCheck( rdr, 4 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint32_t v = (uint32_t)rdr->data[rdr->cursor] | ( (uint32_t)rdr->data[rdr->cursor + 1] << 8 ) |
	             ( (uint32_t)rdr->data[rdr->cursor + 2] << 16 ) | ( (uint32_t)rdr->data[rdr->cursor + 3] << 24 );
	rdr->cursor += 4;
	return v;
}

uint64_t b3RecR_U64( b3RecReader* rdr )
{
	b3RecRdrCheck( rdr, 8 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint64_t v = (uint64_t)rdr->data[rdr->cursor] | ( (uint64_t)rdr->data[rdr->cursor + 1] << 8 ) |
	             ( (uint64_t)rdr->data[rdr->cursor + 2] << 16 ) | ( (uint64_t)rdr->data[rdr->cursor + 3] << 24 ) |
	             ( (uint64_t)rdr->data[rdr->cursor + 4] << 32 ) | ( (uint64_t)rdr->data[rdr->cursor + 5] << 40 ) |
	             ( (uint64_t)rdr->data[rdr->cursor + 6] << 48 ) | ( (uint64_t)rdr->data[rdr->cursor + 7] << 56 );
	rdr->cursor += 8;
	return v;
}

int32_t b3RecR_I32( b3RecReader* rdr )
{
	return (int32_t)b3RecR_U32( rdr );
}

float b3RecR_F32( b3RecReader* rdr )
{
	uint32_t bits = b3RecR_U32( rdr );
	float v;
	memcpy( &v, &bits, 4 );
	return v;
}

double b3RecR_F64( b3RecReader* rdr )
{
	uint64_t bits = b3RecR_U64( rdr );
	double v;
	memcpy( &v, &bits, 8 );
	return v;
}

bool b3RecR_BOOL( b3RecReader* rdr )
{
	return b3RecR_U8( rdr ) != 0u;
}

b3Vec3 b3RecR_VEC3( b3RecReader* rdr )
{
	b3Vec3 v;
	v.x = b3RecR_F32( rdr );
	v.y = b3RecR_F32( rdr );
	v.z = b3RecR_F32( rdr );
	return v;
}

b3Quat b3RecR_QUAT( b3RecReader* rdr )
{
	b3Quat q;
	q.v.x = b3RecR_F32( rdr );
	q.v.y = b3RecR_F32( rdr );
	q.v.z = b3RecR_F32( rdr );
	q.s   = b3RecR_F32( rdr );
	return q;
}

b3Transform b3RecR_TRANSFORM( b3RecReader* rdr )
{
	b3Transform t;
	t.p = b3RecR_VEC3( rdr );
	t.q = b3RecR_QUAT( rdr );
	return t;
}

b3Pos b3RecR_POSITION( b3RecReader* rdr )
{
	b3Pos p;
#if defined( BOX3D_DOUBLE_PRECISION )
	p.x = b3RecR_F64( rdr );
	p.y = b3RecR_F64( rdr );
	p.z = b3RecR_F64( rdr );
#else
	p.x = b3RecR_F32( rdr );
	p.y = b3RecR_F32( rdr );
	p.z = b3RecR_F32( rdr );
#endif
	return p;
}

b3WorldTransform b3RecR_WORLDXF( b3RecReader* rdr )
{
	b3WorldTransform t;
	t.p = b3RecR_POSITION( rdr );
	t.q = b3RecR_QUAT( rdr );
	return t;
}

b3Matrix3 b3RecR_MATRIX3( b3RecReader* rdr )
{
	b3Matrix3 m;
	m.cx = b3RecR_VEC3( rdr );
	m.cy = b3RecR_VEC3( rdr );
	m.cz = b3RecR_VEC3( rdr );
	return m;
}

b3AABB b3RecR_AABB( b3RecReader* rdr )
{
	b3AABB v;
	v.lowerBound = b3RecR_VEC3( rdr );
	v.upperBound = b3RecR_VEC3( rdr );
	return v;
}

b3WorldId b3RecR_WORLDID( b3RecReader* rdr )
{
	return b3LoadWorldId( b3RecR_U32( rdr ) );
}

b3BodyId b3RecR_BODYID( b3RecReader* rdr )
{
	return b3LoadBodyId( b3RecR_U64( rdr ) );
}

b3ShapeId b3RecR_SHAPEID( b3RecReader* rdr )
{
	return b3LoadShapeId( b3RecR_U64( rdr ) );
}

b3JointId b3RecR_JOINTID( b3RecReader* rdr )
{
	return b3LoadJointId( b3RecR_U64( rdr ) );
}

b3Sphere b3RecR_SPHERE( b3RecReader* rdr )
{
	b3Sphere s;
	b3RecRdrBlob( rdr, &s, (int)sizeof( s ) );
	return s;
}

b3Capsule b3RecR_CAPSULE( b3RecReader* rdr )
{
	b3Capsule c;
	b3RecRdrBlob( rdr, &c, (int)sizeof( c ) );
	return c;
}

uint32_t b3RecR_GEOMID( b3RecReader* rdr )
{
	return b3RecR_U32( rdr );
}

b3Filter b3RecR_FILTER( b3RecReader* rdr )
{
	b3Filter f;
	f.categoryBits = b3RecR_U64( rdr );
	f.maskBits     = b3RecR_U64( rdr );
	f.groupIndex   = b3RecR_I32( rdr );
	return f;
}

b3SurfaceMaterial b3RecR_MATERIAL( b3RecReader* rdr )
{
	b3SurfaceMaterial m = b3DefaultSurfaceMaterial();
	m.friction          = b3RecR_F32( rdr );
	m.restitution       = b3RecR_F32( rdr );
	m.rollingResistance = b3RecR_F32( rdr );
	m.tangentVelocity   = b3RecR_VEC3( rdr );
	m.userMaterialId    = b3RecR_U64( rdr );
	m.customColor       = b3RecR_U32( rdr );
	return m;
}

b3MassData b3RecR_MASSDATA( b3RecReader* rdr )
{
	b3MassData md;
	md.mass    = b3RecR_F32( rdr );
	md.center  = b3RecR_VEC3( rdr );
	md.inertia = b3RecR_MATRIX3( rdr );
	return md;
}

b3MotionLocks b3RecR_LOCKS( b3RecReader* rdr )
{
	b3MotionLocks locks;
	locks.linearX  = b3RecR_BOOL( rdr );
	locks.linearY  = b3RecR_BOOL( rdr );
	locks.linearZ  = b3RecR_BOOL( rdr );
	locks.angularX = b3RecR_BOOL( rdr );
	locks.angularY = b3RecR_BOOL( rdr );
	locks.angularZ = b3RecR_BOOL( rdr );
	return locks;
}

// Rotating set of static string buffers, valid until the next 4 STR reads.
const char* b3RecR_STR( b3RecReader* rdr )
{
	char* buf    = rdr->strBufs[rdr->strNext];
	rdr->strNext = ( rdr->strNext + 1 ) & 3;

	uint16_t len = b3RecR_U16( rdr );
	if ( len == 0xFFFFu )
	{
		return NULL;
	}

	int n = (int)len;
	if ( n > B3_NAME_LENGTH )
	{
		n = B3_NAME_LENGTH;
	}
	b3RecRdrCheck( rdr, (int)len );
	if ( rdr->ok && n > 0 )
	{
		memcpy( buf, rdr->data + rdr->cursor, (size_t)n );
	}
	rdr->cursor += (int)len;
	buf[n] = '\0';
	return buf;
}

// Def readers: start from b3Default*Def() then overlay each serialized field
// in the exact order the writer produced them.

b3ExplosionDef b3RecR_EXPLOSIONDEF( b3RecReader* rdr )
{
	b3ExplosionDef def = b3DefaultExplosionDef();
	def.maskBits       = b3RecR_U64( rdr );
	def.position       = b3RecR_POSITION( rdr );
	def.radius         = b3RecR_F32( rdr );
	def.falloff        = b3RecR_F32( rdr );
	def.impulsePerArea = b3RecR_F32( rdr );
	return def;
}

b3BodyDef b3RecR_BODYDEF( b3RecReader* rdr )
{
	b3BodyDef def = b3DefaultBodyDef();
	def.type             = (b3BodyType)b3RecR_I32( rdr );
	def.position         = b3RecR_POSITION( rdr );
	def.rotation         = b3RecR_QUAT( rdr );
	def.linearVelocity   = b3RecR_VEC3( rdr );
	def.angularVelocity  = b3RecR_VEC3( rdr );
	def.linearDamping    = b3RecR_F32( rdr );
	def.angularDamping   = b3RecR_F32( rdr );
	def.gravityScale     = b3RecR_F32( rdr );
	def.sleepThreshold   = b3RecR_F32( rdr );
	def.name             = b3RecR_STR( rdr );
	(void)b3RecR_U64( rdr ); // userData placeholder
	def.motionLocks            = b3RecR_LOCKS( rdr );
	def.enableSleep            = b3RecR_BOOL( rdr );
	def.isAwake                = b3RecR_BOOL( rdr );
	def.isBullet               = b3RecR_BOOL( rdr );
	def.isEnabled              = b3RecR_BOOL( rdr );
	def.allowFastRotation      = b3RecR_BOOL( rdr );
	def.enableContactRecycling = b3RecR_BOOL( rdr );
	def.userData               = NULL;
	return def;
}

// Reserve/grow the reader's material scratch. Returns false and sets ok=false on bad count.
static bool b3RecReserveMat( b3RecReader* rdr, int need )
{
	int remaining = rdr->size - rdr->cursor;
	if ( need < 0 || remaining < 0 || need > remaining )
	{
		rdr->ok = false;
		return false;
	}
	if ( need <= rdr->matScratchCap )
	{
		return true;
	}
	int newCap = need + 8;
	if ( rdr->matScratch != NULL )
	{
		b3Free( rdr->matScratch, (size_t)rdr->matScratchCap * sizeof( b3SurfaceMaterial ) );
	}
	rdr->matScratch    = (b3SurfaceMaterial*)b3Alloc( (size_t)newCap * sizeof( b3SurfaceMaterial ) );
	rdr->matScratchCap = newCap;
	return true;
}

b3ShapeDef b3RecR_SHAPEDEF( b3RecReader* rdr )
{
	b3ShapeDef def = b3DefaultShapeDef();
	def.name         = b3RecR_STR( rdr );
	(void)b3RecR_U64( rdr ); // userData placeholder

	int matCount = b3RecR_I32( rdr );
	if ( matCount < 0 )
	{
		matCount = 0;
	}
	if ( matCount > 0 && b3RecReserveMat( rdr, matCount ) )
	{
		for ( int i = 0; i < matCount; ++i )
		{
			rdr->matScratch[i] = b3RecR_MATERIAL( rdr );
		}
		def.materials    = rdr->matScratch;
		def.materialCount = matCount;
	}
	else
	{
		for ( int i = 0; i < matCount; ++i )
		{
			(void)b3RecR_MATERIAL( rdr );
		}
		def.materials    = NULL;
		def.materialCount = 0;
	}

	def.baseMaterial          = b3RecR_MATERIAL( rdr );
	def.density               = b3RecR_F32( rdr );
	def.explosionScale        = b3RecR_F32( rdr );
	def.filter                = b3RecR_FILTER( rdr );
	def.enableCustomFiltering = b3RecR_BOOL( rdr );
	def.isSensor              = b3RecR_BOOL( rdr );
	def.enableSensorEvents    = b3RecR_BOOL( rdr );
	def.enableContactEvents   = b3RecR_BOOL( rdr );
	def.enableHitEvents       = b3RecR_BOOL( rdr );
	def.enablePreSolveEvents  = b3RecR_BOOL( rdr );
	def.invokeContactCreation = b3RecR_BOOL( rdr );
	def.updateBodyMass        = b3RecR_BOOL( rdr );
	def.userData              = NULL;
	return def;
}

// Shared base for all joint defs. Body ids come in with recorded world0; callers remap them.
static void b3RecR_JointBase( b3RecReader* rdr, b3JointDef* base )
{
	(void)b3RecR_U64( rdr ); // userData
	base->bodyIdA             = b3RecR_BODYID( rdr );
	base->bodyIdB             = b3RecR_BODYID( rdr );
	base->localFrameA         = b3RecR_TRANSFORM( rdr );
	base->localFrameB         = b3RecR_TRANSFORM( rdr );
	base->forceThreshold      = b3RecR_F32( rdr );
	base->torqueThreshold     = b3RecR_F32( rdr );
	base->constraintHertz     = b3RecR_F32( rdr );
	base->constraintDampingRatio = b3RecR_F32( rdr );
	base->drawScale           = b3RecR_F32( rdr );
	base->collideConnected    = b3RecR_BOOL( rdr );
	base->userData            = NULL;
}

b3ParallelJointDef b3RecR_PARALLELJOINTDEF( b3RecReader* rdr )
{
	b3ParallelJointDef def = b3DefaultParallelJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.hertz        = b3RecR_F32( rdr );
	def.dampingRatio = b3RecR_F32( rdr );
	def.maxTorque    = b3RecR_F32( rdr );
	return def;
}

b3DistanceJointDef b3RecR_DISTANCEJOINTDEF( b3RecReader* rdr )
{
	b3DistanceJointDef def = b3DefaultDistanceJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.length            = b3RecR_F32( rdr );
	def.enableSpring      = b3RecR_BOOL( rdr );
	def.lowerSpringForce  = b3RecR_F32( rdr );
	def.upperSpringForce  = b3RecR_F32( rdr );
	def.hertz             = b3RecR_F32( rdr );
	def.dampingRatio      = b3RecR_F32( rdr );
	def.enableLimit       = b3RecR_BOOL( rdr );
	def.minLength         = b3RecR_F32( rdr );
	def.maxLength         = b3RecR_F32( rdr );
	def.enableMotor       = b3RecR_BOOL( rdr );
	def.maxMotorForce     = b3RecR_F32( rdr );
	def.motorSpeed        = b3RecR_F32( rdr );
	return def;
}

b3FilterJointDef b3RecR_FILTERJOINTDEF( b3RecReader* rdr )
{
	b3FilterJointDef def = b3DefaultFilterJointDef();
	b3RecR_JointBase( rdr, &def.base );
	return def;
}

b3MotorJointDef b3RecR_MOTORJOINTDEF( b3RecReader* rdr )
{
	b3MotorJointDef def = b3DefaultMotorJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.linearVelocity      = b3RecR_VEC3( rdr );
	def.maxVelocityForce    = b3RecR_F32( rdr );
	def.angularVelocity     = b3RecR_VEC3( rdr );
	def.maxVelocityTorque   = b3RecR_F32( rdr );
	def.linearHertz         = b3RecR_F32( rdr );
	def.linearDampingRatio  = b3RecR_F32( rdr );
	def.maxSpringForce      = b3RecR_F32( rdr );
	def.angularHertz        = b3RecR_F32( rdr );
	def.angularDampingRatio = b3RecR_F32( rdr );
	def.maxSpringTorque     = b3RecR_F32( rdr );
	return def;
}

b3PrismaticJointDef b3RecR_PRISMATICJOINTDEF( b3RecReader* rdr )
{
	b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.enableSpring       = b3RecR_BOOL( rdr );
	def.hertz              = b3RecR_F32( rdr );
	def.dampingRatio       = b3RecR_F32( rdr );
	def.targetTranslation  = b3RecR_F32( rdr );
	def.enableLimit        = b3RecR_BOOL( rdr );
	def.lowerTranslation   = b3RecR_F32( rdr );
	def.upperTranslation   = b3RecR_F32( rdr );
	def.enableMotor        = b3RecR_BOOL( rdr );
	def.maxMotorForce      = b3RecR_F32( rdr );
	def.motorSpeed         = b3RecR_F32( rdr );
	return def;
}

b3RevoluteJointDef b3RecR_REVOLUTEJOINTDEF( b3RecReader* rdr )
{
	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.targetAngle    = b3RecR_F32( rdr );
	def.enableSpring   = b3RecR_BOOL( rdr );
	def.hertz          = b3RecR_F32( rdr );
	def.dampingRatio   = b3RecR_F32( rdr );
	def.enableLimit    = b3RecR_BOOL( rdr );
	def.lowerAngle     = b3RecR_F32( rdr );
	def.upperAngle     = b3RecR_F32( rdr );
	def.enableMotor    = b3RecR_BOOL( rdr );
	def.maxMotorTorque = b3RecR_F32( rdr );
	def.motorSpeed     = b3RecR_F32( rdr );
	return def;
}

b3SphericalJointDef b3RecR_SPHERICALJOINTDEF( b3RecReader* rdr )
{
	b3SphericalJointDef def = b3DefaultSphericalJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.enableSpring     = b3RecR_BOOL( rdr );
	def.hertz            = b3RecR_F32( rdr );
	def.dampingRatio     = b3RecR_F32( rdr );
	def.targetRotation   = b3RecR_QUAT( rdr );
	def.enableConeLimit  = b3RecR_BOOL( rdr );
	def.coneAngle        = b3RecR_F32( rdr );
	def.enableTwistLimit = b3RecR_BOOL( rdr );
	def.lowerTwistAngle  = b3RecR_F32( rdr );
	def.upperTwistAngle  = b3RecR_F32( rdr );
	def.enableMotor      = b3RecR_BOOL( rdr );
	def.maxMotorTorque   = b3RecR_F32( rdr );
	def.motorVelocity    = b3RecR_VEC3( rdr );
	return def;
}

b3WeldJointDef b3RecR_WELDJOINTDEF( b3RecReader* rdr )
{
	b3WeldJointDef def = b3DefaultWeldJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.linearHertz         = b3RecR_F32( rdr );
	def.angularHertz        = b3RecR_F32( rdr );
	def.linearDampingRatio  = b3RecR_F32( rdr );
	def.angularDampingRatio = b3RecR_F32( rdr );
	return def;
}

b3WheelJointDef b3RecR_WHEELJOINTDEF( b3RecReader* rdr )
{
	b3WheelJointDef def = b3DefaultWheelJointDef();
	b3RecR_JointBase( rdr, &def.base );
	def.enableSuspensionSpring  = b3RecR_BOOL( rdr );
	def.suspensionHertz         = b3RecR_F32( rdr );
	def.suspensionDampingRatio  = b3RecR_F32( rdr );
	def.enableSuspensionLimit   = b3RecR_BOOL( rdr );
	def.lowerSuspensionLimit    = b3RecR_F32( rdr );
	def.upperSuspensionLimit    = b3RecR_F32( rdr );
	def.enableSpinMotor         = b3RecR_BOOL( rdr );
	def.maxSpinTorque           = b3RecR_F32( rdr );
	def.spinSpeed               = b3RecR_F32( rdr );
	def.enableSteering          = b3RecR_BOOL( rdr );
	def.steeringHertz           = b3RecR_F32( rdr );
	def.steeringDampingRatio    = b3RecR_F32( rdr );
	def.targetSteeringAngle     = b3RecR_F32( rdr );
	def.maxSteeringTorque       = b3RecR_F32( rdr );
	def.enableSteeringLimit     = b3RecR_BOOL( rdr );
	def.lowerSteeringLimit      = b3RecR_F32( rdr );
	def.upperSteeringLimit      = b3RecR_F32( rdr );
	return def;
}

// Id retargeting: replace world0 with the replay world's slot index.

static b3BodyId b3RecMakeBodyId( b3RecReader* rdr, b3BodyId recorded )
{
	b3BodyId id;
	id.index1    = recorded.index1;
	id.world0    = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static b3ShapeId b3RecMakeShapeId( b3RecReader* rdr, b3ShapeId recorded )
{
	b3ShapeId id;
	id.index1    = recorded.index1;
	id.world0    = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static b3JointId b3RecMakeJointId( b3RecReader* rdr, b3JointId recorded )
{
	b3JointId id;
	id.index1    = recorded.index1;
	id.world0    = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

// A create op appends the returned id after args. index1 and generation must match;
// world0 always differs so we ignore it.
static void b3RecCheckId( b3RecReader* rdr, const char* kind, int gotIndex, unsigned gotGen,
                          int recIndex, unsigned recGen )
{
	if ( gotIndex != recIndex || gotGen != recGen )
	{
		printf( "b3ValidateReplay: %s id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n",
		        kind, recIndex, recGen, gotIndex, gotGen );
		rdr->ok = false;
	}
}

static void b3RecCheckBodyId( b3RecReader* rdr, b3BodyId got, b3BodyId rec )
{
	b3RecCheckId( rdr, "body", got.index1, got.generation, rec.index1, rec.generation );
}

static void b3RecCheckShapeId( b3RecReader* rdr, b3ShapeId got, b3ShapeId rec )
{
	b3RecCheckId( rdr, "shape", got.index1, got.generation, rec.index1, rec.generation );
}

static void b3RecCheckJointId( b3RecReader* rdr, b3JointId got, b3JointId rec )
{
	b3RecCheckId( rdr, "joint", got.index1, got.generation, rec.index1, rec.generation );
}

// Registry slot reconstruction. Returns the live pointer for the given slot, building it
// on first use. The hull case is handled inline at the call site since it doesn't cache.

static void* b3RecGetLiveMesh( b3RegistrySlot* slot )
{
	if ( slot->live != NULL )
	{
		return slot->live;
	}
	// Mesh is stored by reference: must outlive the world.
	slot->live = b3Alloc( (size_t)slot->byteCount );
	memcpy( slot->live, slot->bytes, (size_t)slot->byteCount );
	return slot->live;
}

static void* b3RecGetLiveHeightField( b3RegistrySlot* slot )
{
	if ( slot->live != NULL )
	{
		return slot->live;
	}
	slot->live = b3UnpackHeightField( slot->bytes, slot->byteCount );
	return slot->live;
}

static void* b3RecGetLiveCompound( b3RegistrySlot* slot )
{
	if ( slot->live != NULL )
	{
		return slot->live;
	}
	// b3ConvertBytesToCompound mutates its input; give it a writable copy.
	slot->live = b3Alloc( (size_t)slot->byteCount );
	memcpy( slot->live, slot->bytes, (size_t)slot->byteCount );
	b3ConvertBytesToCompound( (uint8_t*)slot->live, slot->byteCount );
	return slot->live;
}

// Dispatch functions, one per op

static void b3RecDispatch_DestroyWorld( const b3RecArgs_DestroyWorld* a, b3RecReader* rdr )
{
	(void)a;
	(void)rdr;
	// End-of-session marker. The replay world is torn down in b3ValidateReplay, not here.
}

static void b3RecDispatch_Step( const b3RecArgs_Step* a, b3RecReader* rdr )
{
	(void)a;
	b3World_Step( rdr->replayWorldId, a->dt, a->subStepCount );
}

static void b3RecDispatch_WorldEnableSleeping( const b3RecArgs_WorldEnableSleeping* a, b3RecReader* rdr )
{
	b3World_EnableSleeping( rdr->replayWorldId, a->flag );
}

static void b3RecDispatch_WorldEnableContinuous( const b3RecArgs_WorldEnableContinuous* a, b3RecReader* rdr )
{
	b3World_EnableContinuous( rdr->replayWorldId, a->flag );
}

static void b3RecDispatch_WorldSetRestitutionThreshold( const b3RecArgs_WorldSetRestitutionThreshold* a, b3RecReader* rdr )
{
	b3World_SetRestitutionThreshold( rdr->replayWorldId, a->value );
}

static void b3RecDispatch_WorldSetHitEventThreshold( const b3RecArgs_WorldSetHitEventThreshold* a, b3RecReader* rdr )
{
	b3World_SetHitEventThreshold( rdr->replayWorldId, a->value );
}

static void b3RecDispatch_WorldSetGravity( const b3RecArgs_WorldSetGravity* a, b3RecReader* rdr )
{
	b3World_SetGravity( rdr->replayWorldId, a->gravity );
}

static void b3RecDispatch_WorldExplode( const b3RecArgs_WorldExplode* a, b3RecReader* rdr )
{
	b3World_Explode( rdr->replayWorldId, &a->def );
}

static void b3RecDispatch_WorldSetContactTuning( const b3RecArgs_WorldSetContactTuning* a, b3RecReader* rdr )
{
	b3World_SetContactTuning( rdr->replayWorldId, a->hertz, a->dampingRatio, a->contactSpeed );
}

static void b3RecDispatch_WorldSetContactRecycleDistance( const b3RecArgs_WorldSetContactRecycleDistance* a, b3RecReader* rdr )
{
	b3World_SetContactRecycleDistance( rdr->replayWorldId, a->recycleDistance );
}

static void b3RecDispatch_WorldSetMaximumLinearSpeed( const b3RecArgs_WorldSetMaximumLinearSpeed* a, b3RecReader* rdr )
{
	b3World_SetMaximumLinearSpeed( rdr->replayWorldId, a->maximumLinearSpeed );
}

static void b3RecDispatch_WorldEnableWarmStarting( const b3RecArgs_WorldEnableWarmStarting* a, b3RecReader* rdr )
{
	b3World_EnableWarmStarting( rdr->replayWorldId, a->flag );
}

static void b3RecDispatch_WorldRebuildStaticTree( const b3RecArgs_WorldRebuildStaticTree* a, b3RecReader* rdr )
{
	(void)a;
	b3World_RebuildStaticTree( rdr->replayWorldId );
}

static void b3RecDispatch_WorldEnableSpeculative( const b3RecArgs_WorldEnableSpeculative* a, b3RecReader* rdr )
{
	b3World_EnableSpeculative( rdr->replayWorldId, a->flag );
}

static void b3RecDispatch_CreateBody( const b3RecArgs_CreateBody* a, b3RecReader* rdr )
{
	b3BodyId recId = b3RecR_BODYID( rdr );
	b3BodyId gotId = b3CreateBody( rdr->replayWorldId, &a->def );
	b3RecCheckBodyId( rdr, gotId, recId );
}

static void b3RecDispatch_DestroyBody( const b3RecArgs_DestroyBody* a, b3RecReader* rdr )
{
	b3DestroyBody( b3RecMakeBodyId( rdr, a->body ) );
}

static void b3RecDispatch_BodySetTransform( const b3RecArgs_BodySetTransform* a, b3RecReader* rdr )
{
	b3Body_SetTransform( b3RecMakeBodyId( rdr, a->body ), a->position, a->rotation );
}

static void b3RecDispatch_BodySetLinearVelocity( const b3RecArgs_BodySetLinearVelocity* a, b3RecReader* rdr )
{
	b3Body_SetLinearVelocity( b3RecMakeBodyId( rdr, a->body ), a->v );
}

static void b3RecDispatch_BodySetType( const b3RecArgs_BodySetType* a, b3RecReader* rdr )
{
	b3Body_SetType( b3RecMakeBodyId( rdr, a->body ), (b3BodyType)a->type );
}

static void b3RecDispatch_BodySetName( const b3RecArgs_BodySetName* a, b3RecReader* rdr )
{
	b3Body_SetName( b3RecMakeBodyId( rdr, a->body ), a->name );
}

static void b3RecDispatch_BodySetAngularVelocity( const b3RecArgs_BodySetAngularVelocity* a, b3RecReader* rdr )
{
	b3Body_SetAngularVelocity( b3RecMakeBodyId( rdr, a->body ), a->w );
}

static void b3RecDispatch_BodySetTargetTransform( const b3RecArgs_BodySetTargetTransform* a, b3RecReader* rdr )
{
	b3Body_SetTargetTransform( b3RecMakeBodyId( rdr, a->body ), a->target, a->timeStep, a->wake );
}

static void b3RecDispatch_BodyApplyForce( const b3RecArgs_BodyApplyForce* a, b3RecReader* rdr )
{
	b3Body_ApplyForce( b3RecMakeBodyId( rdr, a->body ), a->force, a->point, a->wake );
}

static void b3RecDispatch_BodyApplyForceToCenter( const b3RecArgs_BodyApplyForceToCenter* a, b3RecReader* rdr )
{
	b3Body_ApplyForceToCenter( b3RecMakeBodyId( rdr, a->body ), a->force, a->wake );
}

static void b3RecDispatch_BodyApplyTorque( const b3RecArgs_BodyApplyTorque* a, b3RecReader* rdr )
{
	b3Body_ApplyTorque( b3RecMakeBodyId( rdr, a->body ), a->torque, a->wake );
}

static void b3RecDispatch_BodyApplyLinearImpulse( const b3RecArgs_BodyApplyLinearImpulse* a, b3RecReader* rdr )
{
	b3Body_ApplyLinearImpulse( b3RecMakeBodyId( rdr, a->body ), a->impulse, a->point, a->wake );
}

static void b3RecDispatch_BodyApplyLinearImpulseToCenter( const b3RecArgs_BodyApplyLinearImpulseToCenter* a, b3RecReader* rdr )
{
	b3Body_ApplyLinearImpulseToCenter( b3RecMakeBodyId( rdr, a->body ), a->impulse, a->wake );
}

static void b3RecDispatch_BodyApplyAngularImpulse( const b3RecArgs_BodyApplyAngularImpulse* a, b3RecReader* rdr )
{
	b3Body_ApplyAngularImpulse( b3RecMakeBodyId( rdr, a->body ), a->impulse, a->wake );
}

static void b3RecDispatch_BodySetMassData( const b3RecArgs_BodySetMassData* a, b3RecReader* rdr )
{
	b3Body_SetMassData( b3RecMakeBodyId( rdr, a->body ), a->massData );
}

static void b3RecDispatch_BodyApplyMassFromShapes( const b3RecArgs_BodyApplyMassFromShapes* a, b3RecReader* rdr )
{
	b3Body_ApplyMassFromShapes( b3RecMakeBodyId( rdr, a->body ) );
}

static void b3RecDispatch_BodySetLinearDamping( const b3RecArgs_BodySetLinearDamping* a, b3RecReader* rdr )
{
	b3Body_SetLinearDamping( b3RecMakeBodyId( rdr, a->body ), a->damping );
}

static void b3RecDispatch_BodySetAngularDamping( const b3RecArgs_BodySetAngularDamping* a, b3RecReader* rdr )
{
	b3Body_SetAngularDamping( b3RecMakeBodyId( rdr, a->body ), a->damping );
}

static void b3RecDispatch_BodySetGravityScale( const b3RecArgs_BodySetGravityScale* a, b3RecReader* rdr )
{
	b3Body_SetGravityScale( b3RecMakeBodyId( rdr, a->body ), a->scale );
}

static void b3RecDispatch_BodySetAwake( const b3RecArgs_BodySetAwake* a, b3RecReader* rdr )
{
	b3Body_SetAwake( b3RecMakeBodyId( rdr, a->body ), a->awake );
}

static void b3RecDispatch_BodyEnableSleep( const b3RecArgs_BodyEnableSleep* a, b3RecReader* rdr )
{
	b3Body_EnableSleep( b3RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b3RecDispatch_BodySetSleepThreshold( const b3RecArgs_BodySetSleepThreshold* a, b3RecReader* rdr )
{
	b3Body_SetSleepThreshold( b3RecMakeBodyId( rdr, a->body ), a->threshold );
}

static void b3RecDispatch_BodyDisable( const b3RecArgs_BodyDisable* a, b3RecReader* rdr )
{
	b3Body_Disable( b3RecMakeBodyId( rdr, a->body ) );
}

static void b3RecDispatch_BodyEnable( const b3RecArgs_BodyEnable* a, b3RecReader* rdr )
{
	b3Body_Enable( b3RecMakeBodyId( rdr, a->body ) );
}

static void b3RecDispatch_BodySetMotionLocks( const b3RecArgs_BodySetMotionLocks* a, b3RecReader* rdr )
{
	b3Body_SetMotionLocks( b3RecMakeBodyId( rdr, a->body ), a->locks );
}

static void b3RecDispatch_BodySetBullet( const b3RecArgs_BodySetBullet* a, b3RecReader* rdr )
{
	b3Body_SetBullet( b3RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b3RecDispatch_BodyEnableContactRecycling( const b3RecArgs_BodyEnableContactRecycling* a, b3RecReader* rdr )
{
	b3Body_EnableContactRecycling( b3RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b3RecDispatch_BodyEnableHitEvents( const b3RecArgs_BodyEnableHitEvents* a, b3RecReader* rdr )
{
	b3Body_EnableHitEvents( b3RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b3RecDispatch_CreateSphereShape( const b3RecArgs_CreateSphereShape* a, b3RecReader* rdr )
{
	b3ShapeId recId = b3RecR_SHAPEID( rdr );
	b3BodyId  bodyId = b3RecMakeBodyId( rdr, a->body );
	b3ShapeId gotId  = b3CreateSphereShape( bodyId, &a->def, &a->sphere );
	b3RecCheckShapeId( rdr, gotId, recId );
}

static void b3RecDispatch_CreateCapsuleShape( const b3RecArgs_CreateCapsuleShape* a, b3RecReader* rdr )
{
	b3ShapeId recId = b3RecR_SHAPEID( rdr );
	b3BodyId  bodyId = b3RecMakeBodyId( rdr, a->body );
	b3ShapeId gotId  = b3CreateCapsuleShape( bodyId, &a->def, &a->capsule );
	b3RecCheckShapeId( rdr, gotId, recId );
}

static void b3RecDispatch_CreateHullShape( const b3RecArgs_CreateHullShape* a, b3RecReader* rdr )
{
	b3ShapeId recId = b3RecR_SHAPEID( rdr );
	if ( !rdr->ok )
	{
		return;
	}
	uint32_t id = a->geometryId;
	if ( id >= (uint32_t)rdr->slotCount )
	{
		printf( "b3ValidateReplay: hull geometryId %u out of range\n", id );
		rdr->ok = false;
		return;
	}
	b3RegistrySlot* slot = rdr->slots + id;
	b3BodyId bodyId = b3RecMakeBodyId( rdr, a->body );
	// Hull is cloned by b3CreateHullShape into the world DB; no caching needed.
	b3ShapeId gotId = b3CreateHullShape( bodyId, &a->def, (const b3HullData*)slot->bytes );
	b3RecCheckShapeId( rdr, gotId, recId );
}

static void b3RecDispatch_CreateMeshShape( const b3RecArgs_CreateMeshShape* a, b3RecReader* rdr )
{
	b3ShapeId recId = b3RecR_SHAPEID( rdr );
	if ( !rdr->ok )
	{
		return;
	}
	uint32_t id = a->geometryId;
	if ( id >= (uint32_t)rdr->slotCount )
	{
		printf( "b3ValidateReplay: mesh geometryId %u out of range\n", id );
		rdr->ok = false;
		return;
	}
	b3RegistrySlot* slot   = rdr->slots + id;
	const b3MeshData* mesh = (const b3MeshData*)b3RecGetLiveMesh( slot );
	b3BodyId bodyId        = b3RecMakeBodyId( rdr, a->body );
	b3ShapeId gotId        = b3CreateMeshShape( bodyId, &a->def, mesh, a->scale );
	b3RecCheckShapeId( rdr, gotId, recId );
}

static void b3RecDispatch_CreateHeightFieldShape( const b3RecArgs_CreateHeightFieldShape* a, b3RecReader* rdr )
{
	b3ShapeId recId = b3RecR_SHAPEID( rdr );
	if ( !rdr->ok )
	{
		return;
	}
	uint32_t id = a->geometryId;
	if ( id >= (uint32_t)rdr->slotCount )
	{
		printf( "b3ValidateReplay: heightfield geometryId %u out of range\n", id );
		rdr->ok = false;
		return;
	}
	b3RegistrySlot* slot         = rdr->slots + id;
	const b3HeightField* hf      = (const b3HeightField*)b3RecGetLiveHeightField( slot );
	b3BodyId bodyId              = b3RecMakeBodyId( rdr, a->body );
	b3ShapeId gotId              = b3CreateHeightFieldShape( bodyId, &a->def, hf );
	b3RecCheckShapeId( rdr, gotId, recId );
}

static void b3RecDispatch_CreateCompoundShape( const b3RecArgs_CreateCompoundShape* a, b3RecReader* rdr )
{
	b3ShapeId recId = b3RecR_SHAPEID( rdr );
	if ( !rdr->ok )
	{
		return;
	}
	uint32_t id = a->geometryId;
	if ( id >= (uint32_t)rdr->slotCount )
	{
		printf( "b3ValidateReplay: compound geometryId %u out of range\n", id );
		rdr->ok = false;
		return;
	}
	b3RegistrySlot* slot      = rdr->slots + id;
	const b3Compound* compound = (const b3Compound*)b3RecGetLiveCompound( slot );
	b3BodyId bodyId            = b3RecMakeBodyId( rdr, a->body );
	// b3CreateCompoundShape takes a non-const def pointer; cast away const for the scratch def
	b3ShapeDef shapeDef = a->def;
	b3ShapeId gotId     = b3CreateCompoundShape( bodyId, &shapeDef, compound );
	b3RecCheckShapeId( rdr, gotId, recId );
}

static void b3RecDispatch_DestroyShape( const b3RecArgs_DestroyShape* a, b3RecReader* rdr )
{
	b3DestroyShape( b3RecMakeShapeId( rdr, a->shape ), a->updateBodyMass );
}

static void b3RecDispatch_ShapeSetDensity( const b3RecArgs_ShapeSetDensity* a, b3RecReader* rdr )
{
	b3Shape_SetDensity( b3RecMakeShapeId( rdr, a->shape ), a->density, a->updateBodyMass );
}

static void b3RecDispatch_ShapeSetFriction( const b3RecArgs_ShapeSetFriction* a, b3RecReader* rdr )
{
	b3Shape_SetFriction( b3RecMakeShapeId( rdr, a->shape ), a->friction );
}

static void b3RecDispatch_ShapeSetRestitution( const b3RecArgs_ShapeSetRestitution* a, b3RecReader* rdr )
{
	b3Shape_SetRestitution( b3RecMakeShapeId( rdr, a->shape ), a->restitution );
}

static void b3RecDispatch_ShapeSetSurfaceMaterial( const b3RecArgs_ShapeSetSurfaceMaterial* a, b3RecReader* rdr )
{
	b3Shape_SetSurfaceMaterial( b3RecMakeShapeId( rdr, a->shape ), a->material );
}

static void b3RecDispatch_ShapeSetFilter( const b3RecArgs_ShapeSetFilter* a, b3RecReader* rdr )
{
	b3Shape_SetFilter( b3RecMakeShapeId( rdr, a->shape ), a->filter, a->invokeContacts );
}

static void b3RecDispatch_ShapeEnableSensorEvents( const b3RecArgs_ShapeEnableSensorEvents* a, b3RecReader* rdr )
{
	b3Shape_EnableSensorEvents( b3RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b3RecDispatch_ShapeEnableContactEvents( const b3RecArgs_ShapeEnableContactEvents* a, b3RecReader* rdr )
{
	b3Shape_EnableContactEvents( b3RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b3RecDispatch_ShapeEnablePreSolveEvents( const b3RecArgs_ShapeEnablePreSolveEvents* a, b3RecReader* rdr )
{
	b3Shape_EnablePreSolveEvents( b3RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b3RecDispatch_ShapeEnableHitEvents( const b3RecArgs_ShapeEnableHitEvents* a, b3RecReader* rdr )
{
	b3Shape_EnableHitEvents( b3RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b3RecDispatch_ShapeSetSphere( const b3RecArgs_ShapeSetSphere* a, b3RecReader* rdr )
{
	b3Shape_SetSphere( b3RecMakeShapeId( rdr, a->shape ), &a->sphere );
}

static void b3RecDispatch_ShapeSetCapsule( const b3RecArgs_ShapeSetCapsule* a, b3RecReader* rdr )
{
	b3Shape_SetCapsule( b3RecMakeShapeId( rdr, a->shape ), &a->capsule );
}

static void b3RecDispatch_ShapeApplyWind( const b3RecArgs_ShapeApplyWind* a, b3RecReader* rdr )
{
	b3Shape_ApplyWind( b3RecMakeShapeId( rdr, a->shape ), a->wind, a->drag, a->lift, a->maxSpeed, a->wake );
}

// Joint creates: remap body ids in the def before calling the API.

static void b3RecDispatch_CreateParallelJoint( const b3RecArgs_CreateParallelJoint* a, b3RecReader* rdr )
{
	b3JointId recId          = b3RecR_JOINTID( rdr );
	b3ParallelJointDef def   = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateParallelJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreateDistanceJoint( const b3RecArgs_CreateDistanceJoint* a, b3RecReader* rdr )
{
	b3JointId recId           = b3RecR_JOINTID( rdr );
	b3DistanceJointDef def    = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateDistanceJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreateFilterJoint( const b3RecArgs_CreateFilterJoint* a, b3RecReader* rdr )
{
	b3JointId recId         = b3RecR_JOINTID( rdr );
	b3FilterJointDef def    = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateFilterJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreateMotorJoint( const b3RecArgs_CreateMotorJoint* a, b3RecReader* rdr )
{
	b3JointId recId         = b3RecR_JOINTID( rdr );
	b3MotorJointDef def     = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateMotorJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreatePrismaticJoint( const b3RecArgs_CreatePrismaticJoint* a, b3RecReader* rdr )
{
	b3JointId recId             = b3RecR_JOINTID( rdr );
	b3PrismaticJointDef def     = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreatePrismaticJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreateRevoluteJoint( const b3RecArgs_CreateRevoluteJoint* a, b3RecReader* rdr )
{
	b3JointId recId            = b3RecR_JOINTID( rdr );
	b3RevoluteJointDef def     = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateRevoluteJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreateSphericalJoint( const b3RecArgs_CreateSphericalJoint* a, b3RecReader* rdr )
{
	b3JointId recId             = b3RecR_JOINTID( rdr );
	b3SphericalJointDef def     = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateSphericalJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreateWeldJoint( const b3RecArgs_CreateWeldJoint* a, b3RecReader* rdr )
{
	b3JointId recId        = b3RecR_JOINTID( rdr );
	b3WeldJointDef def     = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateWeldJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_CreateWheelJoint( const b3RecArgs_CreateWheelJoint* a, b3RecReader* rdr )
{
	b3JointId recId        = b3RecR_JOINTID( rdr );
	b3WheelJointDef def    = a->def;
	def.base.bodyIdA = b3RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b3RecMakeBodyId( rdr, def.base.bodyIdB );
	b3RecCheckJointId( rdr, b3CreateWheelJoint( rdr->replayWorldId, &def ), recId );
}

static void b3RecDispatch_DestroyJoint( const b3RecArgs_DestroyJoint* a, b3RecReader* rdr )
{
	b3DestroyJoint( b3RecMakeJointId( rdr, a->joint ), a->wakeAttached );
}

static void b3RecDispatch_JointSetLocalFrameA( const b3RecArgs_JointSetLocalFrameA* a, b3RecReader* rdr )
{
	b3Joint_SetLocalFrameA( b3RecMakeJointId( rdr, a->joint ), a->localFrame );
}

static void b3RecDispatch_JointSetLocalFrameB( const b3RecArgs_JointSetLocalFrameB* a, b3RecReader* rdr )
{
	b3Joint_SetLocalFrameB( b3RecMakeJointId( rdr, a->joint ), a->localFrame );
}

static void b3RecDispatch_JointSetCollideConnected( const b3RecArgs_JointSetCollideConnected* a, b3RecReader* rdr )
{
	b3Joint_SetCollideConnected( b3RecMakeJointId( rdr, a->joint ), a->shouldCollide );
}

static void b3RecDispatch_JointWakeBodies( const b3RecArgs_JointWakeBodies* a, b3RecReader* rdr )
{
	b3Joint_WakeBodies( b3RecMakeJointId( rdr, a->joint ) );
}

static void b3RecDispatch_JointSetConstraintTuning( const b3RecArgs_JointSetConstraintTuning* a, b3RecReader* rdr )
{
	b3Joint_SetConstraintTuning( b3RecMakeJointId( rdr, a->joint ), a->hertz, a->dampingRatio );
}

static void b3RecDispatch_JointSetForceThreshold( const b3RecArgs_JointSetForceThreshold* a, b3RecReader* rdr )
{
	b3Joint_SetForceThreshold( b3RecMakeJointId( rdr, a->joint ), a->threshold );
}

static void b3RecDispatch_JointSetTorqueThreshold( const b3RecArgs_JointSetTorqueThreshold* a, b3RecReader* rdr )
{
	b3Joint_SetTorqueThreshold( b3RecMakeJointId( rdr, a->joint ), a->threshold );
}

static void b3RecDispatch_ParallelJointSetSpringHertz( const b3RecArgs_ParallelJointSetSpringHertz* a, b3RecReader* rdr )
{
	b3ParallelJoint_SetSpringHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_ParallelJointSetSpringDampingRatio( const b3RecArgs_ParallelJointSetSpringDampingRatio* a,
                                                              b3RecReader* rdr )
{
	b3ParallelJoint_SetSpringDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_ParallelJointSetMaxTorque( const b3RecArgs_ParallelJointSetMaxTorque* a, b3RecReader* rdr )
{
	b3ParallelJoint_SetMaxTorque( b3RecMakeJointId( rdr, a->joint ), a->maxTorque );
}

static void b3RecDispatch_DistanceJointSetLength( const b3RecArgs_DistanceJointSetLength* a, b3RecReader* rdr )
{
	b3DistanceJoint_SetLength( b3RecMakeJointId( rdr, a->joint ), a->length );
}

static void b3RecDispatch_DistanceJointEnableSpring( const b3RecArgs_DistanceJointEnableSpring* a, b3RecReader* rdr )
{
	b3DistanceJoint_EnableSpring( b3RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b3RecDispatch_DistanceJointSetSpringForceRange( const b3RecArgs_DistanceJointSetSpringForceRange* a,
                                                            b3RecReader* rdr )
{
	b3DistanceJoint_SetSpringForceRange( b3RecMakeJointId( rdr, a->joint ), a->lowerForce, a->upperForce );
}

static void b3RecDispatch_DistanceJointSetSpringHertz( const b3RecArgs_DistanceJointSetSpringHertz* a, b3RecReader* rdr )
{
	b3DistanceJoint_SetSpringHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_DistanceJointSetSpringDampingRatio( const b3RecArgs_DistanceJointSetSpringDampingRatio* a,
                                                              b3RecReader* rdr )
{
	b3DistanceJoint_SetSpringDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_DistanceJointEnableLimit( const b3RecArgs_DistanceJointEnableLimit* a, b3RecReader* rdr )
{
	b3DistanceJoint_EnableLimit( b3RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b3RecDispatch_DistanceJointSetLengthRange( const b3RecArgs_DistanceJointSetLengthRange* a, b3RecReader* rdr )
{
	b3DistanceJoint_SetLengthRange( b3RecMakeJointId( rdr, a->joint ), a->minLength, a->maxLength );
}

static void b3RecDispatch_DistanceJointEnableMotor( const b3RecArgs_DistanceJointEnableMotor* a, b3RecReader* rdr )
{
	b3DistanceJoint_EnableMotor( b3RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b3RecDispatch_DistanceJointSetMotorSpeed( const b3RecArgs_DistanceJointSetMotorSpeed* a, b3RecReader* rdr )
{
	b3DistanceJoint_SetMotorSpeed( b3RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b3RecDispatch_DistanceJointSetMaxMotorForce( const b3RecArgs_DistanceJointSetMaxMotorForce* a, b3RecReader* rdr )
{
	b3DistanceJoint_SetMaxMotorForce( b3RecMakeJointId( rdr, a->joint ), a->force );
}

static void b3RecDispatch_MotorJointSetLinearVelocity( const b3RecArgs_MotorJointSetLinearVelocity* a, b3RecReader* rdr )
{
	b3MotorJoint_SetLinearVelocity( b3RecMakeJointId( rdr, a->joint ), a->velocity );
}

static void b3RecDispatch_MotorJointSetAngularVelocity( const b3RecArgs_MotorJointSetAngularVelocity* a, b3RecReader* rdr )
{
	b3MotorJoint_SetAngularVelocity( b3RecMakeJointId( rdr, a->joint ), a->velocity );
}

static void b3RecDispatch_MotorJointSetMaxVelocityForce( const b3RecArgs_MotorJointSetMaxVelocityForce* a, b3RecReader* rdr )
{
	b3MotorJoint_SetMaxVelocityForce( b3RecMakeJointId( rdr, a->joint ), a->maxForce );
}

static void b3RecDispatch_MotorJointSetMaxVelocityTorque( const b3RecArgs_MotorJointSetMaxVelocityTorque* a, b3RecReader* rdr )
{
	b3MotorJoint_SetMaxVelocityTorque( b3RecMakeJointId( rdr, a->joint ), a->maxTorque );
}

static void b3RecDispatch_MotorJointSetLinearHertz( const b3RecArgs_MotorJointSetLinearHertz* a, b3RecReader* rdr )
{
	b3MotorJoint_SetLinearHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_MotorJointSetLinearDampingRatio( const b3RecArgs_MotorJointSetLinearDampingRatio* a, b3RecReader* rdr )
{
	b3MotorJoint_SetLinearDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->damping );
}

static void b3RecDispatch_MotorJointSetAngularHertz( const b3RecArgs_MotorJointSetAngularHertz* a, b3RecReader* rdr )
{
	b3MotorJoint_SetAngularHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_MotorJointSetAngularDampingRatio( const b3RecArgs_MotorJointSetAngularDampingRatio* a,
                                                            b3RecReader* rdr )
{
	b3MotorJoint_SetAngularDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->damping );
}

static void b3RecDispatch_MotorJointSetMaxSpringForce( const b3RecArgs_MotorJointSetMaxSpringForce* a, b3RecReader* rdr )
{
	b3MotorJoint_SetMaxSpringForce( b3RecMakeJointId( rdr, a->joint ), a->maxForce );
}

static void b3RecDispatch_MotorJointSetMaxSpringTorque( const b3RecArgs_MotorJointSetMaxSpringTorque* a, b3RecReader* rdr )
{
	b3MotorJoint_SetMaxSpringTorque( b3RecMakeJointId( rdr, a->joint ), a->maxTorque );
}

static void b3RecDispatch_PrismaticJointEnableSpring( const b3RecArgs_PrismaticJointEnableSpring* a, b3RecReader* rdr )
{
	b3PrismaticJoint_EnableSpring( b3RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b3RecDispatch_PrismaticJointSetSpringHertz( const b3RecArgs_PrismaticJointSetSpringHertz* a, b3RecReader* rdr )
{
	b3PrismaticJoint_SetSpringHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_PrismaticJointSetSpringDampingRatio( const b3RecArgs_PrismaticJointSetSpringDampingRatio* a,
                                                               b3RecReader* rdr )
{
	b3PrismaticJoint_SetSpringDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_PrismaticJointSetTargetTranslation( const b3RecArgs_PrismaticJointSetTargetTranslation* a,
                                                              b3RecReader* rdr )
{
	b3PrismaticJoint_SetTargetTranslation( b3RecMakeJointId( rdr, a->joint ), a->translation );
}

static void b3RecDispatch_PrismaticJointEnableLimit( const b3RecArgs_PrismaticJointEnableLimit* a, b3RecReader* rdr )
{
	b3PrismaticJoint_EnableLimit( b3RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b3RecDispatch_PrismaticJointSetLimits( const b3RecArgs_PrismaticJointSetLimits* a, b3RecReader* rdr )
{
	b3PrismaticJoint_SetLimits( b3RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b3RecDispatch_PrismaticJointEnableMotor( const b3RecArgs_PrismaticJointEnableMotor* a, b3RecReader* rdr )
{
	b3PrismaticJoint_EnableMotor( b3RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b3RecDispatch_PrismaticJointSetMotorSpeed( const b3RecArgs_PrismaticJointSetMotorSpeed* a, b3RecReader* rdr )
{
	b3PrismaticJoint_SetMotorSpeed( b3RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b3RecDispatch_PrismaticJointSetMaxMotorForce( const b3RecArgs_PrismaticJointSetMaxMotorForce* a, b3RecReader* rdr )
{
	b3PrismaticJoint_SetMaxMotorForce( b3RecMakeJointId( rdr, a->joint ), a->force );
}

static void b3RecDispatch_RevoluteJointEnableSpring( const b3RecArgs_RevoluteJointEnableSpring* a, b3RecReader* rdr )
{
	b3RevoluteJoint_EnableSpring( b3RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b3RecDispatch_RevoluteJointSetSpringHertz( const b3RecArgs_RevoluteJointSetSpringHertz* a, b3RecReader* rdr )
{
	b3RevoluteJoint_SetSpringHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_RevoluteJointSetSpringDampingRatio( const b3RecArgs_RevoluteJointSetSpringDampingRatio* a,
                                                              b3RecReader* rdr )
{
	b3RevoluteJoint_SetSpringDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_RevoluteJointSetTargetAngle( const b3RecArgs_RevoluteJointSetTargetAngle* a, b3RecReader* rdr )
{
	b3RevoluteJoint_SetTargetAngle( b3RecMakeJointId( rdr, a->joint ), a->angle );
}

static void b3RecDispatch_RevoluteJointEnableLimit( const b3RecArgs_RevoluteJointEnableLimit* a, b3RecReader* rdr )
{
	b3RevoluteJoint_EnableLimit( b3RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b3RecDispatch_RevoluteJointSetLimits( const b3RecArgs_RevoluteJointSetLimits* a, b3RecReader* rdr )
{
	b3RevoluteJoint_SetLimits( b3RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b3RecDispatch_RevoluteJointEnableMotor( const b3RecArgs_RevoluteJointEnableMotor* a, b3RecReader* rdr )
{
	b3RevoluteJoint_EnableMotor( b3RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b3RecDispatch_RevoluteJointSetMotorSpeed( const b3RecArgs_RevoluteJointSetMotorSpeed* a, b3RecReader* rdr )
{
	b3RevoluteJoint_SetMotorSpeed( b3RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b3RecDispatch_RevoluteJointSetMaxMotorTorque( const b3RecArgs_RevoluteJointSetMaxMotorTorque* a, b3RecReader* rdr )
{
	b3RevoluteJoint_SetMaxMotorTorque( b3RecMakeJointId( rdr, a->joint ), a->torque );
}

static void b3RecDispatch_SphericalJointEnableConeLimit( const b3RecArgs_SphericalJointEnableConeLimit* a, b3RecReader* rdr )
{
	b3SphericalJoint_EnableConeLimit( b3RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b3RecDispatch_SphericalJointSetConeLimit( const b3RecArgs_SphericalJointSetConeLimit* a, b3RecReader* rdr )
{
	b3SphericalJoint_SetConeLimit( b3RecMakeJointId( rdr, a->joint ), a->angleRadians );
}

static void b3RecDispatch_SphericalJointEnableTwistLimit( const b3RecArgs_SphericalJointEnableTwistLimit* a, b3RecReader* rdr )
{
	b3SphericalJoint_EnableTwistLimit( b3RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b3RecDispatch_SphericalJointSetTwistLimits( const b3RecArgs_SphericalJointSetTwistLimits* a, b3RecReader* rdr )
{
	b3SphericalJoint_SetTwistLimits( b3RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b3RecDispatch_SphericalJointEnableSpring( const b3RecArgs_SphericalJointEnableSpring* a, b3RecReader* rdr )
{
	b3SphericalJoint_EnableSpring( b3RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b3RecDispatch_SphericalJointSetSpringHertz( const b3RecArgs_SphericalJointSetSpringHertz* a, b3RecReader* rdr )
{
	b3SphericalJoint_SetSpringHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_SphericalJointSetSpringDampingRatio( const b3RecArgs_SphericalJointSetSpringDampingRatio* a,
                                                               b3RecReader* rdr )
{
	b3SphericalJoint_SetSpringDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_SphericalJointSetTargetRotation( const b3RecArgs_SphericalJointSetTargetRotation* a,
                                                           b3RecReader* rdr )
{
	b3SphericalJoint_SetTargetRotation( b3RecMakeJointId( rdr, a->joint ), a->targetRotation );
}

static void b3RecDispatch_SphericalJointEnableMotor( const b3RecArgs_SphericalJointEnableMotor* a, b3RecReader* rdr )
{
	b3SphericalJoint_EnableMotor( b3RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b3RecDispatch_SphericalJointSetMotorVelocity( const b3RecArgs_SphericalJointSetMotorVelocity* a, b3RecReader* rdr )
{
	b3SphericalJoint_SetMotorVelocity( b3RecMakeJointId( rdr, a->joint ), a->motorVelocity );
}

static void b3RecDispatch_SphericalJointSetMaxMotorTorque( const b3RecArgs_SphericalJointSetMaxMotorTorque* a, b3RecReader* rdr )
{
	b3SphericalJoint_SetMaxMotorTorque( b3RecMakeJointId( rdr, a->joint ), a->torque );
}

static void b3RecDispatch_WeldJointSetLinearHertz( const b3RecArgs_WeldJointSetLinearHertz* a, b3RecReader* rdr )
{
	b3WeldJoint_SetLinearHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_WeldJointSetLinearDampingRatio( const b3RecArgs_WeldJointSetLinearDampingRatio* a, b3RecReader* rdr )
{
	b3WeldJoint_SetLinearDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_WeldJointSetAngularHertz( const b3RecArgs_WeldJointSetAngularHertz* a, b3RecReader* rdr )
{
	b3WeldJoint_SetAngularHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_WeldJointSetAngularDampingRatio( const b3RecArgs_WeldJointSetAngularDampingRatio* a, b3RecReader* rdr )
{
	b3WeldJoint_SetAngularDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_WheelJointEnableSuspension( const b3RecArgs_WheelJointEnableSuspension* a, b3RecReader* rdr )
{
	b3WheelJoint_EnableSuspension( b3RecMakeJointId( rdr, a->joint ), a->flag );
}

static void b3RecDispatch_WheelJointSetSuspensionHertz( const b3RecArgs_WheelJointSetSuspensionHertz* a, b3RecReader* rdr )
{
	b3WheelJoint_SetSuspensionHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_WheelJointSetSuspensionDampingRatio( const b3RecArgs_WheelJointSetSuspensionDampingRatio* a,
                                                               b3RecReader* rdr )
{
	b3WheelJoint_SetSuspensionDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_WheelJointEnableSuspensionLimit( const b3RecArgs_WheelJointEnableSuspensionLimit* a,
                                                           b3RecReader* rdr )
{
	b3WheelJoint_EnableSuspensionLimit( b3RecMakeJointId( rdr, a->joint ), a->flag );
}

static void b3RecDispatch_WheelJointSetSuspensionLimits( const b3RecArgs_WheelJointSetSuspensionLimits* a, b3RecReader* rdr )
{
	b3WheelJoint_SetSuspensionLimits( b3RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b3RecDispatch_WheelJointEnableSpinMotor( const b3RecArgs_WheelJointEnableSpinMotor* a, b3RecReader* rdr )
{
	b3WheelJoint_EnableSpinMotor( b3RecMakeJointId( rdr, a->joint ), a->flag );
}

static void b3RecDispatch_WheelJointSetSpinMotorSpeed( const b3RecArgs_WheelJointSetSpinMotorSpeed* a, b3RecReader* rdr )
{
	b3WheelJoint_SetSpinMotorSpeed( b3RecMakeJointId( rdr, a->joint ), a->speed );
}

static void b3RecDispatch_WheelJointSetMaxSpinTorque( const b3RecArgs_WheelJointSetMaxSpinTorque* a, b3RecReader* rdr )
{
	b3WheelJoint_SetMaxSpinTorque( b3RecMakeJointId( rdr, a->joint ), a->torque );
}

static void b3RecDispatch_WheelJointEnableSteering( const b3RecArgs_WheelJointEnableSteering* a, b3RecReader* rdr )
{
	b3WheelJoint_EnableSteering( b3RecMakeJointId( rdr, a->joint ), a->flag );
}

static void b3RecDispatch_WheelJointSetSteeringHertz( const b3RecArgs_WheelJointSetSteeringHertz* a, b3RecReader* rdr )
{
	b3WheelJoint_SetSteeringHertz( b3RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b3RecDispatch_WheelJointSetSteeringDampingRatio( const b3RecArgs_WheelJointSetSteeringDampingRatio* a,
                                                             b3RecReader* rdr )
{
	b3WheelJoint_SetSteeringDampingRatio( b3RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b3RecDispatch_WheelJointSetMaxSteeringTorque( const b3RecArgs_WheelJointSetMaxSteeringTorque* a, b3RecReader* rdr )
{
	b3WheelJoint_SetMaxSteeringTorque( b3RecMakeJointId( rdr, a->joint ), a->torque );
}

static void b3RecDispatch_WheelJointEnableSteeringLimit( const b3RecArgs_WheelJointEnableSteeringLimit* a, b3RecReader* rdr )
{
	b3WheelJoint_EnableSteeringLimit( b3RecMakeJointId( rdr, a->joint ), a->flag );
}

static void b3RecDispatch_WheelJointSetSteeringLimits( const b3RecArgs_WheelJointSetSteeringLimits* a, b3RecReader* rdr )
{
	b3WheelJoint_SetSteeringLimits( b3RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b3RecDispatch_WheelJointSetTargetSteeringAngle( const b3RecArgs_WheelJointSetTargetSteeringAngle* a,
                                                            b3RecReader* rdr )
{
	b3WheelJoint_SetTargetSteeringAngle( b3RecMakeJointId( rdr, a->joint ), a->radians );
}

static void b3RecDispatch_StateHash( const b3RecArgs_StateHash* a, b3RecReader* rdr )
{
	b3World* world = b3GetWorldFromId( rdr->replayWorldId );
	uint64_t got   = b3HashWorldState( world );
	if ( got != a->hash )
	{
		printf( "b3ValidateReplay: state hash mismatch (recorded=0x%016llX, got=0x%016llX)\n",
		        (unsigned long long)a->hash, (unsigned long long)got );
		rdr->diverged = true;
	}
}

static void b3RecDispatch_RecordingBounds( const b3RecArgs_RecordingBounds* a, b3RecReader* rdr )
{
	(void)a;
	(void)rdr;
	// Informational only; ignore during validation.
}

// X-macro dispatch switch: read opcode+u24 payloadSize, dispatch, skip unknown ops.
// Returns the opcode dispatched, or -1 when the stream is exhausted or broken.

static int b3RecDispatchOne( b3RecReader* rdr )
{
	if ( rdr->cursor >= rdr->size || !rdr->ok )
	{
		return -1;
	}
	uint8_t  opcode      = b3RecR_U8( rdr );
	uint32_t payloadSize = b3RecR_U24( rdr );
	if ( !rdr->ok )
	{
		return -1;
	}
	int payloadStart = rdr->cursor;

	switch ( opcode )
	{
#define ARG( TAG, field ) a.field = b3RecR_##TAG( rdr );
#define B3_REC_OP( op, Name, RET, ... )                                                                                          \
		case op:                                                                                                                     \
		{                                                                                                                             \
			b3RecArgs_##Name a;                                                                                                       \
			memset( &a, 0, sizeof( a ) );                                                                                             \
			__VA_ARGS__                                                                                                               \
			if ( rdr->ok )                                                                                                            \
			{                                                                                                                         \
				b3RecDispatch_##Name( &a, rdr );                                                                                      \
			}                                                                                                                         \
			break;                                                                                                                    \
		}
#include "recording_ops.inl"
#undef B3_REC_OP
#undef ARG
		default:
			// Unknown opcode: skip by jumping to the end of the declared payload
			rdr->cursor = payloadStart + (int)payloadSize;
			break;
	}
	return (int)(unsigned)opcode;
}

// Public entry point

bool b3ValidateReplay( const void* data, int size, int workerCount )
{
	(void)workerCount;

	if ( data == NULL || size < (int)sizeof( b3RecHeader ) )
	{
		printf( "b3ValidateReplay: data too small\n" );
		return false;
	}

	b3RecHeader hdr;
	memcpy( &hdr, data, sizeof( hdr ) );

	if ( hdr.magic != B3_REC_MAGIC )
	{
		printf( "b3ValidateReplay: bad magic 0x%08X\n", hdr.magic );
		return false;
	}
	if ( hdr.versionMajor != B3_REC_VERSION_MAJOR || hdr.versionMinor != B3_REC_VERSION_MINOR )
	{
		printf( "b3ValidateReplay: version mismatch %u.%u vs %u.%u\n",
		        hdr.versionMajor, hdr.versionMinor, B3_REC_VERSION_MAJOR, B3_REC_VERSION_MINOR );
		return false;
	}
	if ( hdr.pointerWidth != sizeof( void* ) )
	{
		printf( "b3ValidateReplay: pointer width mismatch %u vs %u\n", hdr.pointerWidth, (unsigned)sizeof( void* ) );
		return false;
	}
	if ( hdr.bigEndian != 0 )
	{
		printf( "b3ValidateReplay: big-endian recordings not supported\n" );
		return false;
	}
	int headerEnd = (int)sizeof( b3RecHeader );
	int opEnd     = ( hdr.registryOffset != 0 ) ? (int)hdr.registryOffset : size;

	if ( opEnd < headerEnd || opEnd > size )
	{
		printf( "b3ValidateReplay: corrupt registryOffset\n" );
		return false;
	}

	// Preload the trailing registry block
	b3RegistrySlot* slots    = NULL;
	int             slotCount = 0;

	if ( hdr.registryOffset != 0 && hdr.registryByteCount > 0 )
	{
		int regStart = (int)hdr.registryOffset;
		int regEnd   = regStart + (int)hdr.registryByteCount;
		if ( regEnd > size )
		{
			printf( "b3ValidateReplay: registry block out of bounds\n" );
			return false;
		}

		// Parse entry count
		if ( regStart + 4 > size )
		{
			printf( "b3ValidateReplay: registry too small for entry count\n" );
			return false;
		}
		const uint8_t* rp = (const uint8_t*)data + regStart;
		uint32_t count    = (uint32_t)rp[0] | ( (uint32_t)rp[1] << 8 ) | ( (uint32_t)rp[2] << 16 ) |
		                    ( (uint32_t)rp[3] << 24 );
		rp += 4;

		if ( count > 0 )
		{
			slots     = (b3RegistrySlot*)b3Alloc( (size_t)count * sizeof( b3RegistrySlot ) );
			slotCount = (int)count;
			memset( slots, 0, (size_t)count * sizeof( b3RegistrySlot ) );

			for ( uint32_t i = 0; i < count; ++i )
			{
				const uint8_t* dataEnd = (const uint8_t*)data + regEnd;
				if ( rp + 5 > dataEnd )
				{
					printf( "b3ValidateReplay: registry truncated at entry %u\n", i );
					// Partial cleanup
					for ( uint32_t j = 0; j < i; ++j )
					{
						if ( slots[j].bytes != NULL )
						{
							b3Free( slots[j].bytes, (size_t)slots[j].byteCount );
						}
					}
					b3Free( slots, (size_t)count * sizeof( b3RegistrySlot ) );
					return false;
				}
				uint8_t  kind      = rp[0];
				uint32_t byteCount = (uint32_t)rp[1] | ( (uint32_t)rp[2] << 8 ) | ( (uint32_t)rp[3] << 16 ) |
				                     ( (uint32_t)rp[4] << 24 );
				rp += 5;
				if ( rp + byteCount > dataEnd )
				{
					printf( "b3ValidateReplay: registry entry %u bytes out of bounds\n", i );
					for ( uint32_t j = 0; j < i; ++j )
					{
						if ( slots[j].bytes != NULL )
						{
							b3Free( slots[j].bytes, (size_t)slots[j].byteCount );
						}
					}
					b3Free( slots, (size_t)count * sizeof( b3RegistrySlot ) );
					return false;
				}
				// Copy into fresh aligned allocation so structs with uint64_t first field are safe
				uint8_t* bytes = (uint8_t*)b3Alloc( byteCount > 0 ? (size_t)byteCount : 1u );
				if ( byteCount > 0 )
				{
					memcpy( bytes, rp, (size_t)byteCount );
				}
				rp += byteCount;
				slots[i].kind      = (b3GeometryKind)kind;
				slots[i].byteCount = (int)byteCount;
				slots[i].bytes     = bytes;
				slots[i].live      = NULL;
			}
		}
	}

	// Create the replay world
	b3WorldDef worldDef      = b3DefaultWorldDef();
	b3WorldId  replayWorldId = b3CreateWorld( &worldDef );

	b3RecReader rdr;
	memset( &rdr, 0, sizeof( rdr ) );
	rdr.data          = (const uint8_t*)data;
	rdr.size          = size;
	rdr.replayWorldId = replayWorldId;
	rdr.ok            = true;
	rdr.diverged      = false;
	rdr.slots         = slots;
	rdr.slotCount     = slotCount;

	// When a snapshot seed is present, restore the world from it and set the op cursor
	// to just past the snapshot blob. Without a snapshot, start from the header end.
	if ( hdr.snapshotSize > 0 )
	{
		int snapStart = headerEnd;
		int snapSize  = (int)hdr.snapshotSize;
		if ( snapStart + snapSize > size )
		{
			printf( "b3ValidateReplay: snapshot blob out of bounds\n" );
			rdr.ok = false;
		}
		else
		{
			b3World* replayWorld = b3GetWorldFromId( replayWorldId );
			if ( b3DeserializeIntoShell( (const uint8_t*)data + snapStart, snapSize, replayWorld, &rdr ) == false )
			{
				printf( "b3ValidateReplay: snapshot deserialization failed\n" );
				rdr.ok = false;
			}
			else
			{
				rdr.cursor = snapStart + snapSize;
			}
		}
	}
	else
	{
		rdr.cursor = headerEnd;
	}

	// Dispatch op stream
	while ( rdr.cursor < opEnd && rdr.ok )
	{
		int op = b3RecDispatchOne( &rdr );
		if ( op < 0 )
		{
			break;
		}
	}

	bool ok = rdr.ok && !rdr.diverged;

	// Tear down replay world before freeing live geometry
	b3DestroyWorld( replayWorldId );

	// Free live geometry (allocated after world creation, freed after world destruction)
	for ( int i = 0; i < slotCount; ++i )
	{
		b3RegistrySlot* slot = slots + i;
		if ( slot->live != NULL )
		{
			switch ( slot->kind )
			{
				case b3_geometryMesh:
					b3Free( slot->live, (size_t)slot->byteCount );
					break;
				case b3_geometryHeightField:
					b3DestroyHeightField( (b3HeightField*)slot->live );
					break;
				case b3_geometryCompound:
					b3Free( slot->live, (size_t)slot->byteCount );
					break;
				default:
					break;
			}
		}
		if ( slot->bytes != NULL )
		{
			b3Free( slot->bytes, slot->byteCount > 0 ? (size_t)slot->byteCount : 1u );
		}
	}
	if ( slots != NULL )
	{
		b3Free( slots, (size_t)slotCount * sizeof( b3RegistrySlot ) );
	}

	// Free reader scratch
	if ( rdr.matScratch != NULL )
	{
		b3Free( rdr.matScratch, (size_t)rdr.matScratchCap * sizeof( b3SurfaceMaterial ) );
	}

	return ok;
}

// b3RecPlayer implementation

#define B3_REC_KEYFRAME_INTERVAL_DEFAULT 16
#define B3_REC_KEYFRAME_BUDGET_DEFAULT   ( (size_t)512 * 1024 * 1024 )

// Stored snapshot for fast backward seek.
typedef struct b3RecKeyframe
{
	uint8_t* image;       // serialized world image at the end of this frame
	int      imageSize;
	int      imageCapacity; // allocation size (may exceed imageSize)
	int      frame;         // frame index this restores to
	int      cursor;        // op-stream cursor for the frame AFTER this one
	int      divergeFrame;  // divergeFrame state at capture
	bool     diverged;      // rdr.diverged state at capture
} b3RecKeyframe;

struct b3RecPlayer
{
	uint8_t* data;             // owned copy of recording bytes
	int      size;
	int      headerEnd;        // first byte of op stream (past header + snapshot blob)
	int      registryEnd;      // end of op stream = start of registry block (or size)
	float    lengthScale;
	float    previousLengthScale;
	int      frame;
	int      frameCount;
	float    recordedDt;
	int      recordedSubStepCount;
	bool     atEnd;
	int      divergeFrame;     // first frame that diverged, -1 until then
	bool     snapshotSeeded;   // true when snapshotSize > 0

	b3RecReader rdr;

	// Frame-0 restore image: points into owned data when snapshot-seeded,
	// or NULL when from-creation (restart rebuilds world from scratch).
	const uint8_t* frame0Image;
	int            frame0Size;

	// Keyframe ring
	b3RecKeyframe* keyframes;
	int            keyframeCount;
	int            keyframeCapacity;
	size_t         keyframeBudget;
	size_t         keyframeBytes;
	int            keyframeMinInterval;
	int            keyframeInterval;
	int            lastKeyframeFrame;

	// Pre-populated recording used by b3SerializeWorld during keyframe capture.
	// Its registry mirrors rdr.slots so geometry ids stay stable.
	b3Recording*   keyframeRec;
};

// Load the trailing registry block and fill rdr->slots/slotCount.
// Returns true on success. On failure sets rdr->ok = false and returns false.
static bool b3RecLoadSlots( b3RecReader* rdr, const void* data, int size, uint64_t registryOffset, uint64_t registryByteCount )
{
	if ( registryOffset == 0 || registryByteCount == 0 )
	{
		rdr->slots     = NULL;
		rdr->slotCount = 0;
		return true;
	}

	int regStart = (int)registryOffset;
	int regEnd   = regStart + (int)registryByteCount;
	if ( regEnd > size )
	{
		printf( "b3RecPlayer: registry block out of bounds\n" );
		return false;
	}
	if ( regStart + 4 > size )
	{
		printf( "b3RecPlayer: registry too small\n" );
		return false;
	}

	const uint8_t* rp    = (const uint8_t*)data + regStart;
	uint32_t       count = (uint32_t)rp[0] | ( (uint32_t)rp[1] << 8 ) | ( (uint32_t)rp[2] << 16 ) | ( (uint32_t)rp[3] << 24 );
	rp += 4;

	if ( count == 0 )
	{
		rdr->slots     = NULL;
		rdr->slotCount = 0;
		return true;
	}

	b3RegistrySlot* slots = (b3RegistrySlot*)b3Alloc( (size_t)count * sizeof( b3RegistrySlot ) );
	memset( slots, 0, (size_t)count * sizeof( b3RegistrySlot ) );

	const uint8_t* dataEnd = (const uint8_t*)data + regEnd;
	for ( uint32_t i = 0; i < count; ++i )
	{
		if ( rp + 5 > dataEnd )
		{
			printf( "b3RecPlayer: registry truncated at entry %u\n", i );
			for ( uint32_t j = 0; j < i; ++j )
			{
				if ( slots[j].bytes != NULL )
				{
					b3Free( slots[j].bytes, (size_t)slots[j].byteCount );
				}
			}
			b3Free( slots, (size_t)count * sizeof( b3RegistrySlot ) );
			return false;
		}
		uint8_t  kind      = rp[0];
		uint32_t byteCount = (uint32_t)rp[1] | ( (uint32_t)rp[2] << 8 ) | ( (uint32_t)rp[3] << 16 ) | ( (uint32_t)rp[4] << 24 );
		rp += 5;
		if ( rp + byteCount > dataEnd )
		{
			printf( "b3RecPlayer: registry entry %u bytes out of bounds\n", i );
			for ( uint32_t j = 0; j < i; ++j )
			{
				if ( slots[j].bytes != NULL )
				{
					b3Free( slots[j].bytes, (size_t)slots[j].byteCount );
				}
			}
			b3Free( slots, (size_t)count * sizeof( b3RegistrySlot ) );
			return false;
		}
		uint8_t* bytes = (uint8_t*)b3Alloc( byteCount > 0 ? (size_t)byteCount : 1u );
		if ( byteCount > 0 )
		{
			memcpy( bytes, rp, (size_t)byteCount );
		}
		rp += byteCount;
		slots[i].kind      = (b3GeometryKind)kind;
		slots[i].byteCount = (int)byteCount;
		slots[i].bytes     = bytes;
		slots[i].live      = NULL;
	}

	rdr->slots     = slots;
	rdr->slotCount = (int)count;
	return true;
}

// Free slots loaded by b3RecLoadSlots.
static void b3RecFreeSlots( b3RegistrySlot* slots, int slotCount )
{
	if ( slots == NULL )
	{
		return;
	}
	for ( int i = 0; i < slotCount; ++i )
	{
		b3RegistrySlot* slot = slots + i;
		if ( slot->live != NULL )
		{
			switch ( slot->kind )
			{
				case b3_geometryMesh:
					b3Free( slot->live, (size_t)slot->byteCount );
					break;
				case b3_geometryHeightField:
					b3DestroyHeightField( (b3HeightField*)slot->live );
					break;
				case b3_geometryCompound:
					b3Free( slot->live, (size_t)slot->byteCount );
					break;
				default:
					break;
			}
		}
		if ( slot->bytes != NULL )
		{
			b3Free( slot->bytes, slot->byteCount > 0 ? (size_t)slot->byteCount : 1u );
		}
	}
	b3Free( slots, (size_t)slotCount * sizeof( b3RegistrySlot ) );
}

// Walk the op stream once without dispatching: count Step ops and grab the first step's tuning.
static void b3RecScanFile( b3RecPlayer* player )
{
	const uint8_t* data   = player->data;
	int            size   = player->registryEnd;
	int            cursor = player->headerEnd;
	int            frameCount = 0;
	bool           gotStep   = false;

	while ( cursor + 4 <= size )
	{
		uint8_t  opcode      = data[cursor];
		uint32_t payloadSize = (uint32_t)data[cursor + 1] | ( (uint32_t)data[cursor + 2] << 8 ) | ( (uint32_t)data[cursor + 3] << 16 );
		int      payloadStart = cursor + 4;
		if ( payloadStart + (int)payloadSize > size )
		{
			break;
		}
		if ( opcode == 0x80 ) // Step
		{
			frameCount += 1;
			if ( !gotStep && payloadSize >= 12 )
			{
				uint32_t dtBits = (uint32_t)data[payloadStart + 4] | ( (uint32_t)data[payloadStart + 5] << 8 ) |
				                  ( (uint32_t)data[payloadStart + 6] << 16 ) | ( (uint32_t)data[payloadStart + 7] << 24 );
				memcpy( &player->recordedDt, &dtBits, 4 );
				player->recordedSubStepCount = (int)( (uint32_t)data[payloadStart + 8] | ( (uint32_t)data[payloadStart + 9] << 8 ) |
				                                      ( (uint32_t)data[payloadStart + 10] << 16 ) | ( (uint32_t)data[payloadStart + 11] << 24 ) );
				gotStep = true;
			}
		}
		cursor = payloadStart + (int)payloadSize;
	}
	player->frameCount = frameCount;
}

// Free one keyframe's heap.
static void b3RecFreeKeyframe( b3RecKeyframe* kf )
{
	if ( kf->image != NULL )
	{
		b3Free( kf->image, (size_t)kf->imageCapacity );
	}
}

// Pre-populate keyframeRec's registry to mirror rdr.slots so geometry ids stay stable during
// b3SerializeWorld. b3InternGeometry deduplicates so the count stays constant.
static void b3RecSeedKeyframeRegistry( b3RecPlayer* player )
{
	b3GeometryRegistry* reg = &player->keyframeRec->registry;
	for ( int i = 0; i < player->rdr.slotCount; ++i )
	{
		b3RegistrySlot* slot = player->rdr.slots + i;
		// Copy so b3InternGeometry can take ownership (it frees on dedup, which won't happen
		// since we're inserting fresh entries in order).
		int      n    = slot->byteCount > 0 ? slot->byteCount : 1;
		uint8_t* copy = (uint8_t*)b3Alloc( (size_t)n );
		if ( slot->byteCount > 0 )
		{
			memcpy( copy, slot->bytes, (size_t)slot->byteCount );
		}
		uint64_t h  = b3Hash64Blob( slot->bytes, slot->byteCount );
		uint32_t id = b3InternGeometry( reg, slot->kind, h, copy, slot->byteCount );
		// Each slot should get id == its index since we seed in order.
		B3_ASSERT( id == (uint32_t)i );
		(void)id;
	}
}

// Capture a restore-point keyframe for the just-completed frame. rdr.cursor already points to
// the next frame's Step op.
static void b3RecCaptureKeyframe( b3RecPlayer* player )
{
	b3World*   world = b3GetWorldFromId( player->rdr.replayWorldId );
	b3RecBuffer buf  = { 0 };

	int regCountBefore = player->keyframeRec->registry.count;
	b3SerializeWorld( world, &buf, player->keyframeRec );
	// Registry must not grow: all geometry was pre-seeded.
	B3_ASSERT( player->keyframeRec->registry.count == regCountBefore );

	size_t newBytes = (size_t)buf.capacity;

	// Make room under the budget by doubling the spacing and evicting off-grid keyframes.
	while ( player->keyframeCount > 0 && player->keyframeBytes + newBytes > player->keyframeBudget )
	{
		player->keyframeInterval *= 2;
		int    kept      = 0;
		size_t keptBytes = 0;
		for ( int i = 0; i < player->keyframeCount; ++i )
		{
			b3RecKeyframe* kf = player->keyframes + i;
			if ( kf->frame % player->keyframeInterval == 0 )
			{
				player->keyframes[kept] = *kf;
				keptBytes += (size_t)kf->imageCapacity;
				kept += 1;
			}
			else
			{
				b3RecFreeKeyframe( kf );
			}
		}
		bool progress    = ( kept < player->keyframeCount );
		player->keyframeCount = kept;
		player->keyframeBytes = keptBytes;
		if ( !progress )
		{
			break;
		}
	}

	// Grow the keyframe ring if needed.
	if ( player->keyframeCount >= player->keyframeCapacity )
	{
		int newCap = player->keyframeCapacity < 8 ? 8 : player->keyframeCapacity * 2;
		player->keyframes = (b3RecKeyframe*)b3GrowAlloc( player->keyframes,
		                                                  player->keyframeCapacity * (int)sizeof( b3RecKeyframe ),
		                                                  newCap * (int)sizeof( b3RecKeyframe ) );
		player->keyframeCapacity = newCap;
	}

	b3RecKeyframe* kf = player->keyframes + player->keyframeCount;
	kf->image         = buf.data;
	kf->imageSize     = buf.size;
	kf->imageCapacity = buf.capacity;
	kf->frame         = player->frame;
	kf->cursor        = player->rdr.cursor;
	kf->divergeFrame  = player->divergeFrame;
	kf->diverged      = player->rdr.diverged;

	player->keyframeBytes += newBytes;
	player->keyframeCount += 1;
	player->lastKeyframeFrame = player->frame;
}

// Restore the world in-place from a keyframe image.
static void b3RecRestoreKeyframe( b3RecPlayer* player, const b3RecKeyframe* kf )
{
	b3World* world = b3GetWorldFromId( player->rdr.replayWorldId );
	if ( b3DeserializeIntoShell( kf->image, kf->imageSize, world, &player->rdr ) == false )
	{
		player->rdr.ok = false;
		return;
	}
	player->rdr.cursor   = kf->cursor;
	player->rdr.ok       = true;
	player->rdr.diverged = kf->diverged;
	player->frame        = kf->frame;
	player->divergeFrame = kf->divergeFrame;
	player->atEnd        = false;
}

b3RecPlayer* b3RecPlayer_Create( const void* data, int size, int workerCount )
{
	(void)workerCount;

	if ( data == NULL || size < (int)sizeof( b3RecHeader ) )
	{
		printf( "b3RecPlayer_Create: recording too small\n" );
		return NULL;
	}

	b3RecHeader hdr;
	memcpy( &hdr, data, sizeof( hdr ) );

	if ( hdr.magic != B3_REC_MAGIC )
	{
		printf( "b3RecPlayer_Create: bad magic 0x%08X\n", hdr.magic );
		return NULL;
	}
	if ( hdr.versionMajor != B3_REC_VERSION_MAJOR || hdr.versionMinor != B3_REC_VERSION_MINOR )
	{
		printf( "b3RecPlayer_Create: version mismatch %u.%u vs %u.%u\n",
		        hdr.versionMajor, hdr.versionMinor, B3_REC_VERSION_MAJOR, B3_REC_VERSION_MINOR );
		return NULL;
	}
	if ( hdr.pointerWidth != (uint8_t)sizeof( void* ) )
	{
		printf( "b3RecPlayer_Create: pointer width mismatch %u vs %u\n", hdr.pointerWidth, (unsigned)sizeof( void* ) );
		return NULL;
	}
	if ( hdr.bigEndian != 0 )
	{
		printf( "b3RecPlayer_Create: big-endian recording not supported\n" );
		return NULL;
	}

	int headerEnd = (int)sizeof( b3RecHeader ) + (int)hdr.snapshotSize;
	int registryEnd = ( hdr.registryOffset != 0 ) ? (int)hdr.registryOffset : size;

	if ( headerEnd < (int)sizeof( b3RecHeader ) || headerEnd > registryEnd || registryEnd > size )
	{
		printf( "b3RecPlayer_Create: corrupt offsets\n" );
		return NULL;
	}

	// Own a private copy so the caller can free their buffer right away.
	uint8_t* copy = (uint8_t*)b3Alloc( (size_t)size );
	memcpy( copy, data, (size_t)size );

	b3RecPlayer* player = (b3RecPlayer*)b3Alloc( sizeof( b3RecPlayer ) );
	memset( player, 0, sizeof( b3RecPlayer ) );

	player->data              = copy;
	player->size              = size;
	player->headerEnd         = headerEnd;
	player->registryEnd       = registryEnd;
	player->lengthScale       = hdr.lengthScale;
	player->previousLengthScale = b3GetLengthUnitsPerMeter();
	player->snapshotSeeded    = ( hdr.snapshotSize > 0 );
	player->frame             = 0;
	player->frameCount        = 0;
	player->recordedDt        = 0.0f;
	player->recordedSubStepCount = 0;
	player->atEnd             = false;
	player->divergeFrame      = -1;
	player->keyframeMinInterval = B3_REC_KEYFRAME_INTERVAL_DEFAULT;
	player->keyframeInterval    = B3_REC_KEYFRAME_INTERVAL_DEFAULT;
	player->keyframeBudget      = B3_REC_KEYFRAME_BUDGET_DEFAULT;
	player->lastKeyframeFrame   = 0;

	// Set length scale so replay reproduces the same tuning constants.
	if ( hdr.lengthScale > 0.0f )
	{
		b3SetLengthUnitsPerMeter( hdr.lengthScale );
	}

	// Count frames and read first step's dt so the viewer can show hz up front.
	b3RecScanFile( player );

	// Create the replay world.
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId  worldId  = b3CreateWorld( &worldDef );

	// Initialize the reader.
	player->rdr.data          = copy;
	player->rdr.size          = size;
	player->rdr.cursor        = headerEnd;
	player->rdr.replayWorldId = worldId;
	player->rdr.ok            = true;
	player->rdr.diverged      = false;

	// Load the trailing geometry registry.
	if ( !b3RecLoadSlots( &player->rdr, copy, size, hdr.registryOffset, hdr.registryByteCount ) )
	{
		b3DestroyWorld( worldId );
		b3Free( copy, (size_t)size );
		b3Free( player, sizeof( b3RecPlayer ) );
		return NULL;
	}

	// Restore seed snapshot when present; otherwise the world starts empty.
	if ( player->snapshotSeeded )
	{
		int snapStart = (int)sizeof( b3RecHeader );
		int snapSize  = (int)hdr.snapshotSize;
		b3World* replayWorld = b3GetWorldFromId( worldId );
		if ( b3DeserializeIntoShell( copy + snapStart, snapSize, replayWorld, &player->rdr ) == false )
		{
			printf( "b3RecPlayer_Create: snapshot deserialization failed\n" );
			b3DestroyWorld( worldId );
			b3RecFreeSlots( player->rdr.slots, player->rdr.slotCount );
			b3Free( copy, (size_t)size );
			b3Free( player, sizeof( b3RecPlayer ) );
			return NULL;
		}
		player->rdr.cursor  = headerEnd;
		player->frame0Image = copy + snapStart;
		player->frame0Size  = snapSize;
	}

	// Build the keyframe recording with a pre-seeded registry that mirrors rdr.slots,
	// so b3SerializeWorld geometry ids stay stable across captures.
	player->keyframeRec = b3CreateRecording( 0 );
	b3RecSeedKeyframeRegistry( player );

	return player;
}

void b3RecPlayer_Destroy( b3RecPlayer* player )
{
	if ( player == NULL )
	{
		return;
	}

	if ( b3World_IsValid( player->rdr.replayWorldId ) )
	{
		b3DestroyWorld( player->rdr.replayWorldId );
	}

	// Free live geometry after destroying the world (slot->live may be used by the world).
	b3RecFreeSlots( player->rdr.slots, player->rdr.slotCount );

	// Free reader scratch.
	if ( player->rdr.matScratch != NULL )
	{
		b3Free( player->rdr.matScratch, (size_t)player->rdr.matScratchCap * sizeof( b3SurfaceMaterial ) );
	}

	// Free keyframe ring.
	for ( int i = 0; i < player->keyframeCount; ++i )
	{
		b3RecFreeKeyframe( player->keyframes + i );
	}
	if ( player->keyframes != NULL )
	{
		b3Free( player->keyframes, (size_t)player->keyframeCapacity * sizeof( b3RecKeyframe ) );
	}

	// The keyframe recording owns only its buffer and registry; b3DestroyRecording frees both.
	if ( player->keyframeRec != NULL )
	{
		b3DestroyRecording( player->keyframeRec );
	}

	// frame0Image points into the owned data copy, not separately allocated.

	b3Free( player->data, (size_t)player->size );

	// Restore the global length scale.
	b3SetLengthUnitsPerMeter( player->previousLengthScale );

	b3Free( player, sizeof( b3RecPlayer ) );
}

bool b3RecPlayer_StepFrame( b3RecPlayer* player )
{
	if ( player->atEnd )
	{
		return false;
	}

	bool stepped = false;
	for ( ;; )
	{
		if ( player->rdr.cursor >= player->registryEnd || !player->rdr.ok )
		{
			player->atEnd = true;
			return stepped;
		}

		// Peek the opcode. If we've already stepped and the next op is another Step, this
		// frame is complete. Capture a keyframe here before returning.
		if ( stepped && player->rdr.data[player->rdr.cursor] == 0x80 )
		{
			if ( player->frame > player->lastKeyframeFrame && player->frame % player->keyframeInterval == 0 )
			{
				b3RecCaptureKeyframe( player );
			}
			return true;
		}

		int op = b3RecDispatchOne( &player->rdr );
		if ( op < 0 )
		{
			player->atEnd = true;
			return stepped;
		}
		if ( op == 0x01 ) // DestroyWorld — end of recording
		{
			player->atEnd = true;
			return stepped;
		}
		if ( op == 0x80 ) // Step
		{
			player->frame += 1;
			stepped = true;
			// Latch the first frame that diverged.
			if ( player->divergeFrame < 0 && player->rdr.diverged )
			{
				player->divergeFrame = player->frame;
			}
		}
	}
}

void b3RecPlayer_Restart( b3RecPlayer* player )
{
	if ( player->snapshotSeeded )
	{
		// Restore the frame-0 image in-place; replay world id stays stable.
		b3World* world = b3GetWorldFromId( player->rdr.replayWorldId );
		if ( b3DeserializeIntoShell( player->frame0Image, player->frame0Size, world, &player->rdr ) == false )
		{
			player->rdr.ok = false;
			return;
		}
	}
	else
	{
		// From-creation: destroy and re-create the world.
		b3DestroyWorld( player->rdr.replayWorldId );
		b3WorldDef worldDef         = b3DefaultWorldDef();
		player->rdr.replayWorldId   = b3CreateWorld( &worldDef );
	}
	player->rdr.cursor   = player->headerEnd;
	player->rdr.ok       = true;
	player->rdr.diverged = false;
	player->frame        = 0;
	player->divergeFrame = -1;
	player->atEnd        = false;
}

void b3RecPlayer_SeekFrame( b3RecPlayer* player, int targetFrame )
{
	if ( player == NULL )
	{
		return;
	}
	if ( targetFrame < 0 )
	{
		targetFrame = 0;
	}

	// Find the best keyframe strictly before the target.
	const b3RecKeyframe* best = NULL;
	for ( int i = 0; i < player->keyframeCount; ++i )
	{
		const b3RecKeyframe* kf = player->keyframes + i;
		if ( kf->frame < targetFrame && ( best == NULL || kf->frame > best->frame ) )
		{
			best = kf;
		}
	}

	if ( targetFrame < player->frame )
	{
		// Backward seek: restore keyframe or restart from frame 0.
		if ( best != NULL )
		{
			b3RecRestoreKeyframe( player, best );
		}
		else
		{
			b3RecPlayer_Restart( player );
		}
	}
	else if ( best != NULL && best->frame > player->frame )
	{
		// Forward seek that can skip ahead via a keyframe.
		b3RecRestoreKeyframe( player, best );
	}

	while ( player->frame < targetFrame && b3RecPlayer_StepFrame( player ) )
	{
	}
}

b3WorldId b3RecPlayer_GetWorldId( const b3RecPlayer* player )
{
	return player != NULL ? player->rdr.replayWorldId : b3_nullWorldId;
}

int b3RecPlayer_GetFrame( const b3RecPlayer* player )
{
	return player != NULL ? player->frame : 0;
}

int b3RecPlayer_GetFrameCount( const b3RecPlayer* player )
{
	return player != NULL ? player->frameCount : 0;
}

bool b3RecPlayer_IsAtEnd( const b3RecPlayer* player )
{
	return player != NULL ? player->atEnd : true;
}

bool b3RecPlayer_HasDiverged( const b3RecPlayer* player )
{
	return player != NULL ? player->rdr.diverged : false;
}

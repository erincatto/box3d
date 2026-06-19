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

static void b3RecDispatchOne( b3RecReader* rdr )
{
	uint8_t  opcode      = b3RecR_U8( rdr );
	uint32_t payloadSize = b3RecR_U24( rdr );
	if ( !rdr->ok )
	{
		return;
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
		b3RecDispatchOne( &rdr );
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

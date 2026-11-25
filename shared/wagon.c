// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "wagon.h"

#include "box3d/box3d.h"

#include <float.h>
#include <math.h>

const char* g_wagonGaitNames[e_wagonGaitCount] = { "back", "halt", "walk", "trot", "canter", "gallop" };

#define WAGON_RING_COUNT 12

WagonTuning GetDefaultWagonTuning( void )
{
	WagonTuning tuning = { 0 };

	// Enough travel that bumps land on the spring rather than the bump stop. The empty wagon rides
	// about a third of the way into it, leaving two thirds for compression.
	tuning.suspensionTravel = 0.2f;
	tuning.suspensionRestLength = 0.30f;
	tuning.suspensionHertz = 2.0f;

	// Light bump damping lets a wheel climb an edge without slamming the chassis up. Heavy rebound
	// damping keeps it from bouncing after.
	tuning.suspensionDampingRatio = 1.0f;
	tuning.bumpDampingRatio = 0.15f;
	tuning.maxSuspensionForce = 1.0f;

	// A wheel that bottoms out on a rock at speed sinks into it a little instead of throwing the
	// chassis. The axle capsules are the hard stop behind it.
	tuning.bumpStopHertz = 5.0f;
	tuning.bumpStopDampingRatio = 1.0f;

	// Iron tires on a dirt road. The wagon tips at half track over center of mass height, about 0.9 g,
	// so the tires slide well before that.
	tuning.lateralFriction = 0.55f;
	tuning.rollingFriction = 0.55f;
	tuning.rollingResistance = 0.02f;

	// Enough to lock the rear wheels, so the brake is limited by the tire grip
	tuning.brakeForce = 0.5f;
	tuning.tireForceHeight = 0.0f;

	tuning.draftGain = 5.0f;
	tuning.draft[e_wagonBack] = 0.1f;
	tuning.draft[e_wagonHalt] = 0.0f;
	tuning.draft[e_wagonWalk] = 0.1f;
	tuning.draft[e_wagonTrot] = 0.15f;
	tuning.draft[e_wagonCanter] = 0.2f;
	tuning.draft[e_wagonGallop] = 0.25f;
	tuning.holdback = 0.15f;

	// A draft horse can briefly pull well over its working draft, enough to roll a wheel over a log
	// or start a stalled wagon up a steep pitch
	tuning.strainDraft = 0.6f;
	tuning.strainSpeed = 0.5f;
	tuning.strainDuration = 3.0f;
	tuning.strainRecovery = 6.0f;

	tuning.gaitSpeed[e_wagonBack] = -1.0f;
	tuning.gaitSpeed[e_wagonHalt] = 0.0f;
	tuning.gaitSpeed[e_wagonWalk] = 1.6f;
	tuning.gaitSpeed[e_wagonTrot] = 4.0f;
	tuning.gaitSpeed[e_wagonCanter] = 7.0f;
	tuning.gaitSpeed[e_wagonGallop] = 12.0f;

	tuning.maxSteer = 0.25f * B3_PI;

	// A horse will not take a turn harder than this, so the cramp angle closes as the speed rises
	tuning.maxLateralAcceleration = 0.35f;

	// A horse at the end of 2.5 m shafts sidesteps about 1.3 m/s at this rate
	tuning.steerRate = B3_PI / 6.0f;

	// A wheel first touches a log as tall as its hub at a normal near level. Rejecting that lets the
	// wheel sink in until the normal is shallow, and then the bump stop kicks.
	tuning.maxGroundAngle = 85.0f * B3_PI / 180.0f;

	tuning.rollHertz = 1.0f;
	tuning.rollDampingRatio = 1.0f;
	tuning.maxRollTorque = 1.0f;

	tuning.horsePitchHertz = 3.0f;
	tuning.horsePitchDampingRatio = 1.0f;
	return tuning;
}

WagonDef GetDefaultWagonDef( void )
{
	WagonDef def = { 0 };
	def.position = (b3Pos){ 0.0f, 0.0f, 0.0f };
	def.rotation = b3Quat_identity;
	def.mass = 700.0f;
	def.categoryBits = 0x4000000000000000ull;
	def.tuning = GetDefaultWagonTuning();
	return def;
}

static b3Pos GetKingbolt( const Wagon* wagon )
{
	return b3Body_GetWorldPoint( wagon->chassisId, wagon->kingbolt );
}

// Horizontal heading of the front gear
static b3Vec3 GetHeading( const Wagon* wagon )
{
	b3Quat q = b3Body_GetRotation( wagon->chassisId );
	b3Vec3 heading =
		b3RotateVector( b3MulQuat( q, b3MakeQuatFromAxisAngle( b3Vec3_axisY, wagon->steer ) ), b3Vec3_axisX );
	heading.y = 0.0f;
	return b3Normalize( heading );
}

// The ground under a hoof, straight down along up
static bool CastHoof( const Wagon* wagon, b3Pos point, b3Vec3 up, b3Pos* hoof )
{
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.categoryBits = wagon->categoryBits;
	filter.maskBits = ~wagon->categoryBits;

	// Start well above the tugs so a hoof on a rising slope begins outside the ground
	float above = 2.0f * wagon->tugHeight;
	float below = 2.0f * wagon->tugHeight + wagon->tugDistance;
	b3Pos origin = b3OffsetPos( point, b3MulSV( above, up ) );
	b3RayResult ray = b3World_CastRayClosest( wagon->worldId, origin, b3MulSV( -( above + below ), up ), filter );
	if ( ray.hit == false || ray.fraction <= 0.0f )
	{
		return false;
	}

	*hoof = ray.point;
	return true;
}

// Implicit Euler on a damped spring, stable at any step. A zero step snaps to the target.
static void SpringToward( float* value, float* rate, float target, float hertz, float dampingRatio, float timeStep )
{
	if ( timeStep == 0.0f || hertz <= 0.0f )
	{
		*value = target;
		*rate = 0.0f;
		return;
	}

	float omega = 2.0f * B3_PI * hertz;
	float h = timeStep;
	*rate = ( *rate + h * omega * omega * ( target - *value ) ) /
			( 1.0f + 2.0f * dampingRatio * omega * h + omega * omega * h * h );
	*value += h * *rate;
}

// The shafts hinge at the front gear and rest on the horse, so their pitch is where the tugs sit at the
// horse's back height over the ground under its hooves. The hooves move as the pitch changes, so this
// iterates. The horse stays upright and does not roll with a camber.
static void UpdateHorse( Wagon* wagon, float timeStep )
{
	b3Vec3 up = b3Normalize( b3Neg( b3World_GetGravity( wagon->worldId ) ) );
	b3Pos kingbolt = GetKingbolt( wagon );
	b3Vec3 heading = GetHeading( wagon );
	float maxSine = sinf( wagon->maxShaftPitch );

	float pitch = wagon->targetShaftPitch;
	float groundHeight = -wagon->tugHeight;
	float fore = groundHeight, hind = groundHeight;
	b3Pos hooves[2];
	bool grounded[2] = { false, false };

	for ( int iteration = 0; iteration < 3; ++iteration )
	{
		float reach = wagon->tugDistance * cosf( pitch );
		b3Pos under = b3OffsetPos( kingbolt, b3MulSV( reach, heading ) );
		b3Pos feet[2] = {
			b3OffsetPos( under, b3MulSV( wagon->halfStride, heading ) ),
			b3OffsetPos( under, b3MulSV( -wagon->halfStride, heading ) ),
		};

		float heights[2];
		for ( int i = 0; i < 2; ++i )
		{
			grounded[i] = CastHoof( wagon, feet[i], up, hooves + i );
			heights[i] = grounded[i] ? b3Dot( b3SubPos( hooves[i], kingbolt ), up ) : 0.0f;
		}

		if ( grounded[0] == false && grounded[1] == false )
		{
			break;
		}

		fore = grounded[0] ? heights[0] : heights[1];
		hind = grounded[1] ? heights[1] : heights[0];
		groundHeight = 0.5f * ( fore + hind );

		float sine = b3ClampFloat( ( groundHeight + wagon->tugHeight ) / wagon->tugDistance, -maxSine, maxSine );
		pitch = asinf( sine );
	}

	const WagonTuning* tuning = &wagon->tuning;
	wagon->targetShaftPitch = pitch;
	SpringToward( &wagon->shaftPitch, &wagon->shaftPitchRate, pitch, tuning->horsePitchHertz,
				  tuning->horsePitchDampingRatio, timeStep );
	pitch = b3ClampFloat( wagon->shaftPitch, -wagon->maxShaftPitch, wagon->maxShaftPitch );
	wagon->shaftDirection = b3Blend2( cosf( pitch ), heading, sinf( pitch ), up );

	// The body hangs from the sprung tugs and the legs reach the hooves
	float reach = wagon->tugDistance * cosf( pitch );
	float rootHeight = wagon->tugDistance * sinf( pitch ) - wagon->tugHeight;
	b3Pos root = b3OffsetPos( kingbolt, b3MulAdd( b3MulSV( reach, heading ), rootHeight, up ) );

	// Yaw about y to the heading, then pitch the body to its hooves
	float yaw = atan2f( -heading.z, heading.x );
	float bodyPitch = atan2f( fore - hind, 2.0f * wagon->halfStride );
	SpringToward( &wagon->horsePitch, &wagon->horsePitchRate, bodyPitch, tuning->horsePitchHertz,
				  tuning->horsePitchDampingRatio, timeStep );
	b3Quat q = b3MulQuat( b3MakeQuatFromAxisAngle( b3Vec3_axisY, yaw ),
						  b3MakeQuatFromAxisAngle( b3Vec3_axisZ, wagon->horsePitch ) );
	wagon->horseTransform = (b3WorldTransform){ root, q };

	for ( int i = 0; i < 2; ++i )
	{
		float side = i == 0 ? 1.0f : -1.0f;
		float height = i == 0 ? fore : hind;
		wagon->hooves[i] =
			b3OffsetPos( kingbolt, b3MulAdd( b3MulSV( reach + side * wagon->halfStride, heading ), height, up ) );
		wagon->hoofGrounded[i] = grounded[i];
	}
}

// Frame B of a wheel joint, x up the suspension and z along the axle, turned by the front gear
static b3Quat MakeWheelFrame( float steer )
{
	b3Quat upright = b3MakeQuatFromAxisAngle( b3Vec3_axisZ, 0.5f * B3_PI );
	return b3MulQuat( b3MakeQuatFromAxisAngle( b3Vec3_axisY, steer ), upright );
}

static b3Vec3 GetWheelMount( const Wagon* wagon, const WagonWheel* wheel, float steer )
{
	if ( wheel->steered == false )
	{
		return wheel->baseMount;
	}

	// The turntable swings the whole front axle about the kingbolt
	b3Vec3 arm = b3Sub( wheel->baseMount, wagon->kingbolt );
	arm.y = 0.0f;
	b3Vec3 turned = b3RotateVector( b3MakeQuatFromAxisAngle( b3Vec3_axisY, steer ), arm );
	return (b3Vec3){ wagon->kingbolt.x + turned.x, wheel->baseMount.y, wagon->kingbolt.z + turned.z };
}

Wagon CreateWagon( b3WorldId worldId, const WagonDef* def )
{
	Wagon wagon = { 0 };
	wagon.worldId = worldId;
	wagon.categoryBits = def->categoryBits;
	wagon.tuning = def->tuning;
	wagon.halfTrack = 0.725f;
	wagon.shaftLength = 2.75f;

	// A draft horse about 16 hands, the saddle tug a little below the withers
	wagon.tugDistance = 1.65f;
	wagon.tugHeight = 1.1f;
	wagon.halfStride = 0.7f;
	wagon.maxShaftPitch = 0.25f * B3_PI;
	wagon.gait = e_wagonHalt;
	wagon.brake = true;

	float rearRadius = 0.55f;
	float frontRadius = 0.45f;
	float halfWidth = 0.04f;
	float travel = def->tuning.suspensionTravel;

	// The bed sits clear of the front wheels so the turntable can swing them under it
	float bedHalfLength = 1.6f;
	float bedHalfHeight = 0.2f;
	float bedHalfWidth = 0.6f;
	b3Vec3 bedCenter = { -0.1f, 1.15f, 0.0f };

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = def->position;
	bodyDef.rotation = def->rotation;
	bodyDef.name = "wagon";
	wagon.chassisId = b3CreateBody( worldId, &bodyDef );
	wagon.rollAnchorId = b3_nullBodyId;
	wagon.rollJointId = b3_nullJointId;

	float volume = 8.0f * bedHalfLength * bedHalfHeight * bedHalfWidth;
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = def->mass / volume;
	shapeDef.filter.categoryBits = def->categoryBits;
	b3BoxHull bed = b3MakeOffsetBoxHull( bedHalfLength, bedHalfHeight, bedHalfWidth, bedCenter );
	b3CreateHullShape( wagon.chassisId, &shapeDef, &bed.base );

	// The axles are capsules through the wheel mounts out to the outer face of the wheels. A tipped or
	// bottomed wagon rests on them, which keeps every wheel mount, and so every cast origin, clear of
	// the ground. Their radius is larger than the tire half width so a wheel lying flat stays off the
	// ground. They are massless and made before the mass data is set, which a new shape would reset.
	// The front axle stays straight ahead on the body and does not follow the turntable.
	float axleRadius = 0.08f;
	float axleHalfLength = wagon.halfTrack + halfWidth;
	b3ShapeDef axleDef = b3DefaultShapeDef();
	axleDef.density = 0.0f;
	axleDef.filter.categoryBits = def->categoryBits;
	float axleX[2] = { 1.0f, -1.0f };
	float axleY[2] = { frontRadius + travel, rearRadius + travel };
	for ( int i = 0; i < 2; ++i )
	{
		b3Capsule axle = {
			{ axleX[i], axleY[i], -axleHalfLength },
			{ axleX[i], axleY[i], axleHalfLength },
			axleRadius,
		};
		b3CreateCapsuleShape( wagon.chassisId, &axleDef, &axle );
	}
	wagon.axleRadius = axleRadius;

	// Axles, wheels and running gear sit low. Their mass pulls the center well below the bed.
	b3MassData massData = b3Body_GetMassData( wagon.chassisId );
	massData.center = (b3Vec3){ -0.1f, 0.8f, 0.0f };
	b3Body_SetMassData( wagon.chassisId, massData );

	wagon.mass = b3Body_GetMass( wagon.chassisId );
	b3Vec3 gravity = b3World_GetGravity( worldId );
	wagon.weight = wagon.mass * b3Length( gravity );

	// The body origin is on the ground between the axles when every spring is at full extension
	wagon.kingbolt = (b3Vec3){ 1.0f, frontRadius + travel, 0.0f };
	float rearAxle = -1.0f;
	wagon.wheelbase = wagon.kingbolt.x - rearAxle;

	for ( int i = 0; i < e_wagonWheelCount; ++i )
	{
		WagonWheel* wheel = wagon.wheels + i;
		bool front = i == e_wagonFrontLeft || i == e_wagonFrontRight;
		float side = ( i == e_wagonFrontLeft || i == e_wagonRearLeft ) ? -1.0f : 1.0f;
		wheel->radius = front ? frontRadius : rearRadius;
		wheel->halfWidth = halfWidth;
		wheel->steered = front;
		wheel->braked = front == false;
		float x = front ? wagon.kingbolt.x : rearAxle;
		wheel->baseMount = (b3Vec3){ x, wheel->radius + travel, side * wagon.halfTrack };
		wheel->jointId = b3_nullJointId;
		wheel->groundId = b3_nullBodyId;
		wheel->suspensionLength = travel;
	}

	UpdateHorse( &wagon, 0.0f );

	return wagon;
}

static void DestroyRollJoint( Wagon* wagon )
{
	if ( b3Body_IsValid( wagon->rollAnchorId ) )
	{
		b3DestroyBody( wagon->rollAnchorId );
	}

	wagon->rollAnchorId = b3_nullBodyId;
	wagon->rollJointId = b3_nullJointId;
	wagon->rollTorque = 0.0f;
}

void DestroyWagon( Wagon* wagon )
{
	DestroyRollJoint( wagon );

	if ( b3Body_IsValid( wagon->chassisId ) )
	{
		b3DestroyBody( wagon->chassisId );
	}

	*wagon = (Wagon){ 0 };
}

void SetWagonTuning( Wagon* wagon, const WagonTuning* tuning )
{
	wagon->tuning = *tuning;
	wagon->tuningChanged = true;
}

b3Vec3 GetWagonHeading( const Wagon* wagon )
{
	return GetHeading( wagon );
}

b3Pos GetWagonKingbolt( const Wagon* wagon )
{
	return GetKingbolt( wagon );
}

b3Pos GetWagonTug( const Wagon* wagon )
{
	return b3OffsetPos( GetKingbolt( wagon ), b3MulSV( wagon->tugDistance, wagon->shaftDirection ) );
}

typedef struct WheelCastContext
{
	b3ShapeId shapeId;
	b3Pos point;
	b3Vec3 normal;
	float fraction;
	b3Vec3 up;
	float minUpDot;
	bool hit;
} WheelCastContext;

static float WheelCastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
						   int triangleIndex, int childIndex, void* context )
{
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;

	// Keep the closest hit whatever its slope. Skipping a steep hit would skip its whole shape, and
	// on a mesh that drops the ground beside it.
	WheelCastContext* wheelContext = context;
	wheelContext->shapeId = shapeId;
	wheelContext->point = point;
	wheelContext->normal = normal;
	wheelContext->fraction = fraction;
	wheelContext->hit = true;
	return fraction;
}

typedef struct WheelContact
{
	bool hit;

	// The disc began inside the ground and a ray found the surface instead
	bool overlapped;
	b3BodyId groundId;
	b3Pos point;
	b3Vec3 normal;
} WheelContact;

static WheelContact CastWheel( const Wagon* wagon, const WagonWheel* wheel, b3WorldTransform transform, float steer )
{
	b3Vec3 up = b3RotateVector( transform.q, b3Vec3_axisY );
	b3Quat wheelRotation = b3MulQuat( transform.q, b3MakeQuatFromAxisAngle( b3Vec3_axisY, steer ) );
	b3Vec3 forward = b3RotateVector( wheelRotation, b3Vec3_axisX );

	// A disc in the wheel plane, rounded by the tire half width. It finds a curb or rock ahead of the
	// hub with a tilted normal, so the wheel climbs it instead of snapping up.
	float ringRadius = wheel->radius - wheel->halfWidth;
	b3Vec3 points[WAGON_RING_COUNT];

	float deltaAngle = 2.0f * B3_PI / (float)WAGON_RING_COUNT;

	// Compute the trig once here instead of in the loop.
	b3CosSin cs = b3ComputeCosSin( deltaAngle );
	b3Vec2 radial = { ringRadius, 0.0f };

	for ( int i = 0; i < WAGON_RING_COUNT; ++i )
	{
		b3Vec3 p = b3Blend2( radial.x, up, radial.y, forward );
		points[i] = p;

		// Rotate the frame for the next iteration.
		radial = (b3Vec2){
			cs.cosine * radial.x - cs.sine * radial.y,
			cs.sine * radial.x + cs.cosine * radial.y,
		};
	}

	b3ShapeProxy proxy = { points, WAGON_RING_COUNT, wheel->halfWidth };

	// Start above the mount so a wheel pressed past full compression still begins outside the ground,
	// and reach a little past full extension so a landing wheel gets its joint a step early
	float lift = 0.5f * wheel->radius;
	float travel = wagon->tuning.suspensionTravel;
	float reach = lift + travel + 0.05f;

	b3Vec3 mount = GetWheelMount( wagon, wheel, steer );
	b3Pos origin = b3TransformWorldPoint( transform, b3MulAdd( mount, lift, b3Vec3_axisY ) );

	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.categoryBits = wagon->categoryBits;
	filter.maskBits = ~wagon->categoryBits;

	WheelCastContext context = { 0 };
	context.up = up;
	context.minUpDot = cosf( wagon->tuning.maxGroundAngle );
	context.fraction = 1.0f;

	b3World_CastShape( wagon->worldId, origin, &proxy, b3MulSV( -reach, up ), filter, WheelCastFcn, &context );

	WheelContact contact = { 0 };

	if ( context.hit == false )
	{
		return contact;
	}

	// A steep hit is the wheel meeting a log or curb face. It is ground, so the contact starts at first
	// touch and the wheel climbs. Past the limit it is a wall the chassis should hit instead.
	bool overlapped = context.fraction <= 0.0f;

	// The ring is a polygon, so on an edge the cast normal snaps to the facet that touched, 15 or 45
	// degrees for 12 sides, and the facet sits inside the wheel radius. The joint treats the wheel as
	// round. A round wheel's normal runs from the contact to the hub, so rebuild it in the wheel plane
	// and keep the sideways lean of the cast normal.
	//
	// The joint takes the wheel as the cast shape, a ring rounded by the tire half width, so a leaning
	// normal meets the wheel closer to the hub than its radius. Placing the contact at that distance
	// makes the wheel just touch at the cast pose, so the joint sees no false compression.
	b3Vec3 normal = context.normal;
	b3Pos point = context.point;
	if ( overlapped == false )
	{
		b3Vec3 axle = b3Cross( forward, up );
		b3Pos hub = b3OffsetPos( origin, b3MulSV( -context.fraction * reach, up ) );
		b3Vec3 toHub = b3SubPos( hub, point );
		toHub = b3MulSub( toHub, b3Dot( toHub, axle ), axle );
		float distance;
		b3Vec3 inPlane = b3GetLengthAndNormalize( &distance, toHub );
		float lean = b3Dot( normal, axle );
		float upright = sqrtf( b3MaxFloat( 0.0f, 1.0f - lean * lean ) );
		if ( distance > 0.5f * wheel->radius )
		{
			normal = b3Blend2( upright, inPlane, lean, axle );
		}

		float contactRadius = ringRadius * upright + wheel->halfWidth;
		point = b3OffsetPos( hub, b3MulSV( -contactRadius, normal ) );
	}

	bool wall = b3Dot( normal, up ) < context.minUpDot;
	if ( overlapped == false && wall == false )
	{
		contact.hit = true;
		contact.groundId = b3Shape_GetBody( context.shapeId );
		contact.point = point;
		contact.normal = normal;
		return contact;
	}

	contact.overlapped = overlapped;

	// The disc started inside something or brushed a wall, so its normal is no use. A ray from higher up
	// finds the surface under the hub. The joint clamps the wheel to full compression and the bump stop
	// pushes the chassis out.
	b3Pos rayOrigin = b3TransformWorldPoint( transform, b3MulAdd( mount, lift + wheel->radius, b3Vec3_axisY ) );
	b3Vec3 rayTranslation = b3MulSV( -( reach + 2.0f * wheel->radius ), up );
	b3RayResult ray = b3World_CastRayClosest( wagon->worldId, rayOrigin, rayTranslation, filter );
	if ( ray.hit && ray.fraction > 0.0f && b3Dot( ray.normal, up ) >= context.minUpDot )
	{
		contact.hit = true;
		contact.groundId = b3Shape_GetBody( ray.shapeId );
		contact.point = ray.point;
		contact.normal = ray.normal;
	}

	return contact;
}

static void DestroyWheelJoint( WagonWheel* wheel )
{
	if ( b3Joint_IsValid( wheel->jointId ) )
	{
		b3DestroyJoint( wheel->jointId, false );
	}

	wheel->jointId = b3_nullJointId;
	wheel->groundId = b3_nullBodyId;
}

static void UpdateWheelJoint( Wagon* wagon, WagonWheel* wheel, WheelContact contact, float steer, float brakeForce )
{
	bool valid = b3Joint_IsValid( wheel->jointId );
	if ( valid && wagon->tuningChanged == false && B3_ID_EQUALS( wheel->groundId, contact.groundId ) )
	{
		b3Vec3 localPoint = b3Body_GetLocalPoint( contact.groundId, contact.point );
		b3TireJoint_SetContact( wheel->jointId, localPoint, contact.normal );

		if ( wheel->steered && wheel->jointSteer != steer )
		{
			b3Transform frameB = { GetWheelMount( wagon, wheel, steer ), MakeWheelFrame( steer ) };
			b3Joint_SetLocalFrameB( wheel->jointId, frameB );
			wheel->jointSteer = steer;
		}

		if ( wheel->jointBrake != brakeForce )
		{
			b3TireJoint_SetBrakeForce( wheel->jointId, brakeForce );
			wheel->jointBrake = brakeForce;
		}

		return;
	}

	// A new ground body needs a new joint. The impulses carry over so the wheel does not drop.
	b3TireJointDef def = b3DefaultTireJointDef();
	if ( valid )
	{
		def.suspensionImpulse = b3TireJoint_GetSuspensionImpulse( wheel->jointId );
		def.stopImpulse = b3TireJoint_GetStopImpulse( wheel->jointId );
		def.lateralImpulse = b3TireJoint_GetLateralImpulse( wheel->jointId );
		def.rollingImpulse = b3TireJoint_GetRollingImpulse( wheel->jointId );
		DestroyWheelJoint( wheel );
	}

	const WagonTuning* tuning = &wagon->tuning;
	def.base.bodyIdA = contact.groundId;
	def.base.bodyIdB = wagon->chassisId;
	def.base.localFrameA.p = b3Body_GetLocalPoint( contact.groundId, contact.point );
	def.base.localFrameB = (b3Transform){ GetWheelMount( wagon, wheel, steer ), MakeWheelFrame( steer ) };

	// The chassis still collides with the ground under it
	def.base.collideConnected = true;
	def.normal = contact.normal;
	def.radius = wheel->radius;
	def.halfWidth = wheel->halfWidth;
	def.maxSuspensionLength = tuning->suspensionTravel;
	def.restLength = tuning->suspensionRestLength;
	def.suspensionHertz = tuning->suspensionHertz;
	def.suspensionDampingRatio = tuning->suspensionDampingRatio;
	def.bumpDampingRatio = tuning->bumpDampingRatio;
	def.maxSuspensionForce = tuning->maxSuspensionForce * wagon->weight;
	def.bumpStopHertz = tuning->bumpStopHertz;
	def.bumpStopDampingRatio = tuning->bumpStopDampingRatio;
	def.lateralFriction = tuning->lateralFriction;
	def.rollingFriction = tuning->rollingFriction;
	def.rollingResistance = tuning->rollingResistance;
	def.brakeForce = brakeForce;
	def.tireForceHeight = tuning->tireForceHeight;

	wheel->jointId = b3CreateTireJoint( wagon->worldId, &def );
	wheel->groundId = contact.groundId;
	wheel->jointSteer = steer;
	wheel->jointBrake = brakeForce;
}

// The target up is the ground normal with its component along the chassis forward axis removed. That
// leaves pitch free, so the wagon still follows a hill, and it holds the wagon level across a camber
// rather than toward world up, which would load the downhill wheels and help it tip.
static void UpdateRollJoint( Wagon* wagon, b3WorldTransform transform )
{
	const WagonTuning* tuning = &wagon->tuning;
	if ( tuning->maxRollTorque <= 0.0f || tuning->rollHertz <= 0.0f )
	{
		DestroyRollJoint( wagon );
		return;
	}

	b3Vec3 groundUp = b3Vec3_zero;
	for ( int i = 0; i < e_wagonWheelCount; ++i )
	{
		if ( wagon->wheels[i].grounded )
		{
			groundUp = b3Add( groundUp, wagon->wheels[i].contactNormal );
		}
	}

	// A wagon lying on its side has no wheel on the ground, so it rights toward world up
	if ( b3LengthSquared( groundUp ) == 0.0f )
	{
		groundUp = b3Neg( b3World_GetGravity( wagon->worldId ) );
	}

	b3Vec3 forward = b3RotateVector( transform.q, b3Vec3_axisX );
	float length;
	b3Vec3 up = b3GetLengthAndNormalize( &length, b3MulSub( groundUp, b3Dot( groundUp, forward ), forward ) );
	if ( length < 0.1f )
	{
		// Pointing straight up or down a cliff, roll is undefined
		up = wagon->rollUp;
	}

	float maxTorque = tuning->maxRollTorque * wagon->weight * wagon->halfTrack;

	if ( b3Joint_IsValid( wagon->rollJointId ) == false )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = transform.p;
		bodyDef.name = "wagon roll anchor";
		wagon->rollAnchorId = b3CreateBody( wagon->worldId, &bodyDef );

		// The joint holds frame A's z axis on frame B's, so frame B turns z onto the chassis up axis
		b3ParallelJointDef def = b3DefaultParallelJointDef();
		def.base.bodyIdA = wagon->rollAnchorId;
		def.base.bodyIdB = wagon->chassisId;
		def.base.localFrameA.q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, up );
		def.base.localFrameB.p = b3Body_GetMassData( wagon->chassisId ).center;
		def.base.localFrameB.q = b3MakeQuatFromAxisAngle( b3Vec3_axisX, -0.5f * B3_PI );
		def.hertz = tuning->rollHertz;
		def.dampingRatio = tuning->rollDampingRatio;
		def.maxTorque = maxTorque;
		wagon->rollJointId = b3CreateParallelJoint( wagon->worldId, &def );
		wagon->rollUp = up;
		return;
	}

	if ( wagon->tuningChanged )
	{
		b3ParallelJoint_SetSpringHertz( wagon->rollJointId, tuning->rollHertz );
		b3ParallelJoint_SetSpringDampingRatio( wagon->rollJointId, tuning->rollDampingRatio );
		b3ParallelJoint_SetMaxTorque( wagon->rollJointId, maxTorque );
	}

	// Skip tiny changes so a wagon on flat ground records nothing
	if ( b3Dot( up, wagon->rollUp ) < 0.99999f )
	{
		b3Transform frameA = { b3Vec3_zero, b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, up ) };
		b3Joint_SetLocalFrameA( wagon->rollJointId, frameA );
		wagon->rollUp = up;
	}
}

void UpdateWagon( Wagon* wagon, WagonInput input, float timeStep )
{
	const WagonTuning* tuning = &wagon->tuning;

	// Turn radius is about wheelbase over tan(steer), so v^2 tan(steer) / wheelbase is the lateral
	// acceleration the steer asks for
	b3Vec3 forward = b3RotateVector( b3Body_GetRotation( wagon->chassisId ), b3Vec3_axisX );
	float forwardSpeed = b3Dot( b3Body_GetLinearVelocity( wagon->chassisId ), forward );
	float lateralLimit = tuning->maxLateralAcceleration * b3Length( b3World_GetGravity( wagon->worldId ) );
	float maxSteer = tuning->maxSteer;
	float speedSquared = forwardSpeed * forwardSpeed;
	if ( speedSquared * tanf( maxSteer ) > wagon->wheelbase * lateralLimit )
	{
		maxSteer = atanf( wagon->wheelbase * lateralLimit / speedSquared );
	}
	wagon->steerLimit = maxSteer;

	float targetSteer = b3ClampFloat( input.steer, -maxSteer, maxSteer );
	float maxTurn = tuning->steerRate * timeStep;
	float steer = wagon->steer + b3ClampFloat( targetSteer - wagon->steer, -maxTurn, maxTurn );

	bool moving = input.gait != e_wagonHalt;
	bool steering = steer != wagon->steer;
	bool braking = input.brake;

	if ( b3Body_IsAwake( wagon->chassisId ) == false )
	{
		// A parked wagon costs nothing. Only driver input wakes it.
		if ( moving == false && steering == false && braking == wagon->brake && wagon->tuningChanged == false )
		{
			wagon->gait = input.gait;
			return;
		}

		b3Body_SetAwake( wagon->chassisId, true );
	}

	wagon->gait = input.gait;
	wagon->brake = braking;
	wagon->steer = steer;

	b3WorldTransform transform = b3Body_GetTransform( wagon->chassisId );

	for ( int i = 0; i < e_wagonWheelCount; ++i )
	{
		WagonWheel* wheel = wagon->wheels + i;
		float wheelSteer = wheel->steered ? steer : 0.0f;
		float brakeForce = braking && wheel->braked ? tuning->brakeForce * wagon->weight : 0.0f;

		WheelContact contact = CastWheel( wagon, wheel, transform, wheelSteer );
		wheel->grounded = contact.hit;
		wagon->overlapCastCount += contact.overlapped ? 1 : 0;

		if ( contact.hit == false )
		{
			DestroyWheelJoint( wheel );
			continue;
		}

		wheel->contactPoint = contact.point;
		wheel->contactNormal = contact.normal;
		UpdateWheelJoint( wagon, wheel, contact, wheelSteer, brakeForce );
	}

	UpdateRollJoint( wagon, transform );

	wagon->tuningChanged = false;

	UpdateHorse( wagon, timeStep );

	// The horse pulls along the shafts, so it lifts the front gear up a rise and presses it down a dip
	b3Vec3 heading = wagon->shaftDirection;
	b3Pos kingbolt = GetKingbolt( wagon );
	wagon->speed = b3Dot( b3Body_GetWorldPointVelocity( wagon->chassisId, kingbolt ), heading );
	wagon->draftForce = 0.0f;

	float target = tuning->gaitSpeed[input.gait];
	bool stalled = moving && target > 0.0f && wagon->speed < tuning->strainSpeed;
	bool straining = stalled && wagon->strainSpent == false && tuning->strainDuration > 0.0f;
	if ( straining )
	{
		wagon->strainTime += timeStep;
		wagon->strainSpent = wagon->strainTime >= tuning->strainDuration;
	}
	else
	{
		float recoveryRate = tuning->strainRecovery > 0.0f ? tuning->strainDuration / tuning->strainRecovery : FLT_MAX;
		wagon->strainTime = b3MaxFloat( wagon->strainTime - recoveryRate * timeStep, 0.0f );
		wagon->strainSpent = wagon->strainSpent && wagon->strainTime > 0.0f;
	}

	// The horse takes half a second to dig in or ease off, so the pull does not step
	float strainRate = 2.0f * timeStep;
	float strainTarget = straining ? 1.0f : 0.0f;
	wagon->strain += b3ClampFloat( strainTarget - wagon->strain, -strainRate, strainRate );

	if ( moving )
	{
		float draft = tuning->draft[input.gait];
		draft += wagon->strain * b3MaxFloat( tuning->strainDraft - draft, 0.0f );
		float force = wagon->mass * tuning->draftGain * ( target - wagon->speed );
		force = b3ClampFloat( force, -tuning->holdback * wagon->weight, draft * wagon->weight );
		wagon->draftForce = force;
		b3Body_ApplyForce( wagon->chassisId, b3MulSV( force, heading ), kingbolt, true );
	}
}

void UpdateWagonResults( Wagon* wagon, float timeStep )
{
	bool awake = b3Body_IsAwake( wagon->chassisId );
	b3WorldTransform transform = b3Body_GetTransform( wagon->chassisId );

	wagon->rollTorque = 0.0f;
	if ( b3Joint_IsValid( wagon->rollJointId ) )
	{
		wagon->rollTorque = b3Length( b3Joint_GetConstraintTorque( wagon->rollJointId ) );
	}

	for ( int i = 0; i < e_wagonWheelCount; ++i )
	{
		WagonWheel* wheel = wagon->wheels + i;

		if ( b3Joint_IsValid( wheel->jointId ) == false )
		{
			wheel->jointId = b3_nullJointId;
			wheel->groundId = b3_nullBodyId;
			wheel->grounded = false;
			wheel->normalForce = 0.0f;
			wheel->lateralForce = 0.0f;
			wheel->rollingForce = 0.0f;
			wheel->saturated = false;
			wheel->suspensionLength = wagon->tuning.suspensionTravel;
			wheel->spinAngle = b3UnwindAngle( wheel->spinAngle + wheel->spinSpeed * timeStep );
			continue;
		}

		float length = b3TireJoint_GetSuspensionLength( wheel->jointId );
		wheel->suspensionLength = b3ClampFloat( length, 0.0f, wagon->tuning.suspensionTravel );
		wheel->normalForce = b3TireJoint_GetNormalForce( wheel->jointId );
		wheel->lateralForce = b3TireJoint_GetLateralForce( wheel->jointId );
		wheel->rollingForce = b3TireJoint_GetRollingForce( wheel->jointId );
		wheel->saturated = b3TireJoint_IsSaturated( wheel->jointId );

		if ( awake == false )
		{
			wheel->spinSpeed = 0.0f;
			continue;
		}

		// The wheel turns at the ground speed under its hub. A locked wheel slides instead.
		float steer = wheel->steered ? wagon->steer : 0.0f;
		b3Vec3 mount = GetWheelMount( wagon, wheel, steer );
		b3Pos hub = b3TransformWorldPoint( transform, b3MulSub( mount, wheel->suspensionLength, b3Vec3_axisY ) );
		b3Quat wheelRotation = b3MulQuat( transform.q, b3MakeQuatFromAxisAngle( b3Vec3_axisY, steer ) );
		b3Vec3 forward = b3RotateVector( wheelRotation, b3Vec3_axisX );
		float groundSpeed = b3Dot( b3Body_GetWorldPointVelocity( wagon->chassisId, hub ), forward );

		bool locked = wheel->saturated && wagon->brake && wheel->braked;
		wheel->spinSpeed = locked ? 0.0f : -groundSpeed / wheel->radius;
		wheel->spinAngle = b3UnwindAngle( wheel->spinAngle + wheel->spinSpeed * timeStep );
	}
}

b3WorldTransform GetWagonWheelTransform( const Wagon* wagon, int wheelIndex )
{
	const WagonWheel* wheel = wagon->wheels + wheelIndex;
	b3WorldTransform transform = b3Body_GetTransform( wagon->chassisId );
	float steer = wheel->steered ? wagon->steer : 0.0f;
	b3Vec3 mount = GetWheelMount( wagon, wheel, steer );
	b3Vec3 hub = b3MulSub( mount, wheel->suspensionLength, b3Vec3_axisY );

	b3Quat q = b3MulQuat( transform.q, b3MakeQuatFromAxisAngle( b3Vec3_axisY, steer ) );
	q = b3MulQuat( q, b3MakeQuatFromAxisAngle( b3Vec3_axisZ, wheel->spinAngle ) );
	q = b3MulQuat( q, b3MakeQuatFromAxisAngle( b3Vec3_axisX, 0.5f * B3_PI ) );

	return (b3WorldTransform){ b3TransformWorldPoint( transform, hub ), q };
}

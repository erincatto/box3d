// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/collision.h"
#include "box3d/id.h"
#include "box3d/math_functions.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// A horse drawn four wheel wagon, shared by the Vehicle / Wagon sample and the vehicle unit tests.
// The wheels are not bodies. Each wheel is a shape cast and, while it finds ground, a tire joint
// between the ground body and the chassis. The front axle is a turntable: it turns about the kingbolt,
// so steering moves the front wheel mounts as well as turning the wheels. The horse's legs are not
// simulated. It stands on the ground between the shafts, which hinge at the front gear and rest on its
// back, and it pulls with a capped force at the kingbolt along the shafts.
//
// The horse collides through a capsule on its own body. Its legs are a lock to a static anchor that
// holds its height, pitch and roll where the hooves put them, so those loads go to the ground. The
// shafts are two locks to the chassis that only act across the ground plane, one at the kingbolt and
// one sideways at the tugs, so a tree that pushes the horse pushes the front gear and turns the wagon.
// The chassis is one hull over the wheels with its nose wedged into the horse. The horse and the
// hull have their own friction so they can slide past a trunk.
//
// Chassis frame: +x forward, +y up, +z to the right, the axles along z. A tire joint's frame B has
// its x axis along the suspension, pointing up into the chassis, and its z axis along the axle. The
// origin of frame B is the wheel mount, where the hub sits at full compression.

// Back is the horse walking backward, pushing through the breeching. Its speed is negative.
typedef enum WagonGait
{
	e_wagonBack,
	e_wagonHalt,
	e_wagonWalk,
	e_wagonTrot,
	e_wagonCanter,
	e_wagonGallop,
	e_wagonGaitCount
} WagonGait;

extern const char* g_wagonGaitNames[e_wagonGaitCount];

typedef enum WagonWheelIndex
{
	e_wagonFrontLeft,
	e_wagonFrontRight,
	e_wagonRearLeft,
	e_wagonRearRight,
	e_wagonWheelCount
} WagonWheelIndex;

// Everything above the joint that a designer tunes. Forces are multiples of the wagon's weight so a
// heavier wagon keeps its handling.
typedef struct WagonTuning
{
	// Suspension travel from full compression to full extension, meters
	float suspensionTravel;

	// Spring length where the spring force is zero, measured like the travel
	float suspensionRestLength;

	float suspensionHertz;

	// Damping while the wheel extends and while it compresses
	float suspensionDampingRatio;
	float bumpDampingRatio;

	// Per wheel, as a multiple of the wagon weight
	float maxSuspensionForce;

	// Zero is a rigid stop at full compression
	float bumpStopHertz;
	float bumpStopDampingRatio;

	float lateralFriction;
	float rollingFriction;
	float rollingResistance;

	// Per braked wheel, as a multiple of the wagon weight
	float brakeForce;

	// Height above the contact where the tire friction acts on the chassis, meters
	float tireForceHeight;

	// Draft servo gain, 1/s
	float draftGain;

	// Draft and holdback caps as multiples of the wagon weight. Holdback is also the push when backing.
	float draft[e_wagonGaitCount];
	float holdback;

	// A horse pulling forward that is held below the strain speed digs in and pulls up to the strain
	// draft, a multiple of the wagon weight. It can keep that up for the strain duration and needs the
	// recovery time to get it all back. Speed in meters per second, times in seconds.
	float strainDraft;
	float strainSpeed;
	float strainDuration;
	float strainRecovery;

	float gaitSpeed[e_wagonGaitCount];

	// Front gear cramp limit and turn rate, radians and radians per second
	float maxSteer;
	float steerRate;

	// Hardest turn the horse will take, as a multiple of g. It narrows the cramp limit with speed.
	float maxLateralAcceleration;

	// Steeper cast hits are walls, not ground, radians from the chassis up axis
	float maxGroundAngle;

	// A soft parallel joint that holds the chassis level across the ground under its wheels, for a
	// wagon that should never tip over. The torque cap is a multiple of weight times half track,
	// the moment gravity already gives against tipping. Zero turns it off.
	float rollHertz;
	float rollDampingRatio;
	float maxRollTorque;

	// The horse's legs take up the ground, so the shaft and body pitch follow the hooves through a
	// spring rather than snapping to a step
	float horsePitchHertz;
	float horsePitchDampingRatio;

	// Friction of the horse and the wheel rails, low so they slide around trunks
	float bumperFriction;
} WagonTuning;

WagonTuning GetDefaultWagonTuning( void );

// The chassis collision hull in the chassis frame, meters. From the side it is a box from the floor
// to the top with its lower front cut away up to the nose. From above it is full width back of the
// shoulder and narrows to a point at the nose. Fixed at creation.
typedef struct WagonHull
{
	// Back face, behind the rear wheel rims
	float rear;

	// Where the sides start to narrow, ahead of the front wheel rims
	float shoulder;

	// The point of the wedge, inside the tail of the horse capsule
	float nose;

	// Outside the wheels
	float halfWidth;

	// Floor height, kept below the wheel mounts
	float bottom;

	float top;

	// The nose edge runs from here up to the top. Above the shoulder floor, so the lower front slopes
	// back from the horse to the front wheels.
	float noseBottom;
} WagonHull;

WagonHull GetDefaultWagonHull( void );

typedef struct WagonDef
{
	b3Pos position;
	b3Quat rotation;

	// Chassis mass, kilograms
	float mass;

	// Mass of the horse body, kilograms. Its legs carry it, so it only adds to the wagon's inertia
	// across the ground and decides how hard a collision shoves it. Light keeps the tuned handling.
	float horseMass;

	// Category of the wagon's own shapes, which the wheel casts skip
	uint64_t categoryBits;

	WagonHull hull;

	WagonTuning tuning;
} WagonDef;

WagonDef GetDefaultWagonDef( void );

typedef struct WagonInput
{
	WagonGait gait;

	// Target front gear angle, radians, positive turns left
	float steer;

	bool brake;
} WagonInput;

typedef struct WagonWheel
{
	float radius;
	float halfWidth;
	bool steered;
	bool braked;

	// Mount with the front gear straight
	b3Vec3 baseMount;

	b3JointId jointId;
	b3BodyId groundId;

	// The steer angle and brake the joint was last given, so an unchanged wheel records nothing
	float jointSteer;
	float jointBrake;

	bool grounded;
	b3Pos contactPoint;
	b3Vec3 contactNormal;

	// Hub distance below the mount, clamped to the travel
	float suspensionLength;

	float normalForce;
	float lateralForce;
	float rollingForce;
	bool saturated;

	// Rotation about the axle, for drawing only
	float spinAngle;
	float spinSpeed;
} WagonWheel;

typedef struct Wagon
{
	b3WorldId worldId;
	b3BodyId chassisId;
	uint64_t categoryBits;

	WagonTuning tuning;
	bool tuningChanged;

	float mass;
	float weight;

	// Local position of the king bolt.
	b3Vec3 kingbolt;
	float halfTrack;
	float wheelbase;
	float shaftLength;

	// Horse geometry along the shafts. The shafts rest in the tugs on the horse's back, the tug
	// distance from the kingbolt and the tug height above the ground under the hooves.
	float tugDistance;
	float tugHeight;
	float halfStride;

	// The shafts drop less than they rise, so a horse stepping into a drop floats rather than tucking
	// under the bed
	float minShaftPitch;
	float maxShaftPitch;

	// Body pitch limits. A hoof lower than these allow floats.
	float minHorsePitch;
	float maxHorsePitch;

	WagonWheel wheels[e_wagonWheelCount];

	// Anti roll joint to a static anchor and the up axis it last aimed for
	b3BodyId rollAnchorId;
	b3JointId rollJointId;
	b3Vec3 rollUp;
	float rollTorque;

	WagonGait gait;
	bool brake;
	float steer;

	// Cramp limit at the current speed
	float steerLimit;

	float speed;
	float draftForce;

	// How far the horse has dug in, 0 to 1, and the strain time used. A spent horse cannot strain
	// again until it has fully recovered.
	float strain;
	float strainTime;
	bool strainSpent;

	// Wheel casts that started inside the ground and fell back to a ray
	int overlapCastCount;

	// Shaft pitch above the horizontal heading and the shaft direction the draft acts along. The pitch
	// springs toward the target where the tugs sit over the hooves.
	float shaftPitch;
	float shaftPitchRate;
	float targetShaftPitch;
	b3Vec3 shaftDirection;

	float horsePitch;
	float horsePitchRate;

	// Horse root on the ground under the tugs, x forward and y up, pitched to its fore and hind hooves.
	// Hooves 0 and 1 are fore and hind. A hoof that found no ground takes the other's height.
	b3WorldTransform horseTransform;
	b3Pos hooves[2];
	bool hoofGrounded[2];

	// Horse body, its legs on a static anchor and its shafts on the chassis. The capsule is in the
	// horse body frame, whose origin is the horse root. The frames are the joint frames last set, so
	// an unchanged frame records nothing.
	b3BodyId horseId;
	b3BodyId horseAnchorId;
	b3JointId legJointId;
	b3JointId shaftJointId;
	b3JointId tugJointId;
	b3ShapeId horseShapeId;
	b3Capsule horseCapsule;
	b3Transform legFrameA;
	b3Transform legFrameB;
	b3Transform shaftFrameB;
	b3Transform tugFrameA;
	b3Transform tugFrameB;

	b3ShapeId chassisShapeId;

	// How far the shafts give at the kingbolt and the horse's heading off the front gear, meters and
	// radians, and the horse's touching contacts
	float horseOffset;
	float horseOffsetAngle;
	int horseContactCount;
} Wagon;

Wagon CreateWagon( b3WorldId worldId, const WagonDef* def );
void DestroyWagon( Wagon* wagon );

// Takes effect on the next update. The joints are rebuilt, carrying their impulses.
void SetWagonTuning( Wagon* wagon, const WagonTuning* tuning );

// Call before stepping the world. Casts the wheels, keeps the tire joints in step with the ground and
// applies the draft. A sleeping wagon is skipped unless the input asks it to move.
void UpdateWagon( Wagon* wagon, WagonInput input, float timeStep );

// Call after stepping the world. Reads the joint forces and advances the wheel spin.
void UpdateWagonResults( Wagon* wagon, float timeStep );

// Front gear heading in the horizontal plane
b3Vec3 GetWagonHeading( const Wagon* wagon );

b3Pos GetWagonKingbolt( const Wagon* wagon );

// Where the shafts rest on the horse, from the kingbolt along the shaft direction
b3Pos GetWagonTug( const Wagon* wagon );

// Hub transform for drawing. The rotation carries steering and spin, its y axis along the axle.
b3WorldTransform GetWagonWheelTransform( const Wagon* wagon, int wheelIndex );

#ifdef __cplusplus
}
#endif

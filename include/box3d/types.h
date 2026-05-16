// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "collision.h"
#include "constants.h"
#include "id.h"
#include "math_functions.h"

#include <stdint.h>

#define B3_DEFAULT_CATEGORY_BITS 1
#define B3_DEFAULT_MASK_BITS UINT64_MAX

/// Task interface
/// This is the prototype for a Box3D task. Your task system is expected to run this callback on a worker thread,
/// exactly once per enqueue, passing back the same taskContext pointer supplied to b3EnqueueTaskCallback.
/// @ingroup world
typedef void b3TaskCallback( void* taskContext );

/// These functions can be provided to Box3D to invoke a task system.
/// Returns a pointer to the user's task object. May be nullptr. A nullptr indicates to Box3D that the work was executed
/// serially within the callback and there is no need to call b3FinishTaskCallback. Otherwise the returned
/// value must be non-null will be passed to b3FinishTaskCallback as the userTask.
/// @param task the Box3D task to be called by the scheduler
/// @param taskContext the Box3D context object that the scheduler must pass to the task
/// @param userContext the scheduler context object that is opaque to Box3D
/// @param taskName the Box3D task name that the scheduler can use for diagnostics
/// @ingroup world
typedef void* b3EnqueueTaskCallback( b3TaskCallback* task, void* taskContext, void* userContext, const char* taskName );

/// Finishes a user task object that wraps a Box3D task.
/// @ingroup world
typedef void b3FinishTaskCallback( void* userTask, void* userContext );

// The user needs to be able to create debug draw shapes for multi-pass rendering to work efficiently.
// These user shapes are created and destroyed via callback so they can be bound to shape lifetime and scaling updates.
typedef struct b3DebugShape b3DebugShape;
typedef void* b3CreateDebugShapeCallback( const b3DebugShape* debugShape, void* userContext );
typedef void b3DestroyDebugShapeCallback( void* userShape, void* userContext );

/// Optional friction mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box3D state or user application state.
typedef float b3FrictionCallback( float frictionA, uint64_t userMaterialIdA, float frictionB, uint64_t userMaterialIdB );

/// Optional restitution mixing callback. This intentionally provides no context objects because this is called
/// from a worker thread.
/// @warning This function should not attempt to modify Box3D state or user application state.
typedef float b3RestitutionCallback( float restitutionA, uint64_t userMaterialIdA, float restitutionB, uint64_t userMaterialIdB );

/// Result from b3World_RayCastClosest
/// @ingroup world
typedef struct b3RayResult
{
	b3ShapeId shapeId;
	b3Vec3 point;
	b3Vec3 normal;
	uint64_t userMaterialId;
	float fraction;
	int triangleIndex;
	int childIndex;
	int nodeVisits;
	int leafVisits;
	bool hit;
} b3RayResult;

/// Optional world capacities that can be use to avoid run-time allocations
typedef struct b3Capacity
{
	int staticShapeCount;
	int dynamicShapeCount;
	int staticBodyCount;
	int dynamicBodyCount;
	int contactCount;
} b3Capacity;

/// World definition used to create a simulation world. Must be initialized using b3DefaultWorldDef.
typedef struct b3WorldDef
{
	/// Gravity vector. Box3D has no up-vector defined.
	b3Vec3 gravity;

	/// Restitution speed threshold, usually in m/s. Collisions above this
	/// speed have restitution applied (will bounce).
	float restitutionThreshold;

	/// Hit event speed threshold, usually in m/s. Collisions above this
	/// speed can generate hit events if the shape also enables hit events.
	float hitEventThreshold;

	/// Contact stiffness. Cycles per second. Increasing this increases the speed of overlap recovery, but can introduce jitter.
	float contactHertz;

	/// Contact bounciness. Non-dimensional. You can speed up overlap recovery by decreasing this with
	/// the trade-off that overlap resolution becomes more energetic.
	float contactDampingRatio;

	/// This parameter controls how fast overlap is resolved and usually has units of meters per second. This only
	/// puts a cap on the resolution speed. The resolution speed is increased by increasing the hertz and/or
	/// decreasing the damping ratio.
	float contactSpeed;

	/// Maximum linear speed. Usually meters per second.
	float maximumLinearSpeed;

	/// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
	b3FrictionCallback* frictionCallback;

	/// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
	b3RestitutionCallback* restitutionCallback;

	/// Can bodies go to sleep to improve performance
	bool enableSleep;

	/// Enable continuous collision
	bool enableContinuous;

	/// Number of workers to use with the provided task system. Box3D performs best when using only
	/// performance cores and accessing a single L2 cache. Efficiency cores and hyper-threading provide
	/// little benefit and may even harm performance.
	/// @note Box3D does not create threads. This is the number of threads your applications has created
	/// that you are allocating to b3World_Step.
	/// @warning Do not modify the default value unless you are also providing a task system and providing
	/// task callbacks (enqueueTask and finishTask).
	uint32_t workerCount;

	/// function to spawn task
	b3EnqueueTaskCallback* enqueueTask;

	/// function to finish a task
	b3FinishTaskCallback* finishTask;

	/// User context that is provided to enqueueTask and finishTask
	void* userTaskContext;

	/// User data associated with a world
	void* userData;

	/// Used to create debug draw shapes. This is called when a shape is
	/// first drawn using b3DebugDraw.
	b3CreateDebugShapeCallback* createDebugShape;

	/// Used to destroy debug draw shapes. This is called when a shape is modified or destroyed.
	b3DestroyDebugShapeCallback* destroyDebugShape;

	/// This is passed to the debug shape callbacks to provide a user context.
	void* userDebugShapeContext;

	/// Optional initial capacities
	b3Capacity capacity;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3WorldDef;

/// Use this to initialize your world definition
/// @ingroup world
B3_API b3WorldDef b3DefaultWorldDef( void );

/// The body simulation type.
/// Each body is one of these three types. The type determines how the body behaves in the simulation.
/// @ingroup body
typedef enum b3BodyType
{
	/// zero mass, zero velocity, may be manually moved
	b3_staticBody = 0,

	/// zero mass, velocity set by user, moved by solver
	b3_kinematicBody = 1,

	/// positive mass, velocity determined by forces, moved by solver
	b3_dynamicBody = 2,

	/// number of body types
	b3_bodyTypeCount,
} b3BodyType;

/// Motion locks to restrict the body movement
typedef struct b3MotionLocks
{
	/// Prevent translation along the x-axis
	bool linearX;

	/// Prevent translation along the y-axis
	bool linearY;

	/// Prevent translation along the z-axis
	bool linearZ;

	/// Prevent rotation around the x-axis
	bool angularX;

	/// Prevent rotation around the y-axis
	bool angularY;

	/// Prevent rotation around the z-axis
	bool angularZ;
} b3MotionLocks;

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
/// Body definitions are temporary objects used to bundle creation parameters.
/// Must be initialized using b3DefaultBodyDef().
/// @ingroup body
typedef struct b3BodyDef
{
	/// The body type: static, kinematic, or dynamic.
	b3BodyType type;

	/// The initial world position of the body. Bodies should be created with the desired position.
	/// @note Creating bodies at the origin and then moving them nearly doubles the cost of body creation, especially
	/// if the body is moved after shapes have been added.
	b3Vec3 position;

	/// The initial world rotation of the body.
	b3Quat rotation;

	/// The initial linear velocity of the body's origin. Usually in meters per second.
	b3Vec3 linearVelocity;

	/// The initial angular velocity of the body. Radians per second.
	b3Vec3 angularVelocity;

	/// Linear damping is used to reduce the linear velocity. The damping parameter
	/// can be larger than 1 but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Generally linear damping is undesirable because it makes objects move slowly
	/// as if they are floating.
	float linearDamping;

	/// Angular damping is used to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Angular damping can be use slow down rotating bodies.
	float angularDamping;

	/// Scale the gravity applied to this body. Non-dimensional.
	float gravityScale;

	/// Sleep speed threshold, default is 0.05 meters per second
	float sleepThreshold;

	/// Optional body name for debugging. Up to B3_NAME_LENGTH characters (including null termination)
	const char* name;

	/// Use this to store application specific body data.
	void* userData;

	/// Motions locks to restrict linear and angular movement
	b3MotionLocks motionLocks;

	/// Set this flag to false if this body should never fall asleep.
	bool enableSleep;

	/// Is this body initially awake or sleeping?
	bool isAwake;

	/// Treat this body as a high speed object that performs continuous collision detection
	/// against dynamic and kinematic bodies, but not other bullet bodies.
	/// @warning Bullets should be used sparingly. They are not a solution for general dynamic-versus-dynamic
	/// continuous collision. They do not guarantee accurate collision if both bodies are fast moving because
	/// the bullet does a continuous check after all non-bullet bodies have moved. You could get unlucky and have
	/// the bullet body end a time step very close to a non-bullet body and the non-bullet body then moves over
	/// the bullet body. In continuous collision, initial overlap is ignored to avoid freezing bodies in place.
	/// I do not recommend using them for game projectiles if precise collision timing is needed. Instead consider
	/// using a ray or shape cast. You can use a marching ray or shape cast for projectile that moves over time.
	/// If you want a fast moving projectile to collide with a fast moving target, you need to consider the relative
	/// movement in your ray or shape cast. This is out of the scope of Box3D.
	/// So what are good use cases for bullets? Pinball games or games with dynamic containers that hold other objects.
	/// It should be a use case where it doesn't break the game if there is a collision missed, but having them
	/// captured improves the quality of the game.
	bool isBullet;

	/// Used to disable a body. A disabled body does not move or collide.
	bool isEnabled;

	/// This allows this body to bypass rotational speed limits. Should only be used
	/// for circular objects, like wheels.
	bool allowFastRotation;

	/// Enable contact recycling. True by default. Leaving this enabled improves performance
	/// but may lead to ghost collision that should be avoided on characters.
	bool enableContactRecycling;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3BodyDef;

/// Use this to initialize your body definition
/// @ingroup body
B3_API b3BodyDef b3DefaultBodyDef( void );

/// This is used to filter collision on shapes. It affects shape-vs-shape collision
/// and shape-versus-query collision (such as b3World_CastRay).
/// @ingroup shape
typedef struct b3Filter
{
	/// The collision category bits. Normally you would just set one bit. The category bits should
	/// represent your application object types. For example:
	/// @code{.cpp}
	/// enum MyCategories
	/// {
	///    Static  = 0x00000001,
	///    Dynamic = 0x00000002,
	///    Debris  = 0x00000004,
	///    Player  = 0x00000008,
	///    // etc
	/// };
	/// @endcode
	uint64_t categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	/// For example, you may want your player to only collide with static objects
	/// and other players.
	/// @code{.c}
	/// maskBits = Static | Player;
	/// @endcode
	uint64_t maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). A group index of zero has no effect. Non-zero group filtering
	/// always wins against the mask bits.
	/// For example, you may want ragdolls to collide with other ragdolls but you don't want
	/// ragdoll self-collision. In this case you would give each ragdoll a unique negative group index
	/// and apply that group index to all shapes on the ragdoll.
	int groupIndex;
} b3Filter;

/// Use this to initialize your filter
/// @ingroup shape
B3_API b3Filter b3DefaultFilter( void );

/// The query filter is used to filter collisions between queries and shapes. For example,
/// you may want a ray-cast representing a projectile to hit players and the static environment
/// but not debris.
/// @ingroup shape
typedef struct b3QueryFilter
{
	/// The collision category bits of this query. Normally you would just set one bit.
	uint64_t categoryBits;

	/// The collision mask bits. This states the shape categories that this
	/// query would accept for collision.
	uint64_t maskBits;
} b3QueryFilter;

/// Use this to initialize your query filter
/// @ingroup shape
B3_API b3QueryFilter b3DefaultQueryFilter( void );

/// Shape type
/// @ingroup shape
typedef enum b3ShapeType
{
	/// A capsule is an extruded sphere
	b3_capsuleShape,

	/// A compound shape composed of up to 64K spheres, capsules, hulls, and meshes
	b3_compoundShape,

	/// A height field useful for terrain
	b3_heightShape,

	/// A convex hull
	b3_hullShape,

	/// A triangle soup
	b3_meshShape,

	/// A sphere with an offset
	b3_sphereShape,

	/// The number of shape types
	b3_shapeTypeCount
} b3ShapeType;

/// Used to create a shape
typedef struct b3ShapeDef
{
	/// Optional body name for debugging. Up to 31 characters (excluding null termination)
	const char* name;

	/// Use this to store application specific shape data.
	void* userData;

	/// Surface material used on mesh shapes per triangle. Ignored for convex shapes. Ignored for compound shapes.
	b3SurfaceMaterial* materials;

	/// Surface material count.
	int materialCount;

	/// The base surface material. Ignored for compound shapes.
	b3SurfaceMaterial baseMaterial;

	/// The density, usually in kg/m^2.
	float density;

	/// Explosion scale for b3World_Explode. non-dimensional
	float explosionScale;

	/// Contact filtering data.
	b3Filter filter;

	/// Enable custom filtering. Only one of the two shapes needs to enable custom filtering. See b2WorldDef.
	bool enableCustomFiltering;

	/// A sensor shape generates overlap events but never generates a collision response.
	/// Sensors do not have continuous collision. Instead, use a ray or shape cast for those scenarios.
	/// Sensors still contribute to the body mass if they have non-zero density.
	/// @note Sensor events are disabled by default.
	/// @see enableSensorEvents
	bool isSensor;

	/// Enable sensor events for this shape. This applies to sensors and non-sensors. False by default, even for sensors.
	bool enableSensorEvents;

	/// Enable contact events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	bool enableContactEvents;

	/// Enable hit events for this shape. Only applies to kinematic and dynamic bodies. Ignored for sensors. False by default.
	bool enableHitEvents;

	/// Enable pre-solve contact events for this shape. Only applies to dynamic bodies. These are expensive
	///	and must be carefully handled due to multithreading. Ignored for sensors.
	bool enablePreSolveEvents;

	/// When shapes are created they will scan the environment for collision the next time step. This can significantly slow down
	/// static body creation when there are many static shapes.
	/// This is flag is ignored for dynamic and kinematic shapes which always invoke contact creation.
	bool invokeContactCreation;

	/// Should the body update the mass properties when this shape is created. Default is true.
	bool updateBodyMass;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;

} b3ShapeDef;

/// Use this to initialize your shape definition
/// @ingroup shape
B3_API b3ShapeDef b3DefaultShapeDef( void );

//! @cond
/// Profiling data. Times are in milliseconds.
typedef struct b3Profile
{
	float step;
	float pairs;
	float collide;
	float solve;
	float solverSetup;
	float constraints;
	float prepareConstraints;
	float integrateVelocities;
	float warmStart;
	float solveImpulses;
	float integratePositions;
	float relaxImpulses;
	float applyRestitution;
	float storeImpulses;
	float splitIslands;
	float transforms;
	float sensorHits;
	float jointEvents;
	float hitEvents;
	float refit;
	float bullets;
	float sleepIslands;
	float sensors;
} b3Profile;

/// Counters that give details of the simulation size.
typedef struct b3Counters
{
	int bodyCount;
	int shapeCount;
	int contactCount;
	int jointCount;
	int islandCount;
	int stackUsed;
	int arenaCapacity;          // peak demand across all task arenas in bytes
	int staticTreeHeight;
	int treeHeight;
	int satCallCount;
	int satCacheHitCount;
	int byteCount;
	int taskCount;
	int colorCounts[24];
	int manifoldCounts[B3_CONTACT_MANIFOLD_COUNT_BUCKETS];

	// Number of contacts touched by the collide pass (graph contacts + awake-set non-touching).
	int awakeContactCount;

	// Number of contacts recycled in the most recent step.
	int recycledContactCount;

	// Maximum number of time of impact iterations
	int distanceIterations;
	int pushBackIterations;
	int rootIterations;
} b3Counters;
//! @endcond

/// Joint type enumeration
///
/// This is useful because all joint types use b3JointId and sometimes you
/// want to get the type of a joint.
/// @ingroup joint
typedef enum b3JointType
{
	b3_parallelJoint,
	b3_distanceJoint,
	b3_filterJoint,
	b3_motorJoint,
	b3_prismaticJoint,
	b3_revoluteJoint,
	b3_sphericalJoint,
	b3_weldJoint,
	b3_wheelJoint,
} b3JointType;

/// Base joint definition used by all joint types.
/// The local frames are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be
/// 2. if you add/remove shapes from a body and recompute the mass, the joints will be broken
typedef struct b3JointDef
{
	/// User data pointer
	void* userData;

	/// The first attached body
	b3BodyId bodyIdA;

	/// The second attached body
	b3BodyId bodyIdB;

	/// The first local joint frame
	b3Transform localFrameA;

	/// The second local joint frame
	b3Transform localFrameB;

	/// Force threshold for joint events
	float forceThreshold;

	/// Torque threshold for joint events
	float torqueThreshold;

	/// Constraint hertz (advanced feature)
	float constraintHertz;

	/// Constraint damping ratio (advanced feature)
	float constraintDampingRatio;

	/// Debug draw scale
	float drawScale;

	/// Set this flag to true if the attached bodies should collide
	bool collideConnected;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3JointDef;

/// Distance joint definition
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.
/// @ingroup distance_joint
typedef struct b3DistanceJointDef
{
	/// Base joint definition
	b3JointDef base;

	/// The rest length of this joint. Clamped to a stable minimum value.
	float length;

	/// Enable the distance constraint to behave like a spring. If false
	/// then the distance joint will be rigid, overriding the limit and motor.
	bool enableSpring;

	/// The lower spring force controls how much tension it can sustain
	float lowerSpringForce;

	/// The upper spring force controls how much compression it an sustain
	float upperSpringForce;

	/// The spring linear stiffness Hertz, cycles per second
	float hertz;

	/// The spring linear damping ratio, non-dimensional
	float dampingRatio;

	/// Enable/disable the joint limit
	bool enableLimit;

	/// Minimum length. Clamped to a stable minimum value.
	float minLength;

	/// Maximum length. Must be greater than or equal to the minimum length.
	float maxLength;

	/// Enable/disable the joint motor
	bool enableMotor;

	/// The maximum motor force, usually in newtons
	float maxMotorForce;

	/// The desired motor speed, usually in meters per second
	float motorSpeed;
} b3DistanceJointDef;

/// Use this to initialize your joint definition
/// @ingroup distance_joint
B3_API b3DistanceJointDef b3DefaultDistanceJointDef( void );

/// A motor joint is used to control the relative position and velocity between two bodies.
/// @ingroup motor_joint
typedef struct b3MotorJointDef
{
	/// Base joint definition
	b3JointDef base;

	/// The desired linear velocity
	b3Vec3 linearVelocity;

	/// The maximum motor force in newtons
	float maxVelocityForce;

	/// The desired angular velocity
	b3Vec3 angularVelocity;

	/// The maximum motor torque in newton-meters
	float maxVelocityTorque;

	/// Linear spring hertz for position control
	float linearHertz;

	/// Linear spring damping ratio
	float linearDampingRatio;

	/// Maximum spring force in newtons
	float maxSpringForce;

	/// Angular spring hertz for position control
	float angularHertz;

	/// Angular spring damping ratio
	float angularDampingRatio;

	/// Maximum spring torque in newton-meters
	float maxSpringTorque;
} b3MotorJointDef;

/// Use this to initialize your joint definition
/// @ingroup motor_joint
B3_API b3MotorJointDef b3DefaultMotorJointDef( void );

/// A filter joint is used to disable collision between two specific bodies.
///
/// @ingroup filter_joint
typedef struct b3FilterJointDef
{
	/// Base joint definition
	b3JointDef base;
} b3FilterJointDef;

/// Use this to initialize your joint definition
/// @ingroup filter_joint
B3_API b3FilterJointDef b3DefaultFilterJointDef( void );

/// Parallel joint definition
/// Constrains the angle between axis z in body A and axis z in body B
/// using a spring.
/// Useful to keep a body upright.
/// @ingroup cone_joint
typedef struct b3ParallelJointDef
{
	/// Base joint definition
	b3JointDef base;

	float hertz;
	float dampingRatio;
	float maxTorque;

} b3ParallelJointDef;

/// Use this to initialize your joint definition
/// @ingroup parallel_joint
B3_API b3ParallelJointDef b3DefaultParallelJointDef( void );

/// Prismatic joint definition
/// Body B may slide along the x-axis in local frame A. Body B cannot rotate relative to body A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup prismatic_joint
typedef struct b3PrismaticJointDef
{
	/// Base joint definition
	b3JointDef base;

	/// Enable a linear spring along the prismatic joint axis
	bool enableSpring;

	/// The spring stiffness Hertz, cycles per second
	float hertz;

	/// The spring damping ratio, non-dimensional
	float dampingRatio;

	/// The target translation for the joint in meters. The spring-damper will drive
	/// to this translation.
	float targetTranslation;

	/// Enable/disable the joint limit
	bool enableLimit;

	/// The lower translation limit
	float lowerTranslation;

	/// The upper translation limit
	float upperTranslation;

	/// Enable/disable the joint motor
	bool enableMotor;

	/// The maximum motor force, typically in newtons
	float maxMotorForce;

	/// The desired motor speed, typically in meters per second
	float motorSpeed;
} b3PrismaticJointDef;

/// Use this to initialize your joint definition
/// @ingroupd prismatic_joint
B3_API b3PrismaticJointDef b3DefaultPrismaticJointDef( void );

/// Revolute joint definition
/// A point on body B is fixed to a point on body A. Allows relative rotation about the z-axis.
/// @ingroup revolute_joint
typedef struct b3RevoluteJointDef
{
	/// Base joint definition
	b3JointDef base;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	/// This defines the zero angle for the joint limit.
	float targetAngle;

	/// Enable a rotational spring on the revolute hinge axis
	bool enableSpring;

	/// The spring stiffness Hertz, cycles per second
	float hertz;

	/// The spring damping ratio, non-dimensional
	float dampingRatio;

	/// A flag to enable joint limits
	bool enableLimit;

	/// The lower angle for the joint limit in radians
	float lowerAngle;

	/// The upper angle for the joint limit in radians
	float upperAngle;

	/// A flag to enable the joint motor
	bool enableMotor;

	/// The maximum motor torque, typically in newton-meters
	float maxMotorTorque;

	/// The desired motor speed in radians per second
	float motorSpeed;
} b3RevoluteJointDef;

/// Use this to initialize your joint definition.
/// @ingroup revolute_joint
B3_API b3RevoluteJointDef b3DefaultRevoluteJointDef( void );

/// Spherical joint definition
/// A point on body B is fixed to a point on body A. Allows rotation about the shared point.
/// @ingroup spherical_joint
typedef struct b3SphericalJointDef
{
	/// Base joint definition
	b3JointDef base;

	/// Enable a rotational spring that attempts to align the two joint frames.
	bool enableSpring;

	/// The spring stiffness Hertz, cycles per second. This may be clamped internally
	/// according to the time step to maintain stability. Non-negative number.
	float hertz;

	/// The spring damping ratio, non-dimensional. Non-negative number.
	float dampingRatio;

	/// Target spring rotation, joint frame B relative to joint frame A.
	b3Quat targetRotation;

	/// A flag to enable the cone limit. The cone is centered on the frameA z-axis.
	bool enableConeLimit;

	/// The angle for the cone limit in radians. Valid range is [0, pi]
	float coneAngle;

	/// A flag to enable the twist limit. The twist is centered on the frameB z-axis.
	bool enableTwistLimit;

	/// The angle for the lower twist limit in radians. Valid range is [-pi, pi].
	float lowerTwistAngle;

	/// The angle for the upper twist limit in radians. Valid range is [-pi, pi].
	float upperTwistAngle;

	/// A flag to enable the joint motor
	bool enableMotor;

	/// The maximum motor torque, typically in newton-meters. Non-negative number.
	float maxMotorTorque;

	/// The desired motor angular velocity in radians per second.
	b3Vec3 motorVelocity;
} b3SphericalJointDef;

/// Use this to initialize your joint definition.
/// @ingroup spherical_joint
B3_API b3SphericalJointDef b3DefaultSphericalJointDef( void );

/// Weld joint definition
/// Connects two bodies together rigidly. This constraint provides springs to mimic
/// soft-body simulation.
/// @note The approximate solver in Box3D cannot hold many bodies together rigidly
/// @ingroup weld_joint
typedef struct b3WeldJointDef
{
	/// Base joint definition
	b3JointDef base;

	/// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
	float linearHertz;

	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	float angularHertz;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float linearDampingRatio;

	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float angularDampingRatio;
} b3WeldJointDef;

/// Use this to initialize your joint definition
/// @ingroup weld_joint
B3_API b3WeldJointDef b3DefaultWeldJointDef( void );

/// Wheel joint definition
/// Body A is the chassis and body B is the wheel.
/// The wheel rotates around the local z-axis in frame B.
/// The wheel translations along the local x-axis in frame A.
/// The wheel can optionally steer along the x-axis in frame A.
/// @ingroup wheel_joint
typedef struct b3WheelJointDef
{
	/// Base joint definition
	b3JointDef base;

	/// Enable a linear spring along the local axis
	bool enableSuspensionSpring;

	/// Spring stiffness in Hertz
	float suspensionHertz;

	/// Spring damping ratio, non-dimensional
	float suspensionDampingRatio;

	/// Enable/disable the joint linear limit
	bool enableSuspensionLimit;

	/// The lower suspension translation limit
	float lowerSuspensionLimit;

	/// The upper translation limit
	float upperSuspensionLimit;

	/// Enable/disable the joint rotational motor
	bool enableSpinMotor;

	/// The maximum motor torque, typically in newton-meters
	float maxSpinTorque;

	/// The desired motor speed in radians per second
	float spinSpeed;

	/// Enable steering, otherwise the steering is fixed forward
	bool enableSteering;

	/// Steering stiffness in Hertz
	float steeringHertz;

	/// Spring damping ratio, non-dimensional
	float steeringDampingRatio;

	/// The target steering angle in radians
	float targetSteeringAngle;

	/// The maximum steering torque in N*m
	float maxSteeringTorque;

	/// Enable/disable the steering angular limit
	bool enableSteeringLimit;

	/// The lower steering angle in radians
	float lowerSteeringLimit;

	/// The upper steering angle in radians
	float upperSteeringLimit;
} b3WheelJointDef;

/// Use this to initialize your joint definition
/// @ingroup wheel_joint
B3_API b3WheelJointDef b3DefaultWheelJointDef( void );

/// The explosion definition is used to configure options for explosions. Explosions
/// consider shape geometry when computing the impulse.
/// @ingroup world
typedef struct b3ExplosionDef
{
	/// Mask bits to filter shapes
	uint64_t maskBits;

	/// The center of the explosion in world space
	b3Vec3 position;

	/// The radius of the explosion
	float radius;

	/// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
	float falloff;

	/// Impulse per unit area. This applies an impulse according to the shape area that
	/// is facing the explosion. Explosions only apply to spheres, capsules, and hulls. This
	/// may be negative for implosions.
	float impulsePerArea;
} b3ExplosionDef;

/// Use this to initialize your explosion definition
/// @ingroup world
B3_API b3ExplosionDef b3DefaultExplosionDef( void );

/**
 * @defgroup events Events
 * World event types.
 *
 * Events are used to collect events that occur during the world time step. These events
 * are then available to query after the time step is complete. This is preferable to callbacks
 * because Box3D uses multithreaded simulation.
 *
 * Also when events occur in the simulation step it may be problematic to modify the world, which is
 * often what applications want to do when events occur.
 *
 * With event arrays, you can scan the events in a loop and modify the world. However, you need to be careful
 * that some event data may become invalid. There are several samples that show how to do this safely.
 *
 * @{
 */

/// A begin-touch event is generated when a shape starts to overlap a sensor shape.
typedef struct b3SensorBeginTouchEvent
{
	/// The id of the sensor shape
	b3ShapeId sensorShapeId;

	/// The id of the shape that began touching the sensor shape
	b3ShapeId visitorShapeId;
} b3SensorBeginTouchEvent;

/// An end touch event is generated when a shape stops overlapping a sensor shape.
///	These include things like setting the transform, destroying a body or shape, or changing
///	a filter. You will also get an end event if the sensor or visitor are destroyed.
///	Therefore you should always confirm the shape id is valid using b3Shape_IsValid.
typedef struct b3SensorEndTouchEvent
{
	/// The id of the sensor shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	b3ShapeId sensorShapeId;

	/// The id of the shape that stopped touching the sensor shape
	///	@warning this shape may have been destroyed
	///	@see b2Shape_IsValid
	b3ShapeId visitorShapeId;
} b3SensorEndTouchEvent;

/// Sensor events are buffered in the world and are available
///	as begin/end overlap event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
typedef struct b3SensorEvents
{
	/// Array of sensor begin touch events
	b3SensorBeginTouchEvent* beginEvents;

	/// Array of sensor end touch events
	b3SensorEndTouchEvent* endEvents;

	/// The number of begin touch events
	int beginCount;

	/// The number of end touch events
	int endCount;
} b3SensorEvents;

/// A begin-touch event is generated when two shapes begin touching.
typedef struct b3ContactBeginTouchEvent
{
	/// Id of the first shape
	b3ShapeId shapeIdA;

	/// Id of the second shape
	b3ShapeId shapeIdB;

	/// The transient contact id. This contact maybe destroyed automatically when the world is modified or simulated.
	/// Used b3Contact_IsValid before using this id.
	b3ContactId contactId;
} b3ContactBeginTouchEvent;

/// An end touch event is generated when two shapes stop touching.
///	You will get an end event if you do anything that destroys contacts previous to the last
///	world step. These include things like setting the transform, destroying a body
///	or shape, or changing a filter or body type.
typedef struct b3ContactEndTouchEvent
{
	/// Id of the first shape
	///	@warning this shape may have been destroyed
	///	@see b3Shape_IsValid
	b3ShapeId shapeIdA;

	/// Id of the first shape
	///	@warning this shape may have been destroyed
	///	@see b3Shape_IsValid
	b3ShapeId shapeIdB;

	/// Id of the contact.
	///	@warning this contact may have been destroyed
	///	@see b3Contact_IsValid
	b3ContactId contactId;
} b3ContactEndTouchEvent;

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
/// This may be reported for speculative contacts that have a confirmed impulse.
typedef struct b3ContactHitEvent
{
	/// Id of the first shape
	b3ShapeId shapeIdA;

	/// Id of the second shape
	b3ShapeId shapeIdB;

	/// Id of the contact.
	///	@warning this contact may have been destroyed
	///	@see b3Contact_IsValid
	b3ContactId contactId;

	/// Point where the shapes hit at the beginning of the time step.
	/// This is a mid-point between the two surfaces. It could be at speculative
	/// point where the two shapes were not touching at the beginning of the time step.
	b3Vec3 point;

	/// Normal vector pointing from shape A to shape B
	b3Vec3 normal;

	/// The speed the shapes are approaching. Always positive. Typically in meters per second.
	float approachSpeed;

	/// User material on shape A
	uint64_t userMaterialIdA;

	/// User material on shape B
	uint64_t userMaterialIdB;

} b3ContactHitEvent;

/// Contact events are buffered in the world and are available
///	as event arrays after the time step is complete.
///	Note: these may become invalid if bodies and/or shapes are destroyed
typedef struct b3ContactEvents
{
	/// Array of begin touch events
	b3ContactBeginTouchEvent* beginEvents;

	/// Array of end touch events
	b3ContactEndTouchEvent* endEvents;

	/// Array of hit events
	b3ContactHitEvent* hitEvents;

	/// Number of begin touch events
	int beginCount;

	/// Number of end touch events
	int endCount;

	/// Number of hit events
	int hitCount;
} b3ContactEvents;

/// Body move events triggered when a body moves.
/// Triggered when a body moves due to simulation. Not reported for bodies moved by the user.
/// This also has a flag to indicate that the body went to sleep so the application can also
/// sleep that actor/entity/object associated with the body.
/// On the other hand if the flag does not indicate the body went to sleep then the application
/// can treat the actor/entity/object associated with the body as awake.
/// This is an efficient way for an application to update game object transforms rather than
/// calling functions such as b3Body_GetTransform() because this data is delivered as a contiguous array
/// and it is only populated with bodies that have moved.
/// @note If sleeping is disabled all dynamic and kinematic bodies will trigger move events.
typedef struct b3BodyMoveEvent
{
	void* userData;
	b3Transform transform;
	b3BodyId bodyId;
	bool fellAsleep;
} b3BodyMoveEvent;

/// Body events are buffered in the world and are available
///	as event arrays after the time step is complete.
///	Note: this date becomes invalid if bodies are destroyed
typedef struct b3BodyEvents
{
	/// Array of move events
	b3BodyMoveEvent* moveEvents;

	/// Number of move events
	int moveCount;
} b3BodyEvents;

/// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
/// The observed forces and torques are not returned for efficiency reasons.
typedef struct b3JointEvent
{
	/// The joint id
	b3JointId jointId;

	/// The user data from the joint for convenience
	void* userData;
} b3JointEvent;

/// Joint events are buffered in the world and are available
/// as event arrays after the time step is complete.
/// Note: this data becomes invalid if joints are destroyed
typedef struct b3JointEvents
{
	/// Array of events
	b3JointEvent* jointEvents;

	/// Number of events
	int count;
} b3JointEvents;

/// The contact data for two shapes. By convention the manifold normal points
/// from shape A to shape B.
/// @see b3Shape_GetContactData() and b3Body_GetContactData()
typedef struct b3ContactData
{
	b3ContactId contactId;
	b3ShapeId shapeIdA;
	b3ShapeId shapeIdB;
	const struct b3Manifold* manifolds;
	int manifoldCount;
} b3ContactData;

/**@}*/

/// Prototype for a contact filter callback.
/// This is called when a contact pair is considered for collision. This allows you to
/// perform custom logic to prevent collision between shapes. This is only called if
/// one of the two shapes has custom filtering enabled. @see b3ShapeDef.
/// Notes:
/// - this function must be thread-safe
/// - this is only called if one of the two shapes has enabled custom filtering
/// - this is called only for awake dynamic bodies
/// Return false if you want to disable the collision
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
typedef bool b3CustomFilterFcn( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context );

/// Prototype for a pre-solve callback.
/// This is called after a contact is updated. This allows you to inspect a
/// collision before it goes to the solver.
/// Notes:
/// - this function must be thread-safe
/// - this is only called if the shape has enabled pre-solve events
/// - this may be called for awake dynamic bodies and sensors
/// - this is not called for sensors
/// Return false if you want to disable the contact this step
/// This has limited information because it is used during CCD which does not have the
/// full contact manifold.
/// @warning Do not attempt to modify the world inside this callback
/// @ingroup world
typedef bool b3PreSolveFcn( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Vec3 point, b3Vec3 normal, void* context );

/// Prototype callback for overlap queries.
/// Called for each shape found in the query.
/// @see b3World_QueryAABB
/// @return false to terminate the query.
/// @ingroup world
typedef bool b3OverlapResultFcn( b3ShapeId shapeId, void* context );

/// Prototype callback for ray casts.
/// Called for each shape found in the query. You control how the ray cast
/// proceeds by returning a float:
/// return -1: ignore this shape and continue
/// return 0: terminate the ray cast
/// return fraction: clip the ray to this point
/// return 1: don't clip the ray and continue
/// @param shapeId the shape hit by the ray
/// @param point the point of initial intersection
/// @param normal the normal vector at the point of intersection
/// @param fraction the fraction along the ray at the point of intersection
/// @param userMaterialId the shape or triangle surface type
/// @param triangleIndex the triangle index for mesh or height field shapes or -1 for other shape types
/// @param childIndex the child shape index for compound shapes
/// @param context the user context
/// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
/// @see b3World_CastRay
/// @ingroup world
typedef float b3CastResultFcn( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
							   int triangleIndex, int childIndex, void* context );

// Used to collect collision planes for character movers.
// Return true to continue gathering planes.
typedef bool b3PlaneResultFcn( b3ShapeId shapeId, const b3PlaneResult* plane, int planeCount, void* context );

// Used to filter shapes for shape casting character movers.
// Return true to accept the collision
typedef bool b3MoverFilterFcn( b3ShapeId shapeId, void* context );

/// Body ray cast for ray casting a specific body with a specified transform.
typedef struct b3BodyRayCastInput
{
	b3Vec3 origin;
	b3Vec3 translation;
	b3QueryFilter filter;
	float maxFraction;
} b3BodyRayCastInput;

/// Body shape cast for shape casting a specific body with a specified transform.
typedef struct b3BodyShapeCastInput
{
	b3ShapeProxy proxy;
	b3Vec3 translation;
	b3QueryFilter filter;
	float maxFraction;
	bool canEncroach;
} b3BodyShapeCastInput;

/// Body cast result for ray and shape casts.
typedef struct b3BodyCastResult
{
	b3ShapeId shapeId;
	b3Vec3 point;
	b3Vec3 normal;
	float fraction;
	int triangleIndex;
	uint64_t userMaterialId;
	int iterations;
	bool hit;
} b3BodyCastResult;

/// Body plane result for movers.
typedef struct b3BodyPlaneResult
{
	b3ShapeId shapeId;
	b3PlaneResult result;
} b3BodyPlaneResult;

typedef struct b3ChildShape
{
	union
	{
		b3Capsule capsule;
		const b3Hull* hull;
		b3Mesh mesh;
		b3Sphere sphere;
	};

	b3Transform transform;

	// Index 0 is used for convex shapes.
	// todo limit to 64K?
	int materialIndices[B3_MAX_COMPOUND_MESH_MATERIALS];
	b3ShapeType type;
} b3ChildShape;

typedef bool b3CompoundQueryFcn( const b3Compound* compound, int childIndex, void* context );

B3_API b3ChildShape b3GetCompoundChild( const b3Compound* compound, int childIndex );
B3_API void b3QueryCompound( const b3Compound* compound, b3AABB aabb, b3CompoundQueryFcn* fcn, void* context );

/// These colors are used for debug draw and mostly match the named SVG colors.
/// See https://www.rapidtables.com/web/color/index.html
/// https://johndecember.com/html/spec/colorsvg.html
/// https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
typedef enum b3HexColor
{
	b3_colorAliceBlue = 0xF0F8FF,
	b3_colorAntiqueWhite = 0xFAEBD7,
	b3_colorAqua = 0x00FFFF,
	b3_colorAquamarine = 0x7FFFD4,
	b3_colorAzure = 0xF0FFFF,
	b3_colorBeige = 0xF5F5DC,
	b3_colorBisque = 0xFFE4C4,
	b3_colorBlack = 0x000000,
	b3_colorBlanchedAlmond = 0xFFEBCD,
	b3_colorBlue = 0x0000FF,
	b3_colorBlueViolet = 0x8A2BE2,
	b3_colorBrown = 0xA52A2A,
	b3_colorBurlywood = 0xDEB887,
	b3_colorCadetBlue = 0x5F9EA0,
	b3_colorChartreuse = 0x7FFF00,
	b3_colorChocolate = 0xD2691E,
	b3_colorCoral = 0xFF7F50,
	b3_colorCornflowerBlue = 0x6495ED,
	b3_colorCornsilk = 0xFFF8DC,
	b3_colorCrimson = 0xDC143C,
	b3_colorCyan = 0x00FFFF,
	b3_colorDarkBlue = 0x00008B,
	b3_colorDarkCyan = 0x008B8B,
	b3_colorDarkGoldenRod = 0xB8860B,
	b3_colorDarkGray = 0xA9A9A9,
	b3_colorDarkGreen = 0x006400,
	b3_colorDarkKhaki = 0xBDB76B,
	b3_colorDarkMagenta = 0x8B008B,
	b3_colorDarkOliveGreen = 0x556B2F,
	b3_colorDarkOrange = 0xFF8C00,
	b3_colorDarkOrchid = 0x9932CC,
	b3_colorDarkRed = 0x8B0000,
	b3_colorDarkSalmon = 0xE9967A,
	b3_colorDarkSeaGreen = 0x8FBC8F,
	b3_colorDarkSlateBlue = 0x483D8B,
	b3_colorDarkSlateGray = 0x2F4F4F,
	b3_colorDarkTurquoise = 0x00CED1,
	b3_colorDarkViolet = 0x9400D3,
	b3_colorDeepPink = 0xFF1493,
	b3_colorDeepSkyBlue = 0x00BFFF,
	b3_colorDimGray = 0x696969,
	b3_colorDodgerBlue = 0x1E90FF,
	b3_colorFireBrick = 0xB22222,
	b3_colorFloralWhite = 0xFFFAF0,
	b3_colorForestGreen = 0x228B22,
	b3_colorFuchsia = 0xFF00FF,
	b3_colorGainsboro = 0xDCDCDC,
	b3_colorGhostWhite = 0xF8F8FF,
	b3_colorGold = 0xFFD700,
	b3_colorGoldenRod = 0xDAA520,
	b3_colorGray = 0x808080,
	b3_colorGreen = 0x008000,
	b3_colorGreenYellow = 0xADFF2F,
	b3_colorHoneyDew = 0xF0FFF0,
	b3_colorHotPink = 0xFF69B4,
	b3_colorIndianRed = 0xCD5C5C,
	b3_colorIndigo = 0x4B0082,
	b3_colorIvory = 0xFFFFF0,
	b3_colorKhaki = 0xF0E68C,
	b3_colorLavender = 0xE6E6FA,
	b3_colorLavenderBlush = 0xFFF0F5,
	b3_colorLawnGreen = 0x7CFC00,
	b3_colorLemonChiffon = 0xFFFACD,
	b3_colorLightBlue = 0xADD8E6,
	b3_colorLightCoral = 0xF08080,
	b3_colorLightCyan = 0xE0FFFF,
	b3_colorLightGoldenRodYellow = 0xFAFAD2,
	b3_colorLightGray = 0xD3D3D3,
	b3_colorLightGreen = 0x90EE90,
	b3_colorLightPink = 0xFFB6C1,
	b3_colorLightSalmon = 0xFFA07A,
	b3_colorLightSeaGreen = 0x20B2AA,
	b3_colorLightSkyBlue = 0x87CEFA,
	b3_colorLightSlateGray = 0x778899,
	b3_colorLightSteelBlue = 0xB0C4DE,
	b3_colorLightYellow = 0xFFFFE0,
	b3_colorLime = 0x00FF00,
	b3_colorLimeGreen = 0x32CD32,
	b3_colorLinen = 0xFAF0E6,
	b3_colorMagenta = 0xFF00FF,
	b3_colorMaroon = 0x800000,
	b3_colorMediumAquaMarine = 0x66CDAA,
	b3_colorMediumBlue = 0x0000CD,
	b3_colorMediumOrchid = 0xBA55D3,
	b3_colorMediumPurple = 0x9370DB,
	b3_colorMediumSeaGreen = 0x3CB371,
	b3_colorMediumSlateBlue = 0x7B68EE,
	b3_colorMediumSpringGreen = 0x00FA9A,
	b3_colorMediumTurquoise = 0x48D1CC,
	b3_colorMediumVioletRed = 0xC71585,
	b3_colorMidnightBlue = 0x191970,
	b3_colorMintCream = 0xF5FFFA,
	b3_colorMistyRose = 0xFFE4E1,
	b3_colorMoccasin = 0xFFE4B5,
	b3_colorNavajoWhite = 0xFFDEAD,
	b3_colorNavy = 0x000080,
	b3_colorOldLace = 0xFDF5E6,
	b3_colorOlive = 0x808000,
	b3_colorOliveDrab = 0x6B8E23,
	b3_colorOrange = 0xFFA500,
	b3_colorOrangeRed = 0xFF4500,
	b3_colorOrchid = 0xDA70D6,
	b3_colorPaleGoldenRod = 0xEEE8AA,
	b3_colorPaleGreen = 0x98FB98,
	b3_colorPaleTurquoise = 0xAFEEEE,
	b3_colorPaleVioletRed = 0xDB7093,
	b3_colorPapayaWhip = 0xFFEFD5,
	b3_colorPeachPuff = 0xFFDAB9,
	b3_colorPeru = 0xCD853F,
	b3_colorPink = 0xFFC0CB,
	b3_colorPlum = 0xDDA0DD,
	b3_colorPowderBlue = 0xB0E0E6,
	b3_colorPurple = 0x800080,
	b3_colorRebeccaPurple = 0x663399,
	b3_colorRed = 0xFF0000,
	b3_colorRosyBrown = 0xBC8F8F,
	b3_colorRoyalBlue = 0x4169E1,
	b3_colorSaddleBrown = 0x8B4513,
	b3_colorSalmon = 0xFA8072,
	b3_colorSandyBrown = 0xF4A460,
	b3_colorSeaGreen = 0x2E8B57,
	b3_colorSeaShell = 0xFFF5EE,
	b3_colorSienna = 0xA0522D,
	b3_colorSilver = 0xC0C0C0,
	b3_colorSkyBlue = 0x87CEEB,
	b3_colorSlateBlue = 0x6A5ACD,
	b3_colorSlateGray = 0x708090,
	b3_colorSnow = 0xFFFAFA,
	b3_colorSpringGreen = 0x00FF7F,
	b3_colorSteelBlue = 0x4682B4,
	b3_colorTan = 0xD2B48C,
	b3_colorTeal = 0x008080,
	b3_colorThistle = 0xD8BFD8,
	b3_colorTomato = 0xFF6347,
	b3_colorTurquoise = 0x40E0D0,
	b3_colorViolet = 0xEE82EE,
	b3_colorWheat = 0xF5DEB3,
	b3_colorWhite = 0xFFFFFF,
	b3_colorWhiteSmoke = 0xF5F5F5,
	b3_colorYellow = 0xFFFF00,
	b3_colorYellowGreen = 0x9ACD32,

	b3_colorBox2DRed = 0xDC3132,
	b3_colorBox2DBlue = 0x30AEBF,
	b3_colorBox2DGreen = 0x8CC924,
	b3_colorBox2DYellow = 0xFFEE8C
} b3HexColor;

/// Get the visualization color assigned to a constraint graph color slot. The last index
/// (B3_GRAPH_COLOR_COUNT - 1) is the overflow color.
B3_API b3HexColor b3GetGraphColor( int index );

// This is sent to the user for debug shape creation. The user should know the type in case they have
// custom sphere or capsule rendering.
typedef struct b3DebugShape
{
	b3ShapeId shapeId;
	b3ShapeType type;
	union
	{
		const b3Capsule* capsule;
		const b3Compound* compound;
		const b3HeightField* heightField;
		const b3Hull* hull;
		const b3Mesh* mesh;
		const b3Sphere* sphere;
	};
} b3DebugShape;

typedef struct b3DebugDraw
{
	// Draws a shape and returns true if drawing should continue
	bool ( *DrawShapeFcn )( void* userShape, b3Transform transform, b3HexColor color, void* context );

	/// Draw a line segment.
	void ( *DrawSegmentFcn )( b3Vec3 p1, b3Vec3 p2, b3HexColor color, void* context );

	/// Draw a transform. Choose your own length scale.
	void ( *DrawTransformFcn )( b3Transform transform, void* context );

	/// Draw a point.
	void ( *DrawPointFcn )( b3Vec3 p, float size, b3HexColor color, void* context );

	/// Draw a bounding box.
	void ( *DrawBoundsFcn )( b3AABB aabb, b3HexColor color, void* context );

	/// Draw a bounding box.
	void ( *DrawBoxFcn )( b3Vec3 extents, b3Transform transform, b3HexColor color, void* context );

	/// Draw a string in world space
	void ( *DrawStringFcn )( b3Vec3 p, const char* s, b3HexColor color, void* context );

	/// World bounds to use for debug draw
	b3AABB drawingBounds;

	/// Scale to use when drawing forces
	float forceScale;

	/// Global scaling for joint drawing
	float jointScale;

	/// Option to draw shapes
	bool drawShapes;

	/// Option to draw joints
	bool drawJoints;

	/// Option to draw additional information for joints
	bool drawJointExtras;

	/// Option to draw the bounding boxes for shapes
	bool drawBounds;

	/// Option to draw the mass and center of mass of dynamic bodies
	bool drawMass;

	/// Option to draw body names
	bool drawBodyNames;

	/// Option to draw contact points
	bool drawContacts;

	/// Draw contact anchor A or B
	int drawAnchorA;

	/// Option to visualize the graph coloring used for contacts and joints
	bool drawGraphColors;

	/// Option to draw contact features
	bool drawContactFeatures;

	/// Option to draw contact normals
	bool drawContactNormals;

	/// Option to draw contact normal forces
	bool drawContactForces;

	/// Option to draw contact friction forces
	bool drawFrictionForces;

	/// Option to draw islands as bounding boxes
	bool drawIslands;

	/// User context that is passed as an argument to drawing callback functions
	void* context;
} b3DebugDraw;

B3_API b3DebugDraw b3DefaultDebugDraw( void );

/**@}*/

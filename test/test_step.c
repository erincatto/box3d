// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "determinism.h"
#include "physics_world.h"
#include "platform.h"
#include "recording.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/types.h"

#include <stdio.h>
#include <string.h>

#define MAX_BATCH_TASKS ( B3_MAX_WORKERS + 2 )

typedef enum TaskOrder
{
	e_forward,
	e_reverse,
} TaskOrder;

// Runs every batch on the calling thread. Honoring the deferred flag holds the tree rebuild back until
// right before the Continue that joins it, which catches a join list placed on the wrong batch.
typedef struct SerialHost
{
	TaskOrder order;
	bool honorDeferred;
	int batchCount;
	int deferredBatch;
	int joinBatch;
	int deferredOnlyBatchCount;
	int maxBatchSize;
	int taskCount;
	const b3Task* heldTask;
} SerialHost;

static void RunSerialBatch( SerialHost* host, b3TaskBatch batch )
{
	host->maxBatchSize = batch.count > host->maxBatchSize ? batch.count : host->maxBatchSize;

	int deferredCount = 0;
	for ( int k = 0; k < batch.count; ++k )
	{
		int i = host->order == e_forward ? k : batch.count - 1 - k;
		const b3Task* task = batch.tasks + i;
		if ( task->deferred )
		{
			deferredCount += 1;
			host->deferredBatch = host->batchCount;
			if ( host->honorDeferred )
			{
				host->heldTask = task;
				continue;
			}
		}

		task->fcn( task->context );
	}

	if ( deferredCount == batch.count )
	{
		host->deferredOnlyBatchCount += 1;
	}

	if ( batch.joinCount > 0 )
	{
		host->joinBatch = host->batchCount;
		if ( host->heldTask != NULL )
		{
			host->heldTask->fcn( host->heldTask->context );
			host->heldTask = NULL;
		}
	}

	host->batchCount += 1;
}

static int SerialPhasedStep( SerialHost* host, b3WorldId worldId, float timeStep, int subStepCount )
{
	host->batchCount = 0;
	host->taskCount = 0;
	host->deferredBatch = -1;
	host->joinBatch = -1;
	host->heldTask = NULL;

	b3TaskBatch batch = b3World_BeginStep( worldId, timeStep, subStepCount );
	while ( batch.count > 0 )
	{
		ENSURE( batch.count <= MAX_BATCH_TASKS );
		ENSURE( batch.joinCount == 0 || batch.joinCount == 1 );
		for ( int i = 0; i < batch.count; ++i )
		{
			b3TaskType type = batch.tasks[i].type;
			ENSURE( 0 <= type && type < b3_taskTypeCount );
			ENSURE( batch.tasks[i].deferred == false || type == b3_taskTreeRebuild );
		}
		host->taskCount += batch.count;
		RunSerialBatch( host, batch );
		batch = b3World_ContinueStep( worldId );
	}

	ENSURE( host->heldTask == NULL );
	ENSURE( b3World_GetCounters( worldId ).taskCount == host->taskCount );
	ENSURE( b3World_GetCounters( worldId ).enqueueCount == 0 );
	ENSURE( ( host->deferredBatch == -1 ) == ( host->joinBatch == -1 ) );
	ENSURE( host->deferredBatch < host->joinBatch || host->deferredBatch == -1 );
	return 0;
}

// A pool of threads draining a queue that only grows. Slots are never reused, so producers and
// consumers need no reset handshake.
#define POOL_SLOT_CAPACITY ( 1 << 18 )
#define POOL_MAX_THREADS 8

typedef struct PoolSlot
{
	b3TaskCallback* fcn;
	void* context;
	b3AtomicInt* counter;
} PoolSlot;

typedef struct Pool
{
	PoolSlot* slots;
	b3AtomicInt published;
	b3AtomicInt claimed;
	b3AtomicInt quit;
	b3Thread* threads[POOL_MAX_THREADS];
	int threadCount;
} Pool;

static bool PoolRunOne( Pool* pool )
{
	int claimed = b3AtomicLoadInt( &pool->claimed );
	int published = b3AtomicLoadInt( &pool->published );
	if ( claimed >= published )
	{
		return false;
	}

	if ( b3AtomicCompareExchangeInt( &pool->claimed, claimed, claimed + 1 ) == false )
	{
		return true;
	}

	PoolSlot* slot = pool->slots + claimed;
	slot->fcn( slot->context );
	if ( slot->counter != NULL )
	{
		b3AtomicFetchAddInt( slot->counter, -1 );
	}

	return true;
}

static void PoolThreadMain( void* context )
{
	Pool* pool = context;
	while ( b3AtomicLoadInt( &pool->quit ) == 0 )
	{
		if ( PoolRunOne( pool ) == false )
		{
			b3Yield();
		}
	}
}

static void PoolPush( Pool* pool, b3TaskCallback* fcn, void* context, b3AtomicInt* counter )
{
	int index = b3AtomicLoadInt( &pool->published );
	B3_ASSERT( index < POOL_SLOT_CAPACITY );
	pool->slots[index] = (PoolSlot){ fcn, context, counter };
	b3AtomicStoreInt( &pool->published, index + 1 );
}

static void PoolHelpUntilZero( Pool* pool, b3AtomicInt* counter )
{
	while ( b3AtomicLoadInt( counter ) > 0 )
	{
		if ( PoolRunOne( pool ) == false )
		{
			b3Yield();
		}
	}
}

static void CreatePool( Pool* pool, int threadCount )
{
	memset( pool, 0, sizeof( Pool ) );
	pool->slots = b3Alloc( POOL_SLOT_CAPACITY * sizeof( PoolSlot ) );
	pool->threadCount = threadCount;
	for ( int i = 0; i < threadCount; ++i )
	{
		pool->threads[i] = b3CreateThread( PoolThreadMain, pool, "step_test" );
	}
}

static void DestroyPool( Pool* pool )
{
	b3AtomicStoreInt( &pool->quit, 1 );
	for ( int i = 0; i < pool->threadCount; ++i )
	{
		b3JoinThread( pool->threads[i] );
	}
	b3Free( pool->slots, POOL_SLOT_CAPACITY * sizeof( PoolSlot ) );
}

// Pull host on the pool. The rebuild stays in flight across Continue calls until a batch joins it.
static void PooledPhasedStep( Pool* pool, b3WorldId worldId, float timeStep, int subStepCount )
{
	b3AtomicInt batchCounter;
	b3AtomicInt treeCounter;
	b3AtomicStoreInt( &treeCounter, 0 );

	b3TaskBatch batch = b3World_BeginStep( worldId, timeStep, subStepCount );
	while ( batch.count > 0 )
	{
		int nonDeferred = 0;
		for ( int i = 0; i < batch.count; ++i )
		{
			nonDeferred += batch.tasks[i].deferred ? 0 : 1;
		}

		b3AtomicStoreInt( &batchCounter, nonDeferred );

		for ( int i = 0; i < batch.count; ++i )
		{
			const b3Task* task = batch.tasks + i;
			if ( task->deferred )
			{
				b3AtomicStoreInt( &treeCounter, 1 );
				PoolPush( pool, task->fcn, task->context, &treeCounter );
			}
			else
			{
				PoolPush( pool, task->fcn, task->context, &batchCounter );
			}
		}

		PoolHelpUntilZero( pool, &batchCounter );

		if ( batch.joinCount > 0 )
		{
			PoolHelpUntilZero( pool, &treeCounter );
		}

		batch = b3World_ContinueStep( worldId );
	}

	B3_ASSERT( b3AtomicLoadInt( &treeCounter ) == 0 );
}

static int enqueueOnlyCount;

static void* EnqueueOnlyTask( b3TaskCallback* task, void* taskContext, void* userContext, b3TaskType taskType )
{
	B3_UNUSED( taskType );
	enqueueOnlyCount += 1;
	Pool* pool = userContext;
	PoolPush( pool, task, taskContext, NULL );
	return pool;
}

typedef enum HostKind
{
	e_syncHost,
	e_serialHost,
	e_pooledHost,
} HostKind;

typedef struct HostConfig
{
	HostKind kind;
	TaskOrder order;
	bool honorDeferred;
	int workerCount;
	int threadCount;
} HostConfig;

static int RunFallingHinges( HostConfig config )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = config.workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	Pool pool;
	if ( config.kind == e_pooledHost )
	{
		CreatePool( &pool, config.threadCount - 1 );
	}

	SerialHost serialHost = { .order = config.order, .honorDeferred = config.honorDeferred };

	FallingHingeData data = CreateFallingHinges( worldId );

	float timeStep = 1.0f / 60.0f;
	int stepLimit = 1000;
	for ( int i = 0; i < stepLimit; ++i )
	{
		int subStepCount = 4;
		if ( config.kind == e_serialHost )
		{
			int result = SerialPhasedStep( &serialHost, worldId, timeStep, subStepCount );
			ENSURE( result == 0 );
		}
		else if ( config.kind == e_pooledHost )
		{
			PooledPhasedStep( &pool, worldId, timeStep, subStepCount );
		}
		else
		{
			b3World_Step( worldId, timeStep, subStepCount );
		}

		if ( UpdateFallingHinges( worldId, &data ) )
		{
			break;
		}
	}

	if ( config.kind != e_syncHost )
	{
		ENSURE( b3GetWorldFromId( worldId )->scheduler == NULL );
	}

	ENSURE( serialHost.maxBatchSize <= config.workerCount + 2 );

	b3DestroyWorld( worldId );

	if ( config.kind == e_pooledHost )
	{
		DestroyPool( &pool );
	}

	if ( data.sleepStep != FALLING_HINGE_SLEEP_STEP || data.hash != FALLING_HINGE_HASH )
	{
		printf( "  host=%d order=%d deferred=%d workers=%d threads=%d sleepStep=%d hash=0x%08X\n", config.kind, config.order,
				config.honorDeferred ? 1 : 0, config.workerCount, config.threadCount, data.sleepStep, data.hash );
	}

	ENSURE( data.sleepStep == FALLING_HINGE_SLEEP_STEP );
	ENSURE( data.hash == FALLING_HINGE_HASH );

	DestroyFallingHinges( &data );

	return 0;
}

static int SerialPullTest( void )
{
	int workerCounts[] = { 1, 4, 8 };
	for ( int i = 0; i < 3; ++i )
	{
		HostConfig config = { .kind = e_serialHost, .order = e_forward, .workerCount = workerCounts[i] };
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

// The first solver task to run must orchestrate whatever its slot, or a serial host strands a spinner
static int ReverseOrderTest( void )
{
	int workerCounts[] = { 4, 8 };
	for ( int i = 0; i < 2; ++i )
	{
		HostConfig config = { .kind = e_serialHost, .order = e_reverse, .workerCount = workerCounts[i] };
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

static int DeferredJoinTest( void )
{
	int workerCounts[] = { 1, 4 };
	for ( int i = 0; i < 2; ++i )
	{
		HostConfig config = {
			.kind = e_serialHost, .order = e_forward, .honorDeferred = true, .workerCount = workerCounts[i] };
		ENSURE( RunFallingHinges( config ) == 0 );

		config.order = e_reverse;
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

static int PooledPullTest( void )
{
	int pairs[][2] = { { 1, 1 }, { 2, 2 }, { 4, 4 }, { 8, 8 }, { 8, 2 }, { 2, 8 } };
	for ( int i = 0; i < 6; ++i )
	{
		HostConfig config = { .kind = e_pooledHost, .workerCount = pairs[i][0], .threadCount = pairs[i][1] };
		ENSURE( RunFallingHinges( config ) == 0 );
	}

	return 0;
}

static int EnqueueOnlyTest( void )
{
	int workerCounts[] = { 2, 4 };
	for ( int i = 0; i < 2; ++i )
	{
		Pool pool;
		CreatePool( &pool, 3 );

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.workerCount = workerCounts[i];
		worldDef.enqueueTask = EnqueueOnlyTask;
		worldDef.finishTask = NULL;
		worldDef.userTaskContext = &pool;
		b3WorldId worldId = b3CreateWorld( &worldDef );

		FallingHingeData data = CreateFallingHinges( worldId );

		float timeStep = 1.0f / 60.0f;
		for ( int step = 0; step < 1000; ++step )
		{
			enqueueOnlyCount = 0;
			b3World_Step( worldId, timeStep, 4 );
			b3Counters counters = b3World_GetCounters( worldId );
			ENSURE( counters.enqueueCount == enqueueOnlyCount );
			ENSURE( counters.enqueueCount <= counters.taskCount );

			if ( UpdateFallingHinges( worldId, &data ) )
			{
				break;
			}
		}

		ENSURE( b3GetWorldFromId( worldId )->scheduler == NULL );

		b3DestroyWorld( worldId );
		DestroyPool( &pool );

		ENSURE( data.sleepStep == FALLING_HINGE_SLEEP_STEP );
		ENSURE( data.hash == FALLING_HINGE_HASH );

		DestroyFallingHinges( &data );
	}

	return 0;
}

static int AlternateTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 4;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	FallingHingeData data = CreateFallingHinges( worldId );
	SerialHost host = { .order = e_reverse, .honorDeferred = true };

	float timeStep = 1.0f / 60.0f;
	for ( int i = 0; i < 1000; ++i )
	{
		if ( ( i & 1 ) == 0 )
		{
			b3World_Step( worldId, timeStep, 4 );
		}
		else
		{
			ENSURE( SerialPhasedStep( &host, worldId, timeStep, 4 ) == 0 );
		}

		if ( UpdateFallingHinges( worldId, &data ) )
		{
			break;
		}
	}

	b3DestroyWorld( worldId );

	ENSURE( data.sleepStep == FALLING_HINGE_SLEEP_STEP );
	ENSURE( data.hash == FALLING_HINGE_HASH );

	DestroyFallingHinges( &data );

	return 0;
}

static int EmptyWorldTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 4;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	SerialHost host = { .order = e_forward };
	ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
	ENSURE( host.batchCount == 0 );
	ENSURE( host.deferredBatch == -1 );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );
	ENSURE( host.batchCount == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

static b3BodyId CreateBall( b3WorldId worldId, b3Pos position )
{
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = position;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );
	return bodyId;
}

// The rebuild must join on the solve batch normally and on the collide batch when solve is skipped
static int JoinPlacementTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 4;

	// A lone ball, no contacts. A zero step skips solve, so the collide batch is the rebuild alone and
	// nothing later can join it. It runs as a regular task.
	{
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3BodyId bodyId = CreateBall( worldId, (b3Pos){ 0.0f, 4.0f, 0.0f } );

		SerialHost host = { .order = e_forward, .honorDeferred = true };
		ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );
		ENSURE( host.batchCount > 0 );
		ENSURE( host.deferredBatch == -1 );
		ENSURE( host.deferredOnlyBatchCount == 0 );

		b3Pos p = b3Body_GetPosition( bodyId );
		ENSURE( p.x == 0.0f && p.y == 4.0f );

		b3DestroyWorld( worldId );
	}

	// A lone moving ball, no contacts. The collide batch is empty, so the rebuild rides with the solver
	// tasks as a regular task instead of running alone.
	{
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3BodyId bodyId = CreateBall( worldId, (b3Pos){ 0.0f, 4.0f, 0.0f } );
		b3Body_SetLinearVelocity( bodyId, (b3Vec3){ 10.0f, 0.0f, 0.0f } );

		SerialHost host = { .order = e_forward, .honorDeferred = true };
		for ( int i = 0; i < 4; ++i )
		{
			ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
			ENSURE( host.deferredBatch == -1 );
		}

		ENSURE( host.deferredOnlyBatchCount == 0 );

		b3DestroyWorld( worldId );
	}

	// Touching balls on the ground with a zero step: the collide batch has contacts and the join
	{
		b3WorldId worldId = b3CreateWorld( &worldDef );

		b3BodyDef groundDef = b3DefaultBodyDef();
		b3BodyId groundId = b3CreateBody( worldId, &groundDef );
		b3ShapeDef groundShapeDef = b3DefaultShapeDef();
		b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 1.0f, 20.0f );
		b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

		for ( int i = 0; i < 8; ++i )
		{
			CreateBall( worldId, (b3Pos){ -4.0f + 1.0f * i, 0.5f, 0.0f } );
		}

		SerialHost host = { .order = e_forward, .honorDeferred = true };
		ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );
		ENSURE( SerialPhasedStep( &host, worldId, 0.0f, 4 ) == 0 );

		// Moving step: the join lands on a later batch than the rebuild
		for ( int i = 0; i < 4; ++i )
		{
			ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
			if ( host.deferredBatch >= 0 )
			{
				ENSURE( host.joinBatch > host.deferredBatch );
			}
		}

		b3DestroyWorld( worldId );
	}

	return 0;
}

// No contacts, a moving ball that keeps the rebuild staged, and a resting pair whose joint is destroyed.
// The pair can only sleep after its island splits, so the solve batch carries the rebuild, the split and
// the solver tasks together.
static int SplitBatchTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 4;
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyId bodyA = CreateBall( worldId, (b3Pos){ 0.0f, 0.0f, 0.0f } );
	b3BodyId bodyB = CreateBall( worldId, (b3Pos){ 2.0f, 0.0f, 0.0f } );
	b3BodyId ballId = CreateBall( worldId, (b3Pos){ 0.0f, 20.0f, 0.0f } );
	b3Body_SetLinearVelocity( ballId, (b3Vec3){ 10.0f, 0.0f, 0.0f } );

	b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
	jointDef.base.bodyIdA = bodyA;
	jointDef.base.bodyIdB = bodyB;
	jointDef.base.localFrameA.p = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	jointDef.base.localFrameB.p = (b3Vec3){ -1.0f, 0.0f, 0.0f };
	b3JointId jointId = b3CreateRevoluteJoint( worldId, &jointDef );

	SerialHost host = { .order = e_forward, .honorDeferred = true };
	ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );

	b3DestroyJoint( jointId, true );

	int stepCount = 0;
	while ( b3Body_IsAwake( bodyA ) && stepCount < 120 )
	{
		ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
		stepCount += 1;
	}

	ENSURE( host.maxBatchSize == worldDef.workerCount + 2 );
	ENSURE( b3Body_IsAwake( bodyA ) == false );
	ENSURE( b3Body_IsAwake( ballId ) );

	b3DestroyWorld( worldId );
	return 0;
}

// A scene with bullets, a sensor, joints and sleeping islands, checked against the synchronous step
// every step by state hash.
static b3WorldId CreateMixedScene( int workerCount )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef groundDef = b3DefaultBodyDef();
	b3BodyId groundId = b3CreateBody( worldId, &groundDef );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( 40.0f, 1.0f, 40.0f );
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	b3ShapeDef sensorDef = b3DefaultShapeDef();
	sensorDef.isSensor = true;
	sensorDef.enableSensorEvents = true;
	b3BoxHull sensorBox = b3MakeOffsetBoxHull( 4.0f, 2.0f, 4.0f, (b3Vec3){ 10.0f, 2.0f, 0.0f } );
	b3CreateHullShape( groundId, &sensorDef, &sensorBox.base );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3ShapeDef boxDef = b3DefaultShapeDef();
	boxDef.enableSensorEvents = true;
	boxDef.enableHitEvents = true;

	for ( int row = 0; row < 10; ++row )
	{
		for ( int col = 0; col < 10 - row; ++col )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ -5.0f + col * 1.05f + row * 0.525f, 0.5f + row * 1.0f, 0.0f };
			b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
			b3CreateHullShape( bodyId, &boxDef, &box.base );
		}
	}

	b3BodyId prevId = groundId;
	for ( int i = 0; i < 6; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 12.0f + i * 1.0f, 8.0f, 0.0f };
		b3BodyId linkId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull link = b3MakeBoxHull( 0.5f, 0.125f, 0.125f );
		b3CreateHullShape( linkId, &boxDef, &link.base );

		b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
		jointDef.base.bodyIdA = prevId;
		jointDef.base.bodyIdB = linkId;
		jointDef.base.localFrameA.p =
			prevId.index1 == groundId.index1 ? (b3Vec3){ 11.5f, 8.0f, 0.0f } : (b3Vec3){ 0.5f, 0.0f, 0.0f };
		jointDef.base.localFrameB.p = (b3Vec3){ -0.5f, 0.0f, 0.0f };
		b3CreateRevoluteJoint( worldId, &jointDef );
		prevId = linkId;
	}

	for ( int i = 0; i < 4; ++i )
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		bodyDef.position = (b3Pos){ -30.0f, 1.0f + 2.0f * i, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 200.0f, 0.0f, 0.0f };
		b3BodyId bulletId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef bulletDef = b3DefaultShapeDef();
		bulletDef.enableSensorEvents = true;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
		b3CreateSphereShape( bulletId, &bulletDef, &sphere );
	}

	return worldId;
}

static uint64_t StateHash( b3WorldId worldId )
{
	return b3HashWorldState( b3GetWorldFromId( worldId ) );
}

static int MixedSceneTest( void )
{
	int workerCounts[] = { 1, 4, 8 };
	for ( int w = 0; w < 3; ++w )
	{
		int workerCount = workerCounts[w];
		b3WorldId syncId = CreateMixedScene( workerCount );
		b3WorldId serialId = CreateMixedScene( workerCount );
		b3WorldId pooledId = CreateMixedScene( workerCount );

		Pool pool;
		CreatePool( &pool, 3 );
		SerialHost host = { .order = e_reverse, .honorDeferred = true };

		int sensorBeginTotal = 0;
		int hitTotal = 0;
		int maxBatchCount = 0;
		float timeStep = 1.0f / 60.0f;
		for ( int i = 0; i < 400; ++i )
		{
			b3World_Step( syncId, timeStep, 4 );
			ENSURE( SerialPhasedStep( &host, serialId, timeStep, 4 ) == 0 );
			PooledPhasedStep( &pool, pooledId, timeStep, 4 );

			uint64_t syncHash = StateHash( syncId );
			ENSURE( StateHash( serialId ) == syncHash );
			ENSURE( StateHash( pooledId ) == syncHash );

			b3SensorEvents syncSensors = b3World_GetSensorEvents( syncId );
			b3SensorEvents serialSensors = b3World_GetSensorEvents( serialId );
			ENSURE( syncSensors.beginCount == serialSensors.beginCount );
			ENSURE( syncSensors.endCount == serialSensors.endCount );

			b3ContactEvents syncContacts = b3World_GetContactEvents( syncId );
			b3ContactEvents serialContacts = b3World_GetContactEvents( serialId );
			ENSURE( syncContacts.beginCount == serialContacts.beginCount );
			ENSURE( syncContacts.hitCount == serialContacts.hitCount );

			b3BodyEvents syncBodies = b3World_GetBodyEvents( syncId );
			b3BodyEvents serialBodies = b3World_GetBodyEvents( serialId );
			ENSURE( syncBodies.moveCount == serialBodies.moveCount );

			sensorBeginTotal += serialSensors.beginCount;
			hitTotal += serialContacts.hitCount;
			maxBatchCount = host.batchCount > maxBatchCount ? host.batchCount : maxBatchCount;
		}

		ENSURE( sensorBeginTotal > 0 );
		ENSURE( hitTotal > 0 );
		ENSURE( maxBatchCount == 7 );
		ENSURE( b3World_GetAwakeBodyCount( serialId ) < 55 + 6 + 4 );

		ENSURE( b3World_GetAwakeBodyCount( syncId ) == b3World_GetAwakeBodyCount( serialId ) );
		ENSURE( b3World_GetCounters( serialId ).taskCount > 0 );

		DestroyPool( &pool );
		b3DestroyWorld( syncId );
		b3DestroyWorld( serialId );
		b3DestroyWorld( pooledId );
	}

	return 0;
}

static int PhasedRecordingTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3Recording* recording = b3CreateRecording( 0 );
	b3World_StartRecording( worldId, recording );

	FallingHingeData data = CreateFallingHinges( worldId );

	SerialHost host = { .order = e_reverse, .honorDeferred = true };
	for ( int i = 0; i < 120; ++i )
	{
		ENSURE( SerialPhasedStep( &host, worldId, 1.0f / 60.0f, 4 ) == 0 );
		UpdateFallingHinges( worldId, &data );
	}

	b3World_StopRecording( worldId );
	b3DestroyWorld( worldId );
	DestroyFallingHinges( &data );

	const uint8_t* bytes = b3Recording_GetData( recording );
	int size = b3Recording_GetSize( recording );
	ENSURE( b3ValidateReplay( bytes, size, 1 ) );
	ENSURE( b3ValidateReplay( bytes, size, 4 ) );

	b3DestroyRecording( recording );
	return 0;
}

int StepTest( void )
{
	RUN_SUBTEST( SerialPullTest );
	RUN_SUBTEST( ReverseOrderTest );
	RUN_SUBTEST( DeferredJoinTest );
	RUN_SUBTEST( PooledPullTest );
	RUN_SUBTEST( EnqueueOnlyTest );
	RUN_SUBTEST( AlternateTest );
	RUN_SUBTEST( EmptyWorldTest );
	RUN_SUBTEST( JoinPlacementTest );
	RUN_SUBTEST( SplitBatchTest );
	RUN_SUBTEST( MixedSceneTest );
	RUN_SUBTEST( PhasedRecordingTest );

	return 0;
}

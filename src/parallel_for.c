// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "parallel_for.h"

#include "core.h"
#include "physics_world.h"
#include "platform.h"
#include "solver.h"

#include "box3d/base.h"
#include "box3d/constants.h"

#include <stddef.h>

static void b3RunStepTask( void* taskContext )
{
	b3StepTask* task = taskContext;
	b3StepContext* stepContext = task->stepContext;

	task->fcn( task->context );

	int index = (int)( task - stepContext->stepTasks );
	if ( stepContext->tasks[index].deferred )
	{
		if ( b3AtomicFetchAddInt( &stepContext->treePending, -1 ) == 1 && stepContext->signalSemaphores )
		{
			b3SignalSemaphore( stepContext->treeSemaphore );
		}
	}
	else
	{
		if ( b3AtomicFetchAddInt( &stepContext->pendingCount, -1 ) == 1 && stepContext->signalSemaphores )
		{
			b3SignalSemaphore( stepContext->batchSemaphore );
		}
	}
}

void b3ResetTaskBatch( b3StepContext* stepContext )
{
	stepContext->batchTaskBase = 1;
	stepContext->batchTaskCount = 0;
	stepContext->joinTree = false;
}

void b3AddTask( b3StepContext* stepContext, b3TaskType type, b3TaskCallback* fcn, void* context )
{
	int index = stepContext->batchTaskBase + stepContext->batchTaskCount;
	B3_ASSERT( 1 <= index && index < B3_STEP_TASK_CAPACITY );

	b3StepTask* stepTask = stepContext->stepTasks + index;
	stepTask->fcn = fcn;
	stepTask->context = context;
	stepTask->stepContext = stepContext;

	stepContext->tasks[index] = (b3Task){
		.fcn = b3RunStepTask,
		.context = stepTask,
		.type = type,
		.deferred = false,
	};

	stepContext->batchTaskCount += 1;
	stepContext->taskCount += 1;
}

void b3StageDeferredTask( b3StepContext* stepContext, b3TaskType type, b3TaskCallback* fcn, void* context )
{
	B3_ASSERT( stepContext->treeStaged == false );

	b3StepTask* stepTask = stepContext->stepTasks + 0;
	stepTask->fcn = fcn;
	stepTask->context = context;
	stepTask->stepContext = stepContext;

	stepContext->tasks[0] = (b3Task){
		.fcn = b3RunStepTask,
		.context = stepTask,
		.type = type,
		.deferred = true,
	};

	stepContext->treeStaged = true;
	stepContext->taskCount += 1;
	b3AtomicStoreInt( &stepContext->treePending, 1 );
}

static void b3ParallelForTrampoline( void* taskContext )
{
	b3ParallelForTask* task = (b3ParallelForTask*)taskContext;
	b3ParallelForShared* shared = task->shared;
	int workerIndex = task->workerIndex;
	void* context = shared->context;
	b3ParallelForCallback* callback = shared->callback;

	int blockCount = shared->blockCount;
	int blockSize = shared->blockSize;
	int itemCount = shared->itemCount;

	for ( ;; )
	{
		int blockIndex = b3AtomicFetchAddInt( &shared->nextBlock, 1 );
		if ( blockIndex >= blockCount )
		{
			break;
		}

		int start = blockIndex * blockSize;
		int end = start + blockSize;
		if ( end > itemCount )
		{
			end = itemCount;
		}

		callback( start, end, workerIndex, context );
	}
}

void b3StageParallelFor( b3World* world, b3TaskType type, b3ParallelForCallback* callback, int itemCount, int minRange,
						 void* context )
{
	if ( itemCount <= 0 )
	{
		return;
	}

	B3_ASSERT( minRange > 0 );

	int workerCount = world->workerCount;
	B3_ASSERT( 0 < workerCount && workerCount <= B3_MAX_WORKERS );

	// Target multiple blocks per worker to reduce thread stalls.
	// block size grows once items exceed maxBlockCount * minRange
	// so the block count stays bounded and per-block sync overhead stays low.
	// Benchmarking shows 32 is optimal for the convex pile benchmark and others.
	int blocksPerWorker = 32;
	int maxBlockCount = blocksPerWorker * workerCount;

	int blockSize;
	int blockCount;
	if ( itemCount <= minRange * maxBlockCount )
	{
		blockSize = minRange;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	else
	{
		blockSize = ( itemCount + maxBlockCount - 1 ) / maxBlockCount;
		blockCount = ( itemCount + blockSize - 1 ) / blockSize;
	}
	B3_ASSERT( blockCount >= 1 );
	B3_ASSERT( blockSize * blockCount >= itemCount );

	// No point enqueueing more tasks than blocks.
	int taskCount = workerCount < blockCount ? workerCount : blockCount;

	b3StepContext* stepContext = world->stepContext;
	b3ParallelForShared* shared = &stepContext->parallelFor;
	shared->blockCount = blockCount;
	shared->blockSize = blockSize;
	shared->itemCount = itemCount;
	shared->callback = callback;
	shared->context = context;
	b3AtomicStoreInt( &shared->nextBlock, 0 );

	for ( int i = 0; i < taskCount; ++i )
	{
		b3ParallelForTask* task = stepContext->parallelForTasks + i;
		task->shared = shared;
		task->workerIndex = i;
		b3AddTask( stepContext, type, b3ParallelForTrampoline, task );
	}
}

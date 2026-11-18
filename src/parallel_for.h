// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include "box3d/types.h"

// todo rename files to task.c and task.h

typedef struct b3StepContext b3StepContext;
typedef struct b3World b3World;

// Callback invoked by a staged parallel-for to process a range of items. May be called
// multiple times per worker: work is divided into blocks that workers claim
// atomically, so a worker that finishes early picks up the next unclaimed
// block instead of sitting idle. workerIndex is the worker identity and is
// stable across all invocations from the same worker, so it is safe to use as
// an index into per-worker state (e.g. world->taskContexts.data + workerIndex).
typedef void b3ParallelForCallback( int startIndex, int endIndex, int workerIndex, void* context );

typedef struct b3ParallelForShared
{
	b3AtomicInt nextBlock;
	int blockCount;
	int blockSize;
	int itemCount;
	b3ParallelForCallback* callback;
	void* context;
} b3ParallelForShared;

typedef struct b3ParallelForTask
{
	b3ParallelForShared* shared;
	int workerIndex;
} b3ParallelForTask;

void b3ResetTaskBatch( b3StepContext* stepContext );
void b3AddTask( b3StepContext* stepContext, b3TaskType type, b3TaskCallback* fcn, void* context );
void b3StageDeferredTask( b3StepContext* stepContext, b3TaskType type, b3TaskCallback* fcn, void* context );

// Divide [0, itemCount) into blocks and add up to world->workerCount tasks to the current
// batch. Each task loops, atomically claiming the next unclaimed block until the range is
// drained. minRange is the minimum block size. Block size grows once itemCount exceeds
// 32 * workerCount * minRange so block count stays bounded.
void b3StageParallelFor( b3World* world, b3TaskType type, b3ParallelForCallback* callback, int itemCount, int minRange,
						 void* context );

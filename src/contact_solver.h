// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_internal.h"
#include "solver.h"

typedef struct b3ManifoldConstraintPoint
{
	b3Vec3 rA, rB;
	float baseSeparation;
	float relativeVelocity;
	float normalImpulse;
	float totalNormalImpulse;
	float restitutionImpulse;
	float normalMass;
	float leverArm;
} b3ManifoldConstraintPoint;

typedef struct b3ManifoldConstraint
{
	b3ManifoldConstraintPoint points[B3_MAX_MANIFOLD_POINTS];
	int pointCount;
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;
	// Friction centers
	b3Vec3 centerA, centerB;
	float twistMass;
	float twistImpulse;
	b3Matrix2 tangentMass;
	b3Vec2 frictionImpulse;
	b3Vec3 rollingImpulse;
	float tangentVelocity1;
	float tangentVelocity2;
} b3ManifoldConstraint;

typedef struct b3ContactConstraint
{
	b3ManifoldConstraint* constraints;
	struct b3Contact* contact;
	int indexA;
	int indexB;
	float invMassA, invMassB;
	b3Matrix3 invIA, invIB;
	b3Softness softness;
	b3Matrix3 rollingMass;
	float friction;
	float restitution;
	float rollingResistance;
	int manifoldCount;
} b3ContactConstraint;

int b3GetWideContactConstraintByteCount( int simdWidth );

// Overflow contacts don't fit into the constraint graph coloring
void b3PrepareContacts_Overflow( b3StepContext* context );
void b3WarmStartContacts_Overflow( b3StepContext* context );
void b3PushContacts_Overflow( b3StepContext* context );
void b3SolveContacts_Overflow( b3StepContext* context );
void b3ApplyRestitution_Overflow( b3StepContext* context );
void b3StoreImpulses_Overflow( b3StepContext* context );

void b3PrepareContacts_Mesh( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_Mesh( b3SolverBlock block, b3StepContext* context );
void b3PushContacts_Mesh( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_Mesh( b3SolverBlock block, b3StepContext* context );
void b3ApplyRestitution_Mesh( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_Mesh( b3SolverBlock block, b3StepContext* context, int workerIndex );

void b3PrepareContacts_Convex( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_Convex( b3SolverBlock block, b3StepContext* context );
void b3PushContacts_Convex( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_Convex( b3SolverBlock block, b3StepContext* context );
void b3ApplyRestitution_Convex( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_Convex( b3SolverBlock block, b3StepContext* context, int workerIndex );

void b3PrepareContacts_ConvexW4( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_ConvexW4( b3SolverBlock block, b3StepContext* context );
void b3PushContacts_ConvexW4( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_ConvexW4( b3SolverBlock block, b3StepContext* context );
void b3ApplyRestitution_ConvexW4( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_ConvexW4( b3SolverBlock block, b3StepContext* context, int workerIndex );
int b3GetWideContactConstraintByteCountW4( void );

void b3PrepareContacts_ConvexW8( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_ConvexW8( b3SolverBlock block, b3StepContext* context );
void b3PushContacts_ConvexW8( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_ConvexW8( b3SolverBlock block, b3StepContext* context );
void b3ApplyRestitution_ConvexW8( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_ConvexW8( b3SolverBlock block, b3StepContext* context, int workerIndex );
int b3GetWideContactConstraintByteCountW8( void );

void b3PrepareContacts_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3PushContacts_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3ApplyRestitution_MeshWide( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_MeshWide( b3SolverBlock block, b3StepContext* context, int workerIndex );
int b3GetWideMeshConstraintByteCount( int simdWidth );
int b3GetWideMeshManifoldByteCount( int simdWidth );

void b3PrepareContacts_MeshWideW4( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_MeshWideW4( b3SolverBlock block, b3StepContext* context );
void b3PushContacts_MeshWideW4( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_MeshWideW4( b3SolverBlock block, b3StepContext* context );
void b3ApplyRestitution_MeshWideW4( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_MeshWideW4( b3SolverBlock block, b3StepContext* context, int workerIndex );
int b3GetWideMeshConstraintByteCountW4( void );
int b3GetWideMeshManifoldByteCountW4( void );

void b3PrepareContacts_MeshWideW8( b3SolverBlock block, b3StepContext* context );
void b3WarmStartContacts_MeshWideW8( b3SolverBlock block, b3StepContext* context );
void b3PushContacts_MeshWideW8( b3SolverBlock block, b3StepContext* context );
void b3SolveContacts_MeshWideW8( b3SolverBlock block, b3StepContext* context );
void b3ApplyRestitution_MeshWideW8( b3SolverBlock block, b3StepContext* context );
void b3StoreImpulses_MeshWideW8( b3SolverBlock block, b3StepContext* context, int workerIndex );
int b3GetWideMeshConstraintByteCountW8( void );
int b3GetWideMeshManifoldByteCountW8( void );

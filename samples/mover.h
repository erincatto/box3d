// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

class Sample;

struct MoverShapeUserData
{
	float maxPush;
	bool clipVelocity;
};

struct PlaneExtra
{
	b3Pos point;
	b3ShapeId shapeId;
};

struct CharacterMover
{
	static constexpr int m_planeCapacity = 8;
	static constexpr float m_jumpSpeed = 5.0f;
	static constexpr float m_maxSpeed = 6.0f;
	static constexpr float m_minSpeed = 0.01f;
	static constexpr float m_stopSpeed = 1.0f;
	static constexpr float m_accelerate = 30.0f;
	static constexpr float m_friction = 4.0f;
	static constexpr float m_gravity = 15.0f;

	void Initialize( Sample* sample, b3Pos position );
	void SolveMove( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle, bool clipVelocity );
	void Step( b3ShapeId* ignoreShapes, int ignoreCount, bool clipVelocity );

	Sample* m_sample;
	b3WorldTransform m_transform;
	b3Vec3 m_velocity;
	b3Capsule m_capsule;
	b3CollisionPlane m_planes[m_planeCapacity] = {};
	PlaneExtra m_planeExtras[m_planeCapacity] = {};
	int m_planeCount;
	int m_totalIterations;
	float m_pogoVelocity;
	bool m_onGround;
	bool m_sprint;

	// Transient
	b3ShapeId* m_ignoreShapeIds;
	int m_ignoreCount;
};

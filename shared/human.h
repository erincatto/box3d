// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

typedef enum BoneId
{
	bone_pelvis,
	bone_spine_01,
	bone_spine_02,
	bone_spine_03,
	bone_neck,
	bone_head,
	bone_thigh_l,
	bone_calf_l,
	bone_thigh_r,
	bone_calf_r,
	bone_upper_arm_l,
	bone_lower_arm_l,
	bone_upper_arm_r,
	bone_lower_arm_r,
	bone_count,
} BoneId;

typedef struct Bone
{
	b3BodyId bodyId;
	b3JointId jointId;
	b3BodyId anchorId;
	b3JointId anchorJointId;
	b3Transform localFrameA;
	b3Transform localFrameB;
	b3Transform referenceFrame;
	b3JointType jointType;
	float swingLimit;
	b3Vec2 twistLimit;
	float jointFriction;
	int parentIndex;
} Bone;

#define FILTER_JOINT_COUNT 8

// This must be zero initialized
typedef struct Human
{
	Bone bones[bone_count];
	b3JointId filterJoints[FILTER_JOINT_COUNT];
	int filterJointCount;
	float frictionTorque;
	bool isSpawned;
} Human;

#ifdef __cplusplus
extern "C"
{
#endif

void CreateHuman( Human* human, b3WorldId worldId, b3Pos position, float frictionTorque, float hertz, float dampingRatio,
				  int groupIndex, void* userData, bool colorize );

// There are no allocations so this does not need to be called when the world is destroyed.
void DestroyHuman( Human* human );

void Human_SetVelocity( Human* human, b3Vec3 velocity );
void Human_ApplyRandomAngularImpulse( Human* human, float magnitude );
void Human_SetJointFrictionTorque( Human* human, float torque );
void Human_SetJointSpringHertz( Human* human, float hertz );
void Human_SetJointDampingRatio( Human* human, float dampingRatio );

void Human_AlignSpring( Human* human, b3WorldId worldId, b3BodyId groundId, float hertz, float dampingRatio );
void Human_CreateMotorAnchors( Human* human, b3WorldId worldId );
void Human_CreateParallelAnchors( Human* human, b3WorldId worldId );

void Human_SetBullet( Human* human, bool flag );

// void Human_EnablePoseControl( Human* human, float springHertz, bool enablePoseControl );
// void Human_AdjustPoseControl( Human* human, float springHertz );
// void Human_DriveBase( Human* human, b3Transform transform, float timeStep );

#ifdef __cplusplus
}
#endif

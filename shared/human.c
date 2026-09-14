// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "human.h"

#include "utils.h"

#include "box3d/box3d.h"

#include <assert.h>
#include <stddef.h>

void CreateHuman( Human* human, b3WorldId worldId, b3Pos position, float frictionTorque, float hertz, float dampingRatio,
				  int groupIndex, void* userData, bool colorize )
{
	assert( human->isSpawned == false );

	for ( int i = 0; i < bone_count; ++i )
	{
		human->bones[i].bodyId = b3_nullBodyId;
		human->bones[i].anchorId = b3_nullBodyId;
		human->bones[i].jointId = b3_nullJointId;
		human->bones[i].jointFriction = 1.0f;
		human->bones[i].parentIndex = -1;
	}

	for ( int i = 0; i < FILTER_JOINT_COUNT; ++i )
	{
		human->filterJoints[i] = b3_nullJointId;
	}

	human->frictionTorque = frictionTorque;

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.userData = userData;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	float defaultDensity = shapeDef.density;

	b3HexColor shirtColor = b3_colorMediumTurquoise;
	b3HexColor pantColor = b3_colorDodgerBlue;
	b3HexColor skinColors[4] = { b3_colorNavajoWhite, b3_colorLightYellow, b3_colorPeru, b3_colorTan };
	b3HexColor skinColor = skinColors[groupIndex % 4];

	{
		Bone* bone = human->bones + bone_pelvis;
		bone->parentIndex = -1;

		bodyDef.name = "pelvis";
		bone->referenceFrame = (b3Transform){ { 0.000000f, 0.996219f, -0.023868f }, { { 1.000000f, 0.000000f, 0.000000f }, 0.000000f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.040000f, 0.000001f, 0.000000f }, { -0.040000f, -0.000001f, 0.000000f }, 0.150000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );
	}

	{
		Bone* bone = human->bones + bone_spine_01;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "spine_01";
		bone->referenceFrame = (b3Transform){ { 0.000000f, 1.017288f, -0.024882f }, { { 1.000000f, 0.000000f, 0.000000f }, 0.000000f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = 0.5f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.029876f, -0.146581f, 0.006260f }, { -0.029876f, -0.146574f, 0.006260f }, 0.145663f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000000f, -0.021069f, 0.001014f }, { { -0.642737f, 0.000000f, 0.000000f }, -0.766087f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.707107f, 0.000000f, 0.000000f }, -0.707107f } };
		bone->swingLimit = 35.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -17.5f * B3_DEG_TO_RAD, 17.5f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_spine_03;
		bone->parentIndex = bone_spine_01;

		bodyDef.name = "spine_03";
		bone->referenceFrame = (b3Transform){ { 0.000000f, 1.267766f, -0.022320f }, { { 1.000000f, 0.000000f, 0.000000f }, 0.000000f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = 0.5f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.063996f, -0.117434f, -0.040199f }, { -0.063996f, -0.117432f, -0.040199f }, 0.165004f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000000f, -0.250478f, -0.002562f }, { { -0.642766f, 0.000000f, 0.000000f }, -0.766063f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.707107f, 0.000000f, 0.000000f }, -0.707107f } };
		bone->swingLimit = 35.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -17.5f * B3_DEG_TO_RAD, 17.5f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_neck;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "neck_01";
		bone->referenceFrame = (b3Transform){ { 0.000000f, 1.498783f, 0.024462f }, { { 0.987879f, 0.000000f, 0.000000f }, 0.155228f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;

		shapeDef.baseMaterial.friction = 0.2f;
		shapeDef.baseMaterial.rollingResistance = 0.05f;
		b3Capsule head = { { 0.000000f, -0.212340f, 0.000000f }, { 0.000000f, -0.087340f, 0.000000f }, 0.110000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &head );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000000f, -0.231017f, -0.046781f }, { { -0.516157f, -0.000002f, 0.000003f }, -0.856494f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.707108f, -0.000002f, 0.000002f }, -0.707106f } };
		bone->swingLimit = 30.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -17.5f * B3_DEG_TO_RAD, 17.5f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_thigh_l;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "thigh_l";
		bone->referenceFrame = (b3Transform){ { 0.092175f, 0.971562f, -0.011177f }, { { 0.471412f, 0.529510f, -0.499888f }, -0.497495f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { -0.047269f, 0.000001f, 0.000000f }, { -0.379769f, 0.000005f, 0.000000f }, 0.091539f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.092175f, 0.024656f, -0.012691f }, { { -0.460186f, -0.365368f, -0.637662f }, 0.498118f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.499997f, -0.500003f, 0.500002f }, 0.499998f } };
		bone->swingLimit = 30.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -10.0f * B3_DEG_TO_RAD, 10.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_calf_l;
		bone->parentIndex = bone_thigh_l;

		bodyDef.name = "calf_l";
		bone->referenceFrame = (b3Transform){ { 0.118720f, 0.534541f, -0.035362f }, { { 0.521188f, 0.480597f, -0.546380f }, -0.445935f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = 0.1f;
		shapeDef.baseMaterial.rollingResistance = 0.0f;
		b3Capsule shin = { { -0.445000f, 0.000001f, 0.000000f }, { 0.005000f, -0.000001f, 0.000000f }, 0.080000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &shin );

		shapeDef.density = 0.5f * defaultDensity;
		b3Capsule foot = { { -0.456371f, 0.129321f, 0.000013f }, { -0.359977f, 0.014394f, 0.000013f }, 0.070000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &foot );
		shapeDef.density = defaultDensity;

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -0.438494f, -0.000175f, 0.000000f }, { { 0.344866f, -0.938652f, 0.000002f }, -0.000008f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.008182f, -0.999967f, 0.000001f }, -0.000007f } };
		bone->twistLimit = (b3Vec2){ -30.0f * B3_DEG_TO_RAD, 30.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_thigh_r;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "thigh_r";
		bone->referenceFrame = (b3Transform){ { -0.092175f, 0.971562f, -0.011177f }, { { -0.497495f, 0.499888f, 0.529510f }, -0.471412f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.047269f, -0.000001f, 0.000000f }, { 0.379769f, -0.000005f, 0.000000f }, 0.092587f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { -0.092175f, 0.024656f, -0.012691f }, { { 0.380666f, 0.491846f, 0.488644f }, -0.611889f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.500002f, 0.499998f, -0.499998f }, 0.500002f } };
		bone->swingLimit = 30.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -10.0f * B3_DEG_TO_RAD, 10.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_calf_r;
		bone->parentIndex = bone_thigh_r;

		bodyDef.name = "calf_r";
		bone->referenceFrame = (b3Transform){ { -0.118720f, 0.534541f, -0.035362f }, { { -0.445935f, 0.546380f, 0.480597f }, -0.521188f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		shapeDef.baseMaterial.friction = 0.1f;
		shapeDef.baseMaterial.rollingResistance = 0.0f;
		b3Capsule shin = { { 0.445000f, -0.000002f, 0.000003f }, { -0.005000f, 0.000000f, 0.000003f }, 0.080000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &shin );

		shapeDef.density = 0.5f * defaultDensity;
		b3Capsule foot = { { 0.369113f, -0.006830f, 0.000017f }, { 0.465507f, -0.121757f, 0.000017f }, 0.070000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &foot );
		shapeDef.density = defaultDensity;

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { 0.438494f, 0.000175f, 0.000000f }, { { 0.000007f, 0.000005f, -0.344860f }, -0.938654f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { 0.000006f, 0.000005f, 0.008188f }, -0.999966f } };
		bone->twistLimit = (b3Vec2){ -30.0f * B3_DEG_TO_RAD, 30.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_upper_arm_l;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "upperarm_l";
		bone->referenceFrame = (b3Transform){ { 0.185817f, 1.443630f, 0.031306f }, { { 0.560220f, -0.307179f, 0.366233f }, -0.676512f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.296038f, 0.004386f, -0.000794f }, { -0.016086f, 0.004384f, -0.016127f }, 0.065000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.185817f, -0.175865f, -0.053625f }, { { -0.559466f, -0.213784f, -0.670199f }, 0.438323f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.500953f, -0.502617f, -0.509318f }, -0.486844f } };
		bone->swingLimit = 45.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -20.0f * B3_DEG_TO_RAD, 20.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.8f;
	}

	{
		Bone* bone = human->bones + bone_lower_arm_l;
		bone->parentIndex = bone_upper_arm_l;

		bodyDef.name = "lowerarm_l";
		bone->referenceFrame = (b3Transform){ { 0.347683f, 1.193455f, 0.029378f }, { { 0.406736f, -0.493012f, 0.089135f }, -0.763911f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;

		shapeDef.baseMaterial.friction = 0.3f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { 0.312329f, -0.008256f, -0.016454f }, { 0.033153f, 0.008259f, -0.002741f }, 0.060000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { 0.297979f, 0.000361f, 0.000000f }, { { 0.916726f, 0.399352f, 0.000839f }, -0.011456f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { 0.999922f, -0.003857f, 0.004659f }, -0.010908f } };
		bone->twistLimit = (b3Vec2){ -35.0f * B3_DEG_TO_RAD, 35.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.06f;
	}

	{
		Bone* bone = human->bones + bone_upper_arm_r;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "upperarm_r";
		bone->referenceFrame = (b3Transform){ { -0.185817f, 1.443630f, 0.031306f }, { { 0.676512f, 0.366233f, 0.307179f }, 0.560220f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;

		shapeDef.baseMaterial.friction = 0.6f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { -0.300237f, -0.000001f, -0.000319f }, { 0.011887f, 0.000001f, 0.015013f }, 0.065000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { -0.185817f, -0.175865f, -0.053625f }, { { 0.213793f, 0.559449f, 0.438334f }, -0.670204f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.509319f, 0.486843f, 0.500954f }, -0.502617f } };
		bone->swingLimit = 45.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -20.0f * B3_DEG_TO_RAD, 20.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.8f;
	}

	{
		Bone* bone = human->bones + bone_lower_arm_r;
		bone->parentIndex = bone_upper_arm_r;

		bodyDef.name = "lowerarm_r";
		bone->referenceFrame = (b3Transform){ { -0.347683f, 1.193455f, 0.029378f }, { { 0.763911f, 0.089135f, 0.493012f }, 0.406736f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;

		shapeDef.baseMaterial.friction = 0.3f;
		shapeDef.baseMaterial.rollingResistance = 0.1f;
		b3Capsule capsule = { { -0.312649f, -0.000003f, 0.016471f }, { -0.032986f, 0.000000f, 0.002732f }, 0.060000f };
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -0.297979f, -0.000361f, 0.000000f }, { { -0.405444f, 0.914059f, 0.010541f }, 0.000852f } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.002794f, 0.999936f, 0.010042f }, 0.004364f } };
		bone->twistLimit = (b3Vec2){ -35.0f * B3_DEG_TO_RAD, 35.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.06f;
	}

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		Bone* parent = human->bones + bone->parentIndex;

		b3BodyId bodyIdA = parent->bodyId;
		b3BodyId bodyIdB = bone->bodyId;

		bone->localFrameA.q = b3NormalizeQuat( bone->localFrameA.q );
		bone->localFrameB.q = b3NormalizeQuat( bone->localFrameB.q );

		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJointDef jointDef = b3DefaultRevoluteJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone->localFrameA;
			jointDef.base.localFrameB = bone->localFrameB;
			jointDef.enableLimit = true;
			jointDef.lowerAngle = bone->twistLimit.x;
			jointDef.upperAngle = bone->twistLimit.y;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = bone->jointFriction * frictionTorque;
			bone->jointId = b3CreateRevoluteJoint( worldId, &jointDef );
		}
		else if ( bone->jointType == b3_sphericalJoint )
		{
			b3SphericalJointDef jointDef = b3DefaultSphericalJointDef();
			jointDef.base.bodyIdA = bodyIdA;
			jointDef.base.bodyIdB = bodyIdB;
			jointDef.base.localFrameA = bone->localFrameA;
			jointDef.base.localFrameB = bone->localFrameB;
			jointDef.enableConeLimit = true;
			jointDef.coneAngle = bone->swingLimit;
			jointDef.enableTwistLimit = true;
			jointDef.lowerTwistAngle = bone->twistLimit.x;
			jointDef.upperTwistAngle = bone->twistLimit.y;
			jointDef.enableSpring = hertz > 0.0f;
			jointDef.hertz = hertz;
			jointDef.dampingRatio = dampingRatio;
			jointDef.enableMotor = true;
			jointDef.maxMotorTorque = bone->jointFriction * frictionTorque;
			bone->jointId = b3CreateSphericalJoint( worldId, &jointDef );
		}
	}

	// Disable some collisions
	human->filterJointCount = 0;
	b3FilterJointDef filterDef = b3DefaultFilterJointDef();
	filterDef.base.bodyIdA = human->bones[bone_thigh_l].bodyId;
	filterDef.base.bodyIdB = human->bones[bone_thigh_r].bodyId;
	human->filterJoints[human->filterJointCount++] = b3CreateFilterJoint( worldId, &filterDef );
	filterDef.base.bodyIdA = human->bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = human->bones[bone_upper_arm_l].bodyId;
	human->filterJoints[human->filterJointCount++] = b3CreateFilterJoint( worldId, &filterDef );
	filterDef.base.bodyIdA = human->bones[bone_neck].bodyId;
	filterDef.base.bodyIdB = human->bones[bone_upper_arm_r].bodyId;
	human->filterJoints[human->filterJointCount++] = b3CreateFilterJoint( worldId, &filterDef );

	human->isSpawned = true;
}

void DestroyHuman( Human* human )
{
	assert( human->isSpawned == true );

	for ( int i = 0; i < human->filterJointCount; ++i )
	{
		b3DestroyJoint( human->filterJoints[i], false );
		human->filterJoints[i] = b3_nullJointId;
	}

	for ( int i = 0; i < bone_count; ++i )
	{
		if ( B3_IS_NULL( human->bones[i].jointId ) )
		{
			continue;
		}

		b3DestroyJoint( human->bones[i].jointId, false );
		human->bones[i].jointId = b3_nullJointId;
	}

	for ( int i = 0; i < bone_count; ++i )
	{
		if ( B3_IS_NULL( human->bones[i].bodyId ) )
		{
			continue;
		}

		b3DestroyBody( human->bones[i].bodyId );
		human->bones[i].bodyId = b3_nullBodyId;
	}

	human->isSpawned = false;
}

void Human_SetVelocity( Human* human, b3Vec3 velocity )
{
	for ( int i = 0; i < bone_count; ++i )
	{
		b3BodyId bodyId = human->bones[i].bodyId;

		if ( B3_IS_NULL( bodyId ) )
		{
			continue;
		}

		b3Body_SetLinearVelocity( bodyId, velocity );
	}
}

void Human_ApplyRandomAngularImpulse( Human* human, float magnitude )
{
	assert( human->isSpawned == true );
	b3Vec3 range = { magnitude, magnitude, magnitude };
	b3Vec3 impulse = RandomVec3( b3Neg( range ), range );
	b3Body_ApplyAngularImpulse( human->bones[bone_spine_01].bodyId, impulse, true );
}

void Human_SetJointFrictionTorque( Human* human, float torque )
{
	assert( human->isSpawned == true );
	human->frictionTorque = torque;

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJoint_SetMaxMotorTorque( bone->jointId, bone->jointFriction * torque );
		}
		else
		{
			b3SphericalJoint_SetMaxMotorTorque( bone->jointId, bone->jointFriction * torque );
		}
	}
}

void Human_SetJointSpringHertz( Human* human, float hertz )
{
	assert( human->isSpawned == true );
	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJoint_SetSpringHertz( bone->jointId, hertz );
		}
		else
		{
			b3SphericalJoint_SetSpringHertz( bone->jointId, hertz );
		}
	}
}

void Human_SetJointDampingRatio( Human* human, float dampingRatio )
{
	assert( human->isSpawned == true );
	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		if ( bone->jointType == b3_revoluteJoint )
		{
			b3RevoluteJoint_SetSpringDampingRatio( bone->jointId, dampingRatio );
		}
		else
		{
			b3SphericalJoint_SetSpringDampingRatio( bone->jointId, dampingRatio );
		}
	}
}

void Human_AlignSpring( Human* human, b3WorldId worldId, b3BodyId groundId, float hertz, float dampingRatio )
{
	assert( human->isSpawned == true );

	Bone* bone = human->bones + bone_pelvis;
	assert( B3_IS_NULL( bone->jointId ) == true );
	b3Quat q = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
	b3Quat qb = b3Body_GetRotation( bone->bodyId );

	b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
	jointDef.base.bodyIdA = groundId;
	jointDef.base.bodyIdB = bone->bodyId;
	jointDef.base.localFrameA.q = q;
	jointDef.base.localFrameB.q = b3InvMulQuat( qb, q );
	jointDef.base.drawScale = 2.0f;
	jointDef.base.collideConnected = true;
	jointDef.hertz = hertz;
	jointDef.dampingRatio = dampingRatio;

	bone->jointId = b3CreateParallelJoint( worldId, &jointDef );
}

void Human_CreateMotorAnchors( Human* human, b3WorldId worldId )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_kinematicBody;

	b3MotorJointDef motorDef = b3DefaultMotorJointDef();
	motorDef.angularHertz = 5.0f;
	motorDef.angularDampingRatio = 1.0f;
	motorDef.linearHertz = 5.0f;
	motorDef.linearDampingRatio = 1.0f;
	motorDef.maxSpringForce = FLT_MAX;
	motorDef.maxSpringTorque = FLT_MAX;

	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		b3WorldTransform bodyTransform = b3Body_GetTransform( bone->bodyId );
		anchorDef.position = bodyTransform.p;
		anchorDef.rotation = bodyTransform.q;
		bone->anchorId = b3CreateBody( worldId, &anchorDef );

		motorDef.base.bodyIdA = bone->anchorId;
		motorDef.base.bodyIdB = bone->bodyId;

		bone->anchorJointId = b3CreateMotorJoint( worldId, &motorDef );
	}
}

void Human_CreateParallelAnchors( Human* human, b3WorldId worldId )
{
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_kinematicBody;

	b3Quat qFrameWorld = b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Vec3_axisY );
	b3ParallelJointDef jointDef = b3DefaultParallelJointDef();
	jointDef.hertz = 8.0f;
	jointDef.dampingRatio = 1.0f;
	jointDef.maxTorque = 800.0f;

	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		b3WorldTransform bodyTransform = b3Body_GetTransform( bone->bodyId );
		anchorDef.position = bodyTransform.p;
		anchorDef.rotation = bodyTransform.q;
		bone->anchorId = b3CreateBody( worldId, &anchorDef );

		jointDef.base.bodyIdA = bone->anchorId;
		jointDef.base.bodyIdB = bone->bodyId;

		b3Quat frameQuat = b3InvMulQuat( bodyTransform.q, qFrameWorld );
		jointDef.base.localFrameA.q = frameQuat;
		jointDef.base.localFrameB.q = frameQuat;

		bone->anchorJointId = b3CreateParallelJoint( worldId, &jointDef );
	}
}

void Human_SetBullet( Human* human, bool flag )
{
	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;
		b3Body_SetBullet( bone->bodyId, flag );
	}
}

#if 0
void Human::EnablePoseControl( b3World* world, float springHertz, bool poseControl )
{
	if ( poseControl == m_poseControl )
	{
		return;
	}

	if ( m_poseControl )
	{
		for ( int i = 0; i < bone_count; ++i )
		{
			Bone* bone = human->bones + i;

			world->RemoveJoint( bone->poseJoint, true );
			bone->poseJoint = nullptr;
		}

		world->DestroyBody( m_rootBody, true );
		m_rootBody = nullptr;
	}
	else
	{
		b3BodyDef rootDef = b3DefaultBodyDef();
		rootDef.type = b3_kinematicBody;
		rootDef.position = m_baseTransform.p;
		rootDef.rotation = m_baseTransform.q;
		m_rootBody = world->CreateBody( &rootDef );

		float dampingRatio = 0.9f;

		for ( int i = 0; i < bone_count; ++i )
		{
			Bone* bone = human->bones + i;

			b3Body* bodyIdA = m_rootBody;
			b3Body* bodyIdB = bone->bodyId;

			bool enableCollision = false;
			bool useBlockSolver = false;

			// bone->poseJoint = world->AddRigidJoint( bodyIdA, bodyIdB, bodyIdB->GetPosition(), enableCollision, useBlockSolver );
			bone->poseJoint = world->AddRigidJoint( bodyIdA, bone->referenceFrame, bodyIdB, b3Transform_identity,
													b3Transform_identity, enableCollision, useBlockSolver );
			bone->poseJoint->SetAngularFrequency( springHertz );
			bone->poseJoint->SetAngularDampingRatio( dampingRatio );
			bone->poseJoint->SetLinearFrequency( springHertz );
			bone->poseJoint->SetLinearDampingRatio( dampingRatio );
		}
	}

	m_poseControl = poseControl;
}

void Human::AdjustPoseControl( float springHertz )
{
	if ( m_poseControl == false )
	{
		return;
	}

	for ( int i = 0; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		bone->poseJoint->SetAngularFrequency( springHertz );
		bone->poseJoint->SetLinearFrequency( springHertz );
	}
}

void Human::DriveBase( const b3Transform& transform, float timeStep )
{
	m_rootBody->SetVelocityFromKeyframe( transform, timeStep );
	m_baseTransform = transform;
}

void Human::EnableMotors( bool enableMotors )
{
	if ( enableMotors == m_motorized )
	{
		return;
	}

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		if ( bone->joint == nullptr )
		{
			continue;
		}

		if ( bone->joint->mType == b3_revoluteJoint )
		{
			b3RevoluteJoint* joint = static_cast<b3RevoluteJoint*>( bone->joint );
			if ( enableMotors )
			{
				joint->SetMotorMode( B3_POSITION_MODE );
			}
			else
			{
				joint->SetMotorFriction( bone->jointFriction );
			}
		}
		else if ( bone->joint->mType == b3_sphericalJoint )
		{
			b3SphericalJoint* joint = static_cast<b3SphericalJoint*>( bone->joint );
			if ( enableMotors )
			{
				joint->SetMotorMode( B3_POSITION_MODE );
			}
			else
			{
				joint->SetMotorFriction( bone->jointFriction );
			}
		}
	}

	m_motorized = enableMotors;
}

void Human::AdjustMotors( float springHertz )
{
	if ( m_motorized == false )
	{
		return;
	}

	for ( int i = 1; i < bone_count; ++i )
	{
		Bone* bone = human->bones + i;

		if ( bone->joint == nullptr )
		{
			continue;
		}

		if ( bone->joint->mType == b3_revoluteJoint )
		{
			b3RevoluteJoint* joint = static_cast<b3RevoluteJoint*>( bone->joint );
			joint->mAngularSpring.Frequency = springHertz;
		}
		else if ( bone->joint->mType == b3_sphericalJoint )
		{
			b3SphericalJoint* joint = static_cast<b3SphericalJoint*>( bone->joint );
			joint->mAngularSpring.Frequency = springHertz;
		}
	}
}
#endif

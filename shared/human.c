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
	// shapeDef.friction = 0.2f;
	shapeDef.baseMaterial.rollingResistance = 0.2f;

	b3HexColor shirtColor = b3_colorMediumTurquoise;
	b3HexColor pantColor = b3_colorDodgerBlue;
	b3HexColor skinColors[4] = { b3_colorNavajoWhite, b3_colorLightYellow, b3_colorPeru, b3_colorTan };
	b3HexColor skinColor = skinColors[groupIndex % 4];

	{
		Bone* bone = human->bones + bone_pelvis;

		bone->parentIndex = -1;

		bodyDef.name = "pelvis";
		bone->referenceFrame = (b3Transform){ { 0.0f, 0.932087f, -0.051708f }, { { 0.739169f, 0.0f, 0.0f }, 0.673520f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.07f, 0.0f, -0.08f }, { -0.07f, 0.0f, -0.08f }, 0.13f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;

		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );
	}

	{
		Bone* bone = human->bones + bone_spine_01;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "spine_01";
		bone->referenceFrame = (b3Transform){ { 0.0f, 1.113505f, -0.03481f }, { { 0.739973f, 0.0f, 0.0f }, 0.672637f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		// bodyDef.type = b3_staticBody;
		bone->bodyId = b3CreateBody( worldId, &bodyDef );
		bodyDef.type = b3_dynamicBody;

		b3Capsule capsule = { { 0.06f, -0.0f, -0.052264f }, { -0.06f, 0.0f, -0.052264f }, 0.12f };
		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000000, 0.000000, -0.182204 }, { { -0.999999, 0.000000, -0.000000 }, 0.001194 } };
		bone->localFrameB = (b3Transform){ { 0.000000, 0.000000, -0.007736 }, { { -1.000000, 0.000000, -0.000000 }, 0.000000 } };
		bone->swingLimit = 25.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -15.0f * B3_DEG_TO_RAD, 15.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_spine_02;
		bone->parentIndex = bone_spine_01;

		// bodyDef.name = "spine_02";
		bone->referenceFrame = (b3Transform){ { 0.0f, 1.194336f, -0.027087f }, { { 0.703611f, 0.0f, 0.0f }, 0.710586f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.08f, -0.015133f, -0.091801f }, { -0.08f, -0.015133f, -0.091801f }, 0.10f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA =
			(b3Transform){ { 0.000000, -0.000000, -0.088935 }, { { -0.998619, -0.000000, 0.000000 }, -0.052540 } };
		bone->localFrameB = (b3Transform){ { -0.000000, 0.000000, -0.008199 }, { { -1.000000, 0.000000, -0.000000 }, 0.000000 } };
		bone->swingLimit = 25.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -15.0f * B3_DEG_TO_RAD, 15.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_spine_03;
		bone->parentIndex = bone_spine_02;

		bodyDef.name = "spine_03";
		bone->referenceFrame =
			(b3Transform){ { -0.0f, 1.31043f, -0.028232f }, { { 0.669856f, 0.000001f, -0.000001f }, 0.742491f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.11f, -0.039753f, -0.13f }, { -0.11f, -0.039753f, -0.13f }, 0.145f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA =
			(b3Transform){ { -0.000000, 0.000000, -0.124298 }, { { -0.998921, 0.000001, -0.000001 }, -0.046434 } };
		bone->localFrameB = (b3Transform){ { 0.000000, 0.000000, 0.000000 }, { { -1.000000, 0.000000, -0.000001 }, 0.000000 } };
		bone->swingLimit = 15.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -10.0f * B3_DEG_TO_RAD, 10.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_neck;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "neck";
		bone->referenceFrame = (b3Transform){ { 0.0f, 1.575582f, -0.055837f }, { { 0.879922f, 0.0f, 0.0f }, 0.475118f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { -0.000001f, -0.0f, -0.02f }, { 0.0f, -0.005f, -0.08f }, 0.07f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.000001, -0.000259, -0.266585 }, { { -0.942192, -0.000001, 0.000000 }, 0.335074 } };
		bone->localFrameB = (b3Transform){ { 0.000000, 0.000000, 0.000000 }, { { -1.000000, 0.000000, -0.000001 }, 0.000000 } };
		bone->swingLimit = 45.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -15.0f * B3_DEG_TO_RAD, 15.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.8f;
	}

	{
		Bone* bone = human->bones + bone_head;
		bone->parentIndex = bone_neck;

		bodyDef.name = "head";
		bone->referenceFrame = (b3Transform){ { 0.0f, 1.653348f, -0.003241f }, { { 0.750288f, 0.0f, 0.0f }, 0.661111f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { -0.000001f, 0.016892f, -0.05869f }, { 0.0f, -0.003629f, -0.115072f }, 0.0975f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA =
			(b3Transform){ { 0.000000, 0.001321, -0.093873 }, { { -0.974301, -0.000000, -0.000000 }, -0.225251 } };
		bone->localFrameB = (b3Transform){ { 0.000000, 0.001268, -0.005104 }, { { -1.000000, 0.000000, -0.00000 }, 0.000000 } };
		bone->swingLimit = 15.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -15.0f * B3_DEG_TO_RAD, 15.0f * B3_DEG_TO_RAD };
		bone->jointFriction = 0.4f;
	}

	{
		Bone* bone = human->bones + bone_thigh_l;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "thigh_l";
		bone->referenceFrame =
			(b3Transform){ { 0.090416f, 0.986104f, -0.035090f }, { { -0.703287f, -0.070715f, 0.053866f }, 0.705327f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.023719f, 0.006008f, -0.039068f }, { -0.064492f, -0.004664f, -0.424718f }, 0.09f };
		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.05f, 0.011537f, -0.055325f }, { { -0.714896, -0.022305, -0.698361 }, -0.026790 } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.002064, 0.758987, 0.017046 }, 0.650880 } };
		bone->swingLimit = 10.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -60.0f * B3_DEG_TO_RAD, 40.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_calf_l;
		bone->parentIndex = bone_thigh_l;

		bodyDef.name = "calf_l";
		bone->referenceFrame =
			(b3Transform){ { 0.101198f, 0.527027f, -0.037374f }, { { -0.653328f, -0.066860f, 0.058582f }, 0.751838f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.001778f, 0.0f, 0.009841f }, { -0.078577f, 0.014707f, -0.41816f }, 0.075f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -0.069989, 0.000253, -0.453844 }, { { -0.000677, 0.760087, 0.105674 }, 0.641171 } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.044589, 0.765540, 0.053368 }, 0.639619 } };
		bone->twistLimit = (b3Vec2){ -5.0f * B3_DEG_TO_RAD, 45.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_thigh_r;
		bone->parentIndex = bone_pelvis;

		bodyDef.name = "thigh_r";
		bone->referenceFrame =
			(b3Transform){ { -0.090416f, 0.986104f, -0.03509f }, { { -0.703287f, 0.070715f, -0.053865f }, 0.705326f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { -0.023719f, 0.006008f, -0.039068f }, { 0.064492f, -0.004664f, -0.424718f }, 0.09f };
		shapeDef.filter.groupIndex = -groupIndex;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { -0.05, 0.011537, -0.055326 }, { { -0.039089, -0.714094, 0.043177 }, 0.697623 } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { 0.758805, -0.019886, -0.651012 }, -0.001759 } };
		bone->swingLimit = 10.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -30.0f * B3_DEG_TO_RAD, 60.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_calf_r;
		bone->parentIndex = bone_thigh_r;

		bodyDef.name = "calf_r";
		bone->referenceFrame =
			(b3Transform){ { -0.101198f, 0.527027f, -0.037373f }, { { -0.653327f, 0.06686f, -0.058582f }, 0.751839f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { -0.001820f, 0.0f, 0.010071f }, { 0.077883f, 0.014825f, -0.418047f }, 0.075f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? pantColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { 0.069988, 0.000253, -0.453844 }, { { 0.760086, -0.000675, -0.641171 }, -0.105676 } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { 0.765540, -0.044589, -0.639619 }, -0.053368 } };
		bone->twistLimit = (b3Vec2){ -45.0f * B3_DEG_TO_RAD, 5.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_upper_arm_l;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "upper_arm_l";
		bone->referenceFrame =
			(b3Transform){ { 0.20378f, 1.484275f, -0.115897f }, { { 0.143082f, 0.695980f, -0.690130f }, 0.13733f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { -0.091118f, 0.037775f, 0.229719f }, 0.075f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA = (b3Transform){ { 0.203780, -0.069369, -0.181921 }, { { -0.278486, 0.445600, -0.097014 }, 0.845266 } };
		bone->localFrameB = (b3Transform){ { 0.000000, 0.000000, 0.000000 }, { { -0.201396, -0.001586, 0.901850 }, 0.382234 } };
		bone->swingLimit = 60.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -5.0f * B3_DEG_TO_RAD, 5.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_lower_arm_l;
		bone->parentIndex = bone_upper_arm_l;

		bodyDef.name = "lower_arm_l";
		bone->referenceFrame =
			(b3Transform){ { 0.305614f, 1.242908f, -0.117599f }, { { 0.165048f, 0.563437f, -0.802002f }, 0.109959f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { -0.142406f, 0.039392f, 0.261092f }, 0.05f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { -0.095482, 0.039584, 0.240723 }, { { 0.512487, -0.180629, 0.839474 }, 0.003742 } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { 0.503803, -0.029831, 0.858168 }, 0.094017 } };
		bone->twistLimit = (b3Vec2){ -5.0f * B3_DEG_TO_RAD, 60.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_upper_arm_r;
		bone->parentIndex = bone_spine_03;

		bodyDef.name = "upper_arm_r";
		bone->referenceFrame =
			(b3Transform){ { -0.20378f, 1.484276f, -0.115899f }, { { 0.143083f, -0.695978f, 0.690132f }, 0.137329f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { 0.091118f, 0.037775f, 0.229718f }, 0.075f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? shirtColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_sphericalJoint;
		bone->localFrameA =
			(b3Transform){ { -0.203779, -0.069371, -0.181922 }, { { -0.253621, -0.414842, 0.106962 }, 0.867261 } };
		bone->localFrameB = (b3Transform){ { 0.000000, 0.000000, 0.000000 }, { { -0.201397, 0.001587, -0.901850 }, 0.382233 } };
		bone->swingLimit = 60.0f * B3_DEG_TO_RAD;
		bone->twistLimit = (b3Vec2){ -5.0f * B3_DEG_TO_RAD, 5.0f * B3_DEG_TO_RAD };
	}

	{
		Bone* bone = human->bones + bone_lower_arm_r;
		bone->parentIndex = bone_upper_arm_r;

		bodyDef.name = "lower_arm_r";
		bone->referenceFrame =
			(b3Transform){ { -0.305614f, 1.242907f, -0.117599f }, { { 0.165048f, -0.563437f, 0.802002f }, 0.109959f } };
		bodyDef.rotation = bone->referenceFrame.q;
		bodyDef.position = b3OffsetPos( position, bone->referenceFrame.p );
		bone->bodyId = b3CreateBody( worldId, &bodyDef );

		b3Capsule capsule = { { 0.0f, 0.0f, 0.0f }, { 0.142406f, 0.039392f, 0.261092f }, 0.05f };
		shapeDef.filter.groupIndex = 0;
		shapeDef.baseMaterial.customColor = colorize ? skinColor : 0;
		b3CreateCapsuleShape( bone->bodyId, &shapeDef, &capsule );

		bone->jointType = b3_revoluteJoint;
		bone->localFrameA = (b3Transform){ { 0.095484, 0.039585, 0.240723 }, { { -0.180627, 0.512487, -0.003744 }, -0.839474 } };
		bone->localFrameB = (b3Transform){ { 0.0f, 0.0f, 0.0f }, { { -0.029831, 0.503803, -0.094017 }, -0.858169 } };
		bone->twistLimit = (b3Vec2){ -60.0f * B3_DEG_TO_RAD, 5.0f * B3_DEG_TO_RAD };
	}

	// float dampingRatio = 0.9f;
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
	b3FilterJointDef filterDef = b3DefaultFilterJointDef();
	filterDef.base.bodyIdA = human->bones[bone_thigh_l].bodyId;
	filterDef.base.bodyIdB = human->bones[bone_thigh_r].bodyId;
	human->filterJoints[0] = b3CreateFilterJoint( worldId, &filterDef );
	human->filterJointCount = 1;

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

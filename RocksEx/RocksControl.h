#pragma once

#include "RocksExtern_Delta.h"
#include "RocksExtren_Spiral.h"

ROCKS_MECH  m_mech;

typedef struct cartesianCoordinate
{
	double x;
	double y;
	double z;
}CARTESIAN_COORD;

typedef struct trajectoryPars
{
	double velocity;
	double acceleration;
	double splineTime;
}TRAJ_PARS;

NYCE_STATUS RocksInitDelta(const uint32_t &axesNum, const SAC_AXIS* const axId)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	// Create mechanism
	// ----------------
	m_mech.nrOfJoints = axesNum; // X1, X2, Y and Z
	m_mech.dof[ 0 ]	  = TRUE;     // X
	m_mech.dof[ 1 ]	  = TRUE;     // Y
	m_mech.dof[ 2 ]	  = TRUE;     // Z
	m_mech.dof[ 3 ]	  = FALSE;    // Rx
	m_mech.dof[ 4 ]	  = FALSE;    // Ry
	m_mech.dof[ 5 ]	  = FALSE;    // Rz
	for ( uint32_t ax = 0; ax < axesNum; ax++ )
	{
		m_mech.jointAxisId[ ax ] = axId[ ax ];
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechCreate( &m_mech );

	double e = 40;     
	double f = 105;    
	double re = 194;
	double rf = 90;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksSetMechParsDelta(f, e, rf, re);

	double rate_angle2pu = 131072 * 11 / (2 * M_PI);
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksSetPuRateDelta(rate_angle2pu, rate_angle2pu, rate_angle2pu);

	return nyceStatus;
}

NYCE_STATUS RocksInitCartesian(const uint32_t &axesNum, const SAC_AXIS* const axId)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	// Create mechanism
	// ----------------
	m_mech.nrOfJoints = axesNum; // X1, X2, Y and Z
	m_mech.dof[ 0 ]	  = TRUE;     // X
	m_mech.dof[ 1 ]	  = TRUE;     // Y
	m_mech.dof[ 2 ]	  = TRUE;     // Z
	m_mech.dof[ 3 ]	  = FALSE;    // Rx
	m_mech.dof[ 4 ]	  = FALSE;    // Ry
	m_mech.dof[ 5 ]	  = FALSE;    // Rz
	for ( uint32_t ax = 0; ax < axesNum; ax++ )
	{
		m_mech.jointAxisId[ ax ] = axId[ ax ];
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechCreate( &m_mech );

	return nyceStatus;
}

NYCE_STATUS RocksTerm()
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	if (m_mech.var.jointBuffersAllocated)
	{
		int i;
		for (i = 0; i < ROCKS_MECH_MAX_NR_OF_JOINTS; i++)
		{
			if (m_mech.var.pJointPositionBufferC[i] != NULL)
			{
				free(m_mech.var.pJointPositionBufferC[i]);
				m_mech.var.pJointPositionBufferC[i] = NULL;
			}
			if (m_mech.var.pJointVelocityBufferC[i] != NULL)
			{
				free(m_mech.var.pJointVelocityBufferC[i]);
				m_mech.var.pJointVelocityBufferC[i] = NULL;
			}
		}
	}

	// Delete mechanism
	// ----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechDelete( &m_mech );

	return nyceStatus;
}

NYCE_STATUS RocksPtpDelta(const CARTESIAN_COORD &carCoord, const TRAJ_PARS &trajPars, const double timeout = SAC_INDEFINITE)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_TRAJ_SINE_ACC_PTP_PARS sinePtpPars;
	ROCKS_KIN_INV_PARS kinPars;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sinePtpPars.startPos);
	sinePtpPars.endPos[0] = carCoord.x;
	sinePtpPars.endPos[1] = carCoord.y;
	sinePtpPars.endPos[2] = carCoord.z;
	sinePtpPars.maxVelocity = trajPars.velocity;
	sinePtpPars.maxAcceleration = trajPars.acceleration;
	sinePtpPars.splineTime = trajPars.splineTime;
	sinePtpPars.maxNrOfSplines = 0;
	sinePtpPars.pPositionSplineBuffer = NULL;
	sinePtpPars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccPtp(&m_mech,&sinePtpPars);

	for (int ax = 0; ax < 3; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

	// Synchronize on motion complete
	// ------------------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout);

	return nyceStatus;
}

NYCE_STATUS RocksCricleDelta(const CARTESIAN_COORD &centerOffset, const double &angle, const TRAJ_PARS &trajPars, const double &timeout = SAC_INDEFINITE, const int &repeatTimes = -1)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccCirclePars;
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	const double radius(sqrt(centerOffset.x * centerOffset.x + centerOffset.y * centerOffset.y + centerOffset.z * centerOffset.z));

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, 	sineAccCirclePars.startPos);

	sineAccCirclePars.maxVelocity = trajPars.velocity;
	sineAccCirclePars.maxAcceleration = trajPars.acceleration;
	sineAccCirclePars.splineTime = trajPars.splineTime;
	sineAccCirclePars.center[ 0 ] = sineAccCirclePars.startPos[0] - radius;
	sineAccCirclePars.center[ 1 ] = sineAccCirclePars.startPos[1];
	sineAccCirclePars.angle = angle;
	sineAccCirclePars.plane = ROCKS_PLANE_XY;
	sineAccCirclePars.maxNrOfSplines = 0;
	sineAccCirclePars.pPositionSplineBuffer = NULL;
	sineAccCirclePars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccCirclePars);
	
	ROCKS_POSE pose;
	CalcRotateAngle(pose.r.y, -centerOffset.x, -centerOffset.z);
	pose.r.x = 0;
	pose.r.y = pose.r.y;
	pose.r.z = 0;
	pose.t.x = 0;
	pose.t.y = 0;
	pose.t.z = 0;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin( &m_mech, &pose );

	switch(repeatTimes)
	{
	case -1:
		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
		{
			kinPars.pJointPositionBuffer[ ax ] = NULL;
			kinPars.pJointVelocityBuffer[ ax ] = NULL;
		}
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		break;
	case 0:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		while(nyceStatus == NYCE_OK)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;
	default:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		for(int i = 0; i < repeatTimes && nyceStatus == NYCE_OK; ++i)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;
	}
	
	return nyceStatus;
}
NYCE_STATUS RocksCricleCartesian(const CARTESIAN_COORD &centerOffset, const double &angle, const TRAJ_PARS &trajPars, const double &timeout = SAC_INDEFINITE, const int &repeatTimes = -1)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccCirclePars;
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	const double radius(sqrt(centerOffset.x * centerOffset.x + centerOffset.y * centerOffset.y + centerOffset.z * centerOffset.z));

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinCartesianPosition(&m_mech, 	sineAccCirclePars.startPos);

	sineAccCirclePars.maxVelocity = trajPars.velocity;
	sineAccCirclePars.maxAcceleration = trajPars.acceleration;
	sineAccCirclePars.splineTime = trajPars.splineTime;
	sineAccCirclePars.center[ 0 ] = sineAccCirclePars.startPos[0] - radius;
	sineAccCirclePars.center[ 1 ] = sineAccCirclePars.startPos[1];
	sineAccCirclePars.angle = angle;
	sineAccCirclePars.plane = ROCKS_PLANE_XY;
	sineAccCirclePars.maxNrOfSplines = 0;
	sineAccCirclePars.pPositionSplineBuffer = NULL;
	sineAccCirclePars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccCirclePars);

	ROCKS_POSE pose;
	CalcRotateAngle(pose.r.y, -centerOffset.x, -centerOffset.z);
	pose.r.x = 0;
	pose.r.y = pose.r.y;
	pose.r.z = 0;
	pose.t.x = 0;
	pose.t.y = 0;
	pose.t.z = 0;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin( &m_mech, &pose );

	switch(repeatTimes)
	{
	case -1:
		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
		{
			kinPars.pJointPositionBuffer[ ax ] = NULL;
			kinPars.pJointVelocityBuffer[ ax ] = NULL;
		}
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseCartesian( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		break;
	case 0:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		while(nyceStatus == NYCE_OK)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseCartesian( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;
	default:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		for(int i = 0; i < repeatTimes && nyceStatus == NYCE_OK; ++i)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseCartesian( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;
	}
	return nyceStatus;	
}

const double DOOR_SPLINETIME = 0.001;
const double DOOR_SPEED = 500;
const double DOOR_ACC = DOOR_SPEED * 100;
const double DOOR_HEIGHT = 20;
const double DOOR_WIDTH = 20;
const double DOOR_FILLET = 40;

const double OPT_DOOR_POINT_1[3] = {-65,0,-220};
const double OPT_DOOR_POINT_2[3] = {-25,0,-170};
const double OPT_DOOR_POINT_3[3] = { 25,0,-170};
const double OPT_DOOR_POINT_4[3] = { 65,0,-220};		

ROCKS_TRAJ_SEGMENT_START_PARS segStartPars;
ROCKS_TRAJ_SEGMENT_LINE_PARS segLinePars1,segLinePars2,segLinePars3,segLinePars4;
ROCKS_TRAJ_SEGMENT_ARC_PARS segArcPars1,segArcPars2;

NYCE_STATUS RocksDoorDelta()
{
	NYCE_STATUS nyceStatus(NYCE_OK);
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	double line = sqrt((-138.75 - OPT_DOOR_POINT_2[2]) * (-138.75 - OPT_DOOR_POINT_2[2]) + OPT_DOOR_POINT_2[0] * OPT_DOOR_POINT_2[0]);
	double z = -(138.75 + line * sqrt(4100.0) / 50);
	double center[3] = {0,0,z};

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, segStartPars.startPos);

	segStartPars.splineTime = DOOR_SPLINETIME;
	segStartPars.maxNrOfSplines = 0;
	segStartPars.pPositionSplineBuffer = NULL;
	segStartPars.pVelocitySplineBuffer = NULL;

	segLinePars1.plane = ROCKS_PLANE_ZX;
	segLinePars1.endPos[0] = OPT_DOOR_POINT_2[0];
	segLinePars1.endPos[1] = OPT_DOOR_POINT_2[2];
	segLinePars1.endVelocity = DOOR_SPEED;
	segLinePars1.maxAcceleration = DOOR_SPEED * 100;

	segArcPars1.plane = ROCKS_PLANE_ZX;
	segArcPars1.center[0] = center[0];
	segArcPars1.center[1] = center[2];
	segArcPars1.endPos[0] = OPT_DOOR_POINT_3[0];
	segArcPars1.endPos[1] = OPT_DOOR_POINT_3[2];
	segArcPars1.endVelocity = DOOR_SPEED;
	segArcPars1.maxAcceleration = DOOR_SPEED * 100;
	segArcPars1.positiveAngle = TRUE;

	segLinePars2.plane = ROCKS_PLANE_ZX;
	segLinePars2.endPos[0] = OPT_DOOR_POINT_4[0];
	segLinePars2.endPos[1] = OPT_DOOR_POINT_4[2];
	segLinePars2.endVelocity = 0;
	segLinePars2.maxAcceleration = DOOR_SPEED * 100;

	segLinePars3.plane = ROCKS_PLANE_ZX;
	segLinePars3.endPos[0] = OPT_DOOR_POINT_3[0];
	segLinePars3.endPos[1] = OPT_DOOR_POINT_3[2];
	segLinePars3.endVelocity = DOOR_SPEED;
	segLinePars3.maxAcceleration = DOOR_SPEED * 100;

	segArcPars2.plane = ROCKS_PLANE_ZX;
	segArcPars2.center[0] = center[0];
	segArcPars2.center[1] = center[2];
	segArcPars2.endPos[0] = OPT_DOOR_POINT_2[0];
	segArcPars2.endPos[1] = OPT_DOOR_POINT_2[2];
	segArcPars2.endVelocity = DOOR_SPEED;
	segArcPars2.maxAcceleration = DOOR_SPEED * 100;
	segArcPars2.positiveAngle = FALSE;

	segLinePars4.plane = ROCKS_PLANE_ZX;
	segLinePars4.endPos[0] = OPT_DOOR_POINT_1[0];
	segLinePars4.endPos[1] = OPT_DOOR_POINT_1[2];
	segLinePars4.endVelocity = 0;
	segLinePars4.maxAcceleration = DOOR_SPEED * 100;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech,&segStartPars);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars1);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars1);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars2);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars3);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars2);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars4);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );
	
	while(nyceStatus == NYCE_OK)
	{
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
		{
			kinPars.pJointPositionBuffer[ ax ] = NULL;
			kinPars.pJointVelocityBuffer[ ax ] = NULL;
		}
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE );
	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );

	return nyceStatus;
}

const double DOOR_EX_SPLINETIME = 0.01;
const double DOOR_EX_SPEED = 100;
const double DOOR_EX_ACC = DOOR_SPEED * 100;
const double SPIRAL_MAX_RADIAL_SPEED = 100;
const double SPIRAL_MAX_RADIAL_ACC = SPIRAL_MAX_RADIAL_SPEED * 100;


ROCKS_TRAJ_SEGMENT_SPIRAL_PARS_EX segSpiralPars1, segSpiralPars2, segSpiralPars3, segSpiralPars4;

NYCE_STATUS RocksSpiralExDoorDelta()
{
	NYCE_STATUS nyceStatus(NYCE_OK);
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	double center[2] = {(OPT_DOOR_POINT_2[0] + OPT_DOOR_POINT_3[0]) / 2.0, OPT_DOOR_POINT_1[2]};
	double radius[2] = {sqrt((OPT_DOOR_POINT_2[0] - center[0]) * (OPT_DOOR_POINT_2[0] - center[0]) + (OPT_DOOR_POINT_2[2] - center[1]) * (OPT_DOOR_POINT_2[2] - center[1])), sqrt((OPT_DOOR_POINT_3[0] - center[0]) * (OPT_DOOR_POINT_3[0] - center[0]) + (OPT_DOOR_POINT_3[2] - center[1]) * (OPT_DOOR_POINT_3[2] - center[1]))};
	double angleSpeed(DOOR_EX_SPEED / radius[1]);
	double angleMaxAcc(angleSpeed * 100);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, segStartPars.startPos);

	segStartPars.splineTime = DOOR_EX_SPLINETIME;
	segStartPars.maxNrOfSplines = 0;
	segStartPars.pPositionSplineBuffer = NULL;
	segStartPars.pVelocitySplineBuffer = NULL;

	segSpiralPars1.plane = ROCKS_PLANE_ZX;
	segSpiralPars1.center[0] = center[0];
	segSpiralPars1.center[1] = center[1];
	segSpiralPars1.endPos[0] = OPT_DOOR_POINT_2[0];
	segSpiralPars1.endPos[1] = OPT_DOOR_POINT_2[2];
	segSpiralPars1.endAngleVelocity = angleSpeed;
	segSpiralPars1.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars1.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars1.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars1.originOffset.r.x = 0;
	segSpiralPars1.originOffset.r.y = 0;
	segSpiralPars1.originOffset.r.z = 0;
	segSpiralPars1.originOffset.t.x = 0;
	segSpiralPars1.originOffset.t.y = 0;
	segSpiralPars1.originOffset.t.z = 0;
	
	segArcPars1.plane = ROCKS_PLANE_ZX;
	segArcPars1.center[0] = center[0];
	segArcPars1.center[1] = center[1];
	segArcPars1.endPos[0] = OPT_DOOR_POINT_3[0];
	segArcPars1.endPos[1] = OPT_DOOR_POINT_3[2];
	segArcPars1.endVelocity = DOOR_EX_SPEED;
	segArcPars1.maxAcceleration = DOOR_EX_ACC;
	segArcPars1.positiveAngle = TRUE;
	segArcPars1.originOffset.r.x = 0;
	segArcPars1.originOffset.r.y = 0;
	segArcPars1.originOffset.r.z = 0;
	segArcPars1.originOffset.t.x = 0;
	segArcPars1.originOffset.t.y = 0;
	segArcPars1.originOffset.t.z = 0;

	segSpiralPars2.plane = ROCKS_PLANE_ZX;
	segSpiralPars2.center[0] = center[0];
	segSpiralPars2.center[1] = center[1];
	segSpiralPars2.endPos[0] = OPT_DOOR_POINT_4[0];
	segSpiralPars2.endPos[1] = OPT_DOOR_POINT_4[2];
	segSpiralPars2.endAngleVelocity = 0;
	segSpiralPars2.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars2.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars2.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars2.originOffset.r.x = 0;
	segSpiralPars2.originOffset.r.y = 0;
	segSpiralPars2.originOffset.r.z = 0;
	segSpiralPars2.originOffset.t.x = 0;
	segSpiralPars2.originOffset.t.y = 0;
	segSpiralPars2.originOffset.t.z = 0;

	segSpiralPars3.plane = ROCKS_PLANE_ZX;
	segSpiralPars3.center[0] = center[0];
	segSpiralPars3.center[1] = center[1];
	segSpiralPars3.endPos[0] = OPT_DOOR_POINT_3[0];
	segSpiralPars3.endPos[1] = OPT_DOOR_POINT_3[2];
	segSpiralPars3.endAngleVelocity = angleSpeed;
	segSpiralPars3.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars3.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars3.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars3.originOffset.r.x = 0;
	segSpiralPars3.originOffset.r.y = 0;
	segSpiralPars3.originOffset.r.z = 0;
	segSpiralPars3.originOffset.t.x = 0;
	segSpiralPars3.originOffset.t.y = 0;
	segSpiralPars3.originOffset.t.z = 0;

	segArcPars2.plane = ROCKS_PLANE_ZX;
	segArcPars2.center[0] = center[0];
	segArcPars2.center[1] = center[1];
	segArcPars2.endPos[0] = OPT_DOOR_POINT_2[0];
	segArcPars2.endPos[1] = OPT_DOOR_POINT_2[2];
	segArcPars2.endVelocity = DOOR_EX_SPEED;
	segArcPars2.maxAcceleration = DOOR_EX_ACC;
	segArcPars2.positiveAngle = FALSE;
	segArcPars2.originOffset.r.x = 0;
	segArcPars2.originOffset.r.y = 0;
	segArcPars2.originOffset.r.z = 0;
	segArcPars2.originOffset.t.x = 0;
	segArcPars2.originOffset.t.y = 0;
	segArcPars2.originOffset.t.z = 0;

	segSpiralPars4.plane = ROCKS_PLANE_ZX;
	segSpiralPars4.center[0] = center[0];
	segSpiralPars4.center[1] = center[1];
	segSpiralPars4.endPos[0] = OPT_DOOR_POINT_1[0];
	segSpiralPars4.endPos[1] = OPT_DOOR_POINT_1[2];
	segSpiralPars4.endAngleVelocity = 0;
	segSpiralPars4.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars4.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars4.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars4.originOffset.r.x = 0;
	segSpiralPars4.originOffset.r.y = 0;
	segSpiralPars4.originOffset.r.z = 0;
	segSpiralPars4.originOffset.t.x = 0;
	segSpiralPars4.originOffset.t.y = 0;
	segSpiralPars4.originOffset.t.z = 0;

	

//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

	while(nyceStatus == NYCE_OK)
	{
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);
// 
// 		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
// 		{
// 			kinPars.pJointPositionBuffer[ ax ] = NULL;
// 			kinPars.pJointVelocityBuffer[ ax ] = NULL;
// 		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech,&segStartPars);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars1);
//		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars1);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars2);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars3);
//		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars2);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars4);

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE );
	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );

	return nyceStatus;
}
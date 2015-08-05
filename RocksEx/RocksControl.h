#pragma once

#include "RocksExtern_Delta.h"

ROCKS_MECH  m_mech;
ROCKS_TRAJ_PATH rocksTrajPath;
ROCKS_KIN_INV_PARS kinPars;

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

NYCE_STATUS RocksInit(const uint32_t &axesNum, const SAC_AXIS* const axId)
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

NYCE_STATUS RocksTerm()
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath);

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
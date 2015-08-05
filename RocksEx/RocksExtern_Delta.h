#pragma once

#include <nyceapi.h>

#include "NyceExDefs.h"

#include "CoordinateAlgorithm.h"
#include "DeltaKinAlgorithm.h"

#define DELTA_AXES_NUM 3

DELTA_MECH_PARS delta_mech_pars;
double rate_angle2pu[ROCKS_MECH_MAX_NR_OF_JOINTS];

HANDLE evExportDatas(CreateEvent(NULL,TRUE,FALSE,NULL));

NYCE_STATUS RocksSetMechParsDelta(const double &lenOfBasePlatform, const double &lenOfTravelPlatform, const double &lenOfActiveArm, const double &lenOfPassiveArm)
{
	delta_mech_pars.e = lenOfTravelPlatform;
	delta_mech_pars.f = lenOfBasePlatform;
	delta_mech_pars.re = lenOfPassiveArm;
	delta_mech_pars.rf = lenOfActiveArm;

	return NYCE_OK;
}

NYCE_STATUS RocksSetPuRateDelta(const double &rate1, const double &rate2, const double &rate3)
{
	rate_angle2pu[0] = rate1;
	rate_angle2pu[1] = rate2;
	rate_angle2pu[2] = rate3;

	return NYCE_OK;
}

inline NYCE_STATUS RocksCheckRate()
{
	for (uint32_t ax = 0; ax < DELTA_AXES_NUM; ax++)
	{
		if (rate_angle2pu[ax] < 1)
			return ROCKS_ERR_PU_RATE_ERROR;
	}
	return NYCE_OK;
}

inline void ConvertAngleToPU(const double &angle, double &positionUnit, const uint32_t &index)
{
	positionUnit = angle * rate_angle2pu[index];
}

inline void ConvertPUToAngle(const double &positionUnit, double &angle, const uint32_t &index)
{
	angle = positionUnit / rate_angle2pu[index];
}

NYCE_STATUS RocksKinForwardDelta(ROCKS_MECH* pMech, const double pJointPos[], double pMechPos[])
{
	if (delta_mech_pars.e <= 0 || delta_mech_pars.f <= 0 || delta_mech_pars.re <= 0 || delta_mech_pars.rf <= 0 )
		return ROCKS_ERR_DELTA_PARS_ERROR;

	NYCE_STATUS status = RocksCheckRate();
	if (NyceError(status))
		return status;

	ZeroMemory(pMechPos, sizeof(double) * ROCKS_MECH_MAX_DOF);

	double iointAnglePos[ROCKS_MECH_MAX_NR_OF_JOINTS];
	for (uint32_t ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ax++)
	{
		ConvertPUToAngle(pJointPos[ax], iointAnglePos[ax], ax);
	}

	if(!DeltaCalcPosForward(delta_mech_pars, iointAnglePos, pMechPos))
		return ROCKS_ERR_DELTA_JOINT_POS_ERROR;

	return NYCE_OK; 
}

NYCE_STATUS RocksKinDeltaPosition(ROCKS_MECH* pMech, double pPos[])
{
	double pJointPos[ROCKS_MECH_MAX_DOF];
	NYCE_STATUS status = NYCE_OK;
	for (uint32_t ax = 0; ax < pMech->nrOfJoints; ax++)
	{
		status = NyceError(status) ? status : SacReadVariable(pMech->jointAxisId[ax], SAC_VAR_SETPOINT_POS, &pJointPos[ax]);
	}
	status = NyceError(status) ? status : RocksKinForwardDelta(pMech, pJointPos, pPos);	
	return status;
}

NYCE_STATUS RocksKinInverseDelta(ROCKS_MECH* pMech, const ROCKS_KIN_INV_PARS* pKin)
{
	if (delta_mech_pars.e <= 0 || delta_mech_pars.f <= 0 || delta_mech_pars.re <= 0 || delta_mech_pars.rf <= 0 )
		return ROCKS_ERR_DELTA_PARS_ERROR;

	NYCE_STATUS status = RocksCheckRate();
	if (NyceError(status))
		return status;

	uint32_t ax = 0;
	if (pMech->var.jointBuffersAllocated)
	{
		int i;
		for (i = 0; i < ROCKS_MECH_MAX_NR_OF_JOINTS; i++)
		{
			if (pMech->var.pJointPositionBufferC[i] != NULL)
			{
				free(pMech->var.pJointPositionBufferC[i]);
				pMech->var.pJointPositionBufferC[i] = NULL;
			}
			if (pMech->var.pJointVelocityBufferC[i] != NULL)
			{
				free(pMech->var.pJointVelocityBufferC[i]);
				pMech->var.pJointVelocityBufferC[i] = NULL;
			}
		}
		pMech->var.jointBuffersAllocated = FALSE;
	}

	while (!pMech->var.jointBuffersAllocated)
	{
		pMech->var.pJointPositionBufferC[ax] = (double*)malloc(pMech->var.maxNrOfSplines * sizeof(double));
		pMech->var.pJointVelocityBufferC[ax] = (double*)malloc(pMech->var.maxNrOfSplines * sizeof(double));
		if(++ax == ROCKS_MECH_MAX_NR_OF_JOINTS) 
		{
			pMech->var.jointBuffersAllocated = TRUE;
			pMech->var.pApplyForwardKinFunc = RocksKinForwardDelta;
			pMech->var.pApplyInverseKinFunc = RocksKinInverseDelta;
			pMech->var.pGetWorldSetpointPosFunc = RocksKinDeltaPosition;
		}
	}

	uint32_t realSegNum = 0;
	double (*pPosition)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF];
	double (*pVelocity)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF];

	for (uint32_t index = 0; index < pMech->var.usedNrOfSplines; ++index)
	{
		ConvertPathToWorldCoordinate(pMech, index, pPosition[realSegNum], pVelocity[realSegNum]);
		realSegNum++;
	}

	pMech->var.usedNrOfSplines = realSegNum - 1;

	double jointAnglePos[ROCKS_MECH_MAX_NR_OF_JOINTS];
	double jointAngleVel[ROCKS_MECH_MAX_NR_OF_JOINTS];
	for(uint32_t index = 0; index < realSegNum; ++index)
	{
		if(!DeltaCalcPosInverse(delta_mech_pars, pPosition[index], jointAnglePos))
			return ROCKS_ERR_DELTA_TAJ_OVER_WORKSPACE;
	
		if(!DeltaCalcVelInverse(delta_mech_pars, pPosition[index], pVelocity[index], jointAnglePos, jointAngleVel))
			return ROCKS_ERR_DELTA_TAJ_VEL_ERROR;

		for (uint32_t ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ax++)
		{
			ConvertAngleToPU(jointAnglePos[ax], pMech->var.pJointPositionBufferC[ax][index], ax);
			ConvertAngleToPU(jointAngleVel[ax], pMech->var.pJointVelocityBufferC[ax][index], ax);
		}
	}

	if (WaitForSingleObject(evExportDatas, 0) == WAIT_OBJECT_0 )
	{
		ofstream file("..//xyz&jointDatas.txt");	
		file<<pMech->var.startPos[0]<< " "<<pMech->var.startPos[1]<<" "<<pMech->var.startPos[2]<<endl;
		file<<"|Index|x_pos|y_pos|z_pos|x_vel|y_vel|z_vel|joint1_pos|joint2_pos|joint3_pos|joint1_vel|joint2_vel|joint3_vel|"<<endl<<"|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|"<<endl;
		for (uint32_t i = 0; i < realSegNum; ++i)
		{
			file<<"|"<<i<<"|"<<pPosition[i][0]<<"|"<<pPosition[i][1]<<"|"<<pPosition[i][2]<<"|"<<pVelocity[i][0]<<"|"<<pVelocity[i][1]<<"|"<<pVelocity[i][2]<<"|"<<pMech->var.pJointPositionBufferC[0][i]<<"|"<<pMech->var.pJointPositionBufferC[1][i]<<"|"<<pMech->var.pJointPositionBufferC[2][i]<<"|"<<pMech->var.pJointVelocityBufferC[0][i]<<"|"<<pMech->var.pJointVelocityBufferC[1][i]<<"|"<<pMech->var.pJointVelocityBufferC[2][i]<<"|"<<endl;
		}
		file.close();
	}

	delete []pPosition;
	delete []pVelocity;

	pMech->var.mechStep = ROCKS_MECH_STEP_VALID_INV_KINEMATICS;
	return NYCE_OK;
}

NYCE_STATUS RocksExExportSplineDatas(const BOOL &signal)
{
	if (signal && WaitForSingleObject(evExportDatas, 0) != WAIT_OBJECT_0)
		SetEvent(evExportDatas);
	if (!signal && WaitForSingleObject(evExportDatas, 0) == WAIT_OBJECT_0)
		ResetEvent(evExportDatas);
	return NYCE_OK;
}
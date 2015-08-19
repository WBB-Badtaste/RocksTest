#pragma once

#include <rocksapi.h>
#include <math.h>
#include "NyceExDefs.h"
#include "SpaceAlgorithm.h"
#include "SpiralTrajAlgorithm.h"
#include "RocksSpiralBufferManager.h"

/**
 *  @brief  Spiral definition of a segment profile.
 */
typedef struct rocks_traj_segment_spiral_pars
{
    double      endPos[2];                      /**< End position in 2 world coordinates (XY XZ or YZ) */
    double      center[2];                      /**< Center of the arc (XY XZ or YZ) */
    double      endVelocity;                    /**< End velocity */
    double      maxAcceleration;                /**< Path acceleration constraint */
    ROCKS_PLANE plane;                          /**< Plane of the segment */
    ROCKS_POSE  originOffset;                   /**< Reference frame rotation offsets to be added before the arc */
} ROCKS_TRAJ_SEGMENT_SPIRAL_PARS;

typedef struct rocks_traj_segment_spiral_pars_ex
{
	double      endPos[2];                      /**< End position in 2 world coordinates (XY XZ or YZ) */
	double      center[2];                      /**< Center of the arc (XY XZ or YZ) */
	double      endAngleVelocity;               /**< End angle velocity */
	double      maxAngleAcceleration;           /**< Path angle acceleration constraint */
	double		maxRadialVelocity;				/**< Velocity of radial direction constraint */
	double      maxRadialAcceleration;			/**< Acceleration of radial direction constraint */
	ROCKS_PLANE plane;                          /**< Plane of the segment */
	ROCKS_POSE  originOffset;                   /**< Reference frame rotation offsets to be added before the arc */
} ROCKS_TRAJ_SEGMENT_SPIRAL_PARS_EX;

NYCE_STATUS RocksTrajSegmentSpiral(ROCKS_MECH *pMech, const ROCKS_TRAJ_SEGMENT_SPIRAL_PARS *pTraj)
{
	double startPos[2];
	double moveSignal(0.0);
	switch (pTraj->plane)
	{
	case ROCKS_PLANE_XY:
		startPos[0] = pMech->var.lastSegmentEndPos[0];
		startPos[1] = pMech->var.lastSegmentEndPos[1];
		pMech->var.lastSegmentEndPos[0] = pTraj->endPos[0];
		pMech->var.lastSegmentEndPos[1] = pTraj->endPos[1];
		Roll(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.x);
		Pitch(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.y);
		Yaw(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.z);
		moveSignal = ROCKS_PLANE_XY * 256 + ROCKS_MOVE_TYPE_SPIRAL;
		break;
	case ROCKS_PLANE_YZ:
		startPos[0] = pMech->var.lastSegmentEndPos[1];
		startPos[1] = pMech->var.lastSegmentEndPos[2];
		pMech->var.lastSegmentEndPos[1] = pTraj->endPos[0];
		pMech->var.lastSegmentEndPos[2] = pTraj->endPos[1];
		Roll(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.x);
		Pitch(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.y);
		Yaw(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.z);
		moveSignal = ROCKS_PLANE_YZ * 256 + ROCKS_MOVE_TYPE_SPIRAL;
		break;
	case ROCKS_PLANE_ZX:
		startPos[0] = pMech->var.lastSegmentEndPos[0];
		startPos[1] = pMech->var.lastSegmentEndPos[2];
		pMech->var.lastSegmentEndPos[0] = pTraj->endPos[0];
		pMech->var.lastSegmentEndPos[2] = pTraj->endPos[1];
		Roll(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.x);
		Pitch(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.y);
		Yaw(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.z);
		moveSignal = ROCKS_PLANE_ZX * 256 + ROCKS_MOVE_TYPE_SPIRAL;
		break;
	default:
		break;
	}

	//r=a+b*theta
	double radius[2]={0.0}, a(0.0), b(0.0);
	CalcArchimedeSpiralPars(startPos, pTraj->endPos, pTraj->center, radius, a, b);

	//calc arc len
	double arcLen(CalcArchimedeSpiralArcLen(radius, a, b));

	//判断是否超出加速度上限
	double time(arcLen * 2.0 / (pMech->var.lastSegmentEndVel + pTraj->endVelocity));
	double acc((pTraj->endVelocity - pMech->var.lastSegmentEndVel) / time);
	if ( (acc > 0 && acc > pTraj->maxAcceleration) || (acc < 0 && -acc > pTraj->maxAcceleration))
	{
		pMech->var.mechStep = ROCKS_MECH_STEP_INITIAL;
		return ROCKS_ERR_MAX_ACCELERATION_EXCEEDED;
	}

	//buffer manage
	uint32_t splineNum((uint32_t)(time / pMech->var.splineTime) + 1);
	uint32_t bufferEnd(pMech->var.usedNrOfSplines + splineNum + 7);
	if ( bufferEnd > pMech->var.maxNrOfSplines)
	{
		pMech->var.maxNrOfSplines = bufferEnd + 512;

		double *pPosBuffer = (double *)malloc(pMech->var.maxNrOfSplines * sizeof(double));
		double *pVelBuffer = (double *)malloc(pMech->var.maxNrOfSplines * sizeof(double));
		ZeroMemory(pPosBuffer, pMech->var.maxNrOfSplines * sizeof(double));
		ZeroMemory(pVelBuffer, pMech->var.maxNrOfSplines * sizeof(double));

		memcpy(pPosBuffer, pMech->var.pPositionSplineBuffer, pMech->var.usedNrOfSplines);
		memcpy(pVelBuffer, pMech->var.pVelocitySplineBuffer, pMech->var.usedNrOfSplines);

		free(pMech->var.pPositionSplineBuffer);
		free(pMech->var.pVelocitySplineBuffer);
		pMech->var.pPositionSplineBuffer = pPosBuffer;
		pMech->var.pVelocitySplineBuffer = pVelBuffer;
	}

	//segment heard
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 0] = pMech->var.lastSegmentEndPos[0];		pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 0] = pMech->var.lastSegmentEndPos[1];
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 1] = pMech->var.lastSegmentEndPos[2];		pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 1] = pTraj->originOffset.r.x;
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 2] = pTraj->originOffset.r.y;				pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 2] = pTraj->originOffset.r.z;
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 3] = splineNum;							pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 3] = moveSignal;
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 4] = startPos[0];							pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 4] = startPos[1];
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 5] = pTraj->endPos[0];					pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 5] = pTraj->endPos[1];
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 6] = pTraj->center[0];					pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 6] = pTraj->center[1];

	//calc segment
	pMech->var.usedNrOfSplines += 7;
	for (uint32_t index(0); index < splineNum; ++index, ++pMech->var.usedNrOfSplines)
	{
		if (pMech->var.usedNrOfSplines == bufferEnd - 1)
		{
			pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines] = pTraj->endVelocity;
			pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines] = arcLen;
		}
		double segTime(pMech->var.splineTime * index);
		pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines] = pMech->var.lastSegmentEndVel + acc * segTime;
		pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines] = (pMech->var.lastSegmentEndVel + pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines]) / 2.0 / segTime;
	}

	//调整mech结构体
	pMech->var.lastSegmentEndVel = pTraj->endVelocity;
	pMech->var.lastSplineTime = 0;

	return NYCE_OK;
}

NYCE_STATUS RocksTrajSegmentSpiral(ROCKS_MECH *pMech, const ROCKS_TRAJ_SEGMENT_SPIRAL_PARS_EX *pTraj)
{
	double startPos[2];
	double moveSignal(0.0);
	switch (pTraj->plane)
	{
	case ROCKS_PLANE_XY:
		startPos[0] = pMech->var.lastSegmentEndPos[0];
		startPos[1] = pMech->var.lastSegmentEndPos[1];
		pMech->var.lastSegmentEndPos[0] = pTraj->endPos[0];
		pMech->var.lastSegmentEndPos[1] = pTraj->endPos[1];
		Roll(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.x);
		Pitch(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.y);
		Yaw(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.z);
		moveSignal = ROCKS_PLANE_XY * 256 + ROCKS_MOVE_TYPE_SPIRAL_EX;
		break;
	case ROCKS_PLANE_YZ:
		startPos[0] = pMech->var.lastSegmentEndPos[1];
		startPos[1] = pMech->var.lastSegmentEndPos[2];
		pMech->var.lastSegmentEndPos[1] = pTraj->endPos[0];
		pMech->var.lastSegmentEndPos[2] = pTraj->endPos[1];
		Roll(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.x);
		Pitch(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.y);
		Yaw(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.z);
		moveSignal = ROCKS_PLANE_YZ * 256 + ROCKS_MOVE_TYPE_SPIRAL_EX;
		break;
	case ROCKS_PLANE_ZX:
		startPos[0] = pMech->var.lastSegmentEndPos[0];
		startPos[1] = pMech->var.lastSegmentEndPos[2];
		pMech->var.lastSegmentEndPos[0] = pTraj->endPos[0];
		pMech->var.lastSegmentEndPos[2] = pTraj->endPos[1];
		Roll(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.x);
		Pitch(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.y);
		Yaw(pMech->var.lastSegmentEndPos, pTraj->center[0], pTraj->center[1], pTraj->originOffset.r.z);
		moveSignal = ROCKS_PLANE_ZX * 256 + ROCKS_MOVE_TYPE_SPIRAL_EX;
		break;
	default:
		break;
	}

	//prepare
	const double offset1(startPos[0] - pTraj->center[0]);
	const double offset2(startPos[1] - pTraj->center[1]);
	const double offset3(pTraj->endPos[0] - pTraj->center[0]);
	const double offset4(pTraj->endPos[1] - pTraj->center[1]);
	const double startRadius(sqrt(offset1 * offset1 + offset2 * offset2));
	const double endRadius(sqrt(offset3 * offset3 + offset4 * offset4));
	double startAngle(atan2(offset2, offset1));
	if (startAngle < 0)
		startAngle += M_PI * 2.0;
	double endAngle(atan2(offset4, offset3));
	if (endAngle < 0)
		endAngle += M_PI * 2.0;
	double totalAngle((endAngle - startAngle));
	if (totalAngle > M_PI)
		totalAngle = M_PI * 2.0 - totalAngle;
	if (totalAngle < -M_PI)
		totalAngle = -M_PI * 2.0 - totalAngle;

	//不通用的定义，修改
	const double startAngleVel(totalAngle >= 0 ? pMech->var.lastSegmentEndVel/ startRadius : -pMech->var.lastSegmentEndVel/ startRadius);
	double endAngleVel(totalAngle >= 0 ? pTraj->endAngleVelocity : -pTraj->endAngleVelocity);
	//不通用的定义，修改


	//估算时间
	double totalTime(totalAngle * 2.0 / (endAngleVel +  startAngleVel));

	//估算段数
	const double dSplineNum(totalTime / pMech->var.splineTime); 
	uint32_t splineNum((uint32_t)dSplineNum);

	//修正时间，段数
	if (endAngleVel != 0.0)//末速度不为0时修正
	{
		if (dSplineNum - (double)splineNum > 0.5)
			splineNum++;
		totalTime = splineNum * pMech->var.splineTime;
	}

	//加速到达到最大的奇点时间
	const double singularTime_angleAccMax(totalTime * 0.5);

	//加加速度
	const double angleJerk1((totalAngle - 2.0 * startAngleVel * singularTime_angleAccMax) / (singularTime_angleAccMax * singularTime_angleAccMax * singularTime_angleAccMax));// 肯定与转角同向
	const double angleJerk2(-angleJerk1);

	//奇点数据
	const double singularAngleAcc_angleAccMax(angleJerk1 * singularTime_angleAccMax );
	const double singularAngleVel_angleAccMax(startAngleVel + 0.5 * angleJerk1 * singularTime_angleAccMax * singularTime_angleAccMax);
	const double singularAngle_angleAccMax(startAngle + startAngleVel * singularTime_angleAccMax + angleJerk1 * singularTime_angleAccMax * singularTime_angleAccMax * singularTime_angleAccMax / 6.0);

	//修正末速度
	endAngleVel = startAngleVel + angleJerk1 * singularTime_angleAccMax * singularTime_angleAccMax;
	//endAngleVel = singularAngleVel_angleAccMax + singularAngle_angleAccMax * singularTime_angleAccMax + 0.5 * angleJerk2 * singularTime_angleAccMax * singularTime_angleAccMax;

	//判断最大加速度
	if ( singularAngleAcc_angleAccMax > pTraj->maxAngleAcceleration || singularAngleAcc_angleAccMax < -pTraj->maxAngleAcceleration)
	{
		pMech->var.mechStep = ROCKS_MECH_STEP_INITIAL;
		return ROCKS_ERR_MAX_ANGLE_ACCELERATION_EXCEEDED;
	}

	//径向部分计算
	const double totalRadialDistance(endRadius - startRadius);
	const double abMaxRadialVel(totalRadialDistance * 2.0 / totalTime);
	if ( abMaxRadialVel > pTraj->maxRadialVelocity || abMaxRadialVel < -pTraj->maxRadialVelocity)
	{
		pMech->var.mechStep = ROCKS_MECH_STEP_INITIAL;
		return ROCKS_ERR_MAX_RADIAL_VELOCITY_EXCEEDED;
	}
	double abRadialAcc(abMaxRadialVel * 2.0 / totalTime);
	if ( abRadialAcc > pTraj->maxRadialAcceleration || abRadialAcc < -pTraj->maxRadialAcceleration)
	{
		pMech->var.mechStep = ROCKS_MECH_STEP_INITIAL;
		return ROCKS_ERR_MAX_RADIAL_ACCELERATION_EXCEEDED;
	}
	
	//buffer manage
	uint32_t nextSegBuffer(pMech->var.usedNrOfSplines + splineNum + 7);
	SpiralBufferManage(pMech, nextSegBuffer);

	//segment heard
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 0] = pMech->var.lastSegmentEndPos[0];		pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 0] = pMech->var.lastSegmentEndPos[1];
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 1] = pMech->var.lastSegmentEndPos[2];		pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 1] = pTraj->originOffset.r.x;
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 2] = pTraj->originOffset.r.y;				pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 2] = pTraj->originOffset.r.z;
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 3] = splineNum;							pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 3] = moveSignal;
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 4] = startPos[0];							pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 4] = startPos[1];
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 5] = pTraj->endPos[0];					pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 5] = pTraj->endPos[1];
	pMech->var.pVelocitySplineBuffer[pMech->var.usedNrOfSplines + 6] = pTraj->center[0];					pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines + 6] = pTraj->center[1];

	//calc segment
	pMech->var.usedNrOfSplines += 7;

	const double transitionalRadius(startRadius + 0.125 * abRadialAcc * totalTime * totalTime);
	const double transitionalRadialVel(0.5 * abRadialAcc * totalTime);

	for (uint32_t index(0); index < splineNum; ++index, ++pMech->var.usedNrOfSplines)
	{
		double segTime(0.0);
		if (pMech->var.usedNrOfSplines == nextSegBuffer - 1)
			segTime = totalTime;
		else
			segTime =  pMech->var.splineTime * (index + 1);

		double currentAngle(0.0);
		double currentAngleVel(0.0);
		if (segTime <= singularTime_angleAccMax)
		{
			currentAngle = startAngle + startAngleVel * segTime + angleJerk1 * segTime * segTime * segTime / 6.0;
			currentAngleVel = startAngleVel + 0.5 * angleJerk1 * segTime * segTime;
		}
		else
		{
			double relativeTime(segTime - singularTime_angleAccMax);
			currentAngle = singularAngle_angleAccMax + singularAngleVel_angleAccMax * relativeTime + 0.5 * singularAngleAcc_angleAccMax * relativeTime * relativeTime + angleJerk2 * relativeTime * relativeTime * relativeTime / 6.0;
			currentAngleVel = singularAngleVel_angleAccMax + singularAngleAcc_angleAccMax * relativeTime + 0.5 * angleJerk2 * relativeTime * relativeTime;
		}

		double currentRadius(0.0);
		double currentRadialVel(0.0);
		if (segTime > totalTime * 0.5)
		{
			double accTime(segTime - totalTime * 0.5);
			currentRadius = transitionalRadius + transitionalRadialVel * accTime  - 0.5 * abRadialAcc * accTime * accTime;
			currentRadialVel = transitionalRadialVel - abRadialAcc * accTime;
		}
		else
		{
			currentRadius = startRadius + 0.5 * abRadialAcc * segTime * segTime;
			currentRadialVel = abRadialAcc * segTime;
		}

		switch (pTraj->plane)
		{
		case ROCKS_PLANE_XY:
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][0] = pTraj->center[0] + currentRadius * cos(currentAngle);
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][1] = pTraj->center[1] + currentRadius * sin(currentAngle);
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][2] = pMech->var.lastSegmentEndPos[2];

			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][0] = -currentAngleVel * currentRadius * sin(currentAngle) + currentRadialVel * cos(currentAngle);
			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][1] = currentAngleVel * currentRadius * cos(currentAngle) + currentRadialVel * sin(currentAngle);
			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][2] = 0;
			break;										
		case ROCKS_PLANE_YZ:							
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][0] = pMech->var.lastSegmentEndPos[0];
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][1] = pTraj->center[0] + currentRadius * cos(currentAngle);
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][2] = pTraj->center[1] + currentRadius * sin(currentAngle);

			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][0] = 0;
			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][1] = -currentAngleVel * currentRadius * sin(currentAngle) + currentRadialVel * cos(currentAngle);
			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][2] = currentAngleVel * currentRadius * cos(currentAngle) + currentRadialVel * sin(currentAngle);
			break;										
		case ROCKS_PLANE_ZX:							
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][0] = pTraj->center[0] + currentRadius * cos(currentAngle);
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][1] = pMech->var.lastSegmentEndPos[1];
			pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][2] = pTraj->center[1] + currentRadius * sin(currentAngle);

			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][0] = -currentAngleVel * currentRadius * sin(currentAngle) + currentRadialVel * cos(currentAngle);
			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][1] = 0;
			pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines][2] = currentAngleVel * currentRadius * cos(currentAngle) + currentRadialVel * sin(currentAngle);
			break;
		default:
			break;
		}
		
		if (pMech->var.refFramePose2.r.x)
		{
			Roll(pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.startPos, pMech->var.refFramePose2.r.x);
			Roll(pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.refFramePose2.r.x);
		}

		if (pMech->var.refFramePose2.r.y)
		{
			Pitch(pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.startPos, pMech->var.refFramePose2.r.y);
			Pitch(pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.refFramePose2.r.y);
		}

		if (pMech->var.refFramePose2.r.z)
		{
			Yaw(pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.startPos, pMech->var.refFramePose2.r.z);
			Yaw(pVelSpiralSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.refFramePose2.r.z);
		}

		pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][0] += pMech->var.refFramePose2.t.x;
		pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][1] += pMech->var.refFramePose2.t.y;
		pPosSpiralSplineBuffer[pMech->var.usedNrOfSplines][2] += pMech->var.refFramePose2.t.z;
	}

	//调整mech结构体
	pMech->var.lastSegmentEndVel = endAngleVel * endRadius;
	if (pMech->var.lastSegmentEndVel < 0)
		pMech->var.lastSegmentEndVel = -pMech->var.lastSegmentEndVel;
	if (pMech->var.lastSegmentEndVel == 0)
		pMech->var.lastSplineTime = totalTime - (splineNum - 1) * pMech->var.splineTime;
	else
		pMech->var.lastSplineTime = -1.0;
	return NYCE_OK;
}
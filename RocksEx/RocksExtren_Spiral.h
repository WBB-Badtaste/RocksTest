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

	//prepare
	double offset1(startPos[0] - pTraj->center[0]);
	double offset2(startPos[1] - pTraj->center[1]);
	double offset3(pTraj->endPos[0] - pTraj->center[0]);
	double offset4(pTraj->endPos[1] - pTraj->center[1]);

	//calc angle pars
	double startAngle(atan2(offset2, offset1));
	double endtAngle(atan2(offset4, offset3));
	double totalAngle(endtAngle - startAngle);
	double time(totalAngle * 2.0 / (pTraj->endAngleVelocity + pMech->var.lastSegmentEndVel));
	double angleAcc((pTraj->endAngleVelocity - pMech->var.lastSegmentEndVel) / time);
	if ( (angleAcc > 0 && angleAcc > pTraj->maxAngleAcceleration) || (angleAcc < 0 && - angleAcc > pTraj->maxAngleAcceleration))
	{
		pMech->var.mechStep = ROCKS_MECH_STEP_INITIAL;
		return ROCKS_ERR_MAX_ANGLE_ACCELERATION_EXCEEDED;
	}

	//calc radial pars
	double startRadius(sqrt(offset1 * offset1 + offset2 * offset2));
	double endRadius(sqrt(offset3 * offset3 + offset4 * offset4));
	double totalRadialDistance(endRadius - startRadius);
	double abMaxRadialVel(totalRadialDistance * 2.0 / time);
	if ( (abMaxRadialVel > 0 && abMaxRadialVel > pTraj->maxRadialVelocity) || (abMaxRadialVel < 0 && - abMaxRadialVel > pTraj->maxRadialVelocity))
	{
		pMech->var.mechStep = ROCKS_MECH_STEP_INITIAL;
		return ROCKS_ERR_MAX_RADIAL_VELOCITY_EXCEEDED;
	}
	double abRadialAcc(abMaxRadialVel * 2.0 / time);
	if ( (abRadialAcc > 0 && abRadialAcc > pTraj->maxRadialAcceleration) || (abRadialAcc < 0 && - abRadialAcc > pTraj->maxRadialAcceleration))
	{
		pMech->var.mechStep = ROCKS_MECH_STEP_INITIAL;
		return ROCKS_ERR_MAX_RADIAL_ACCELERATION_EXCEEDED;
	}

	//buffer manage
	uint32_t splineNum((uint32_t)(time / pMech->var.splineTime) + 1);
	uint32_t bufferEnd(pMech->var.usedNrOfSplines + splineNum + 7);
	SpiralBufferManage(pMech, bufferEnd);

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

	double endVel(0.0);
	if (totalAngle <= 0)
		endVel = pTraj->endAngleVelocity * endRadius;
	else
		endVel = -pTraj->endAngleVelocity * endRadius;

	for (uint32_t index(0); index < splineNum; ++index, ++pMech->var.usedNrOfSplines)
	{
		if (pMech->var.usedNrOfSplines == bufferEnd - 1)
		{
			switch (pTraj->plane)
			{
			case ROCKS_PLANE_XY:
				pPosSplineBuffer[pMech->var.usedNrOfSplines][0] = pTraj->center[0] + endRadius * cos(endtAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][1] = pTraj->center[1] + endRadius * sin(endtAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][2] = pMech->var.lastSegmentEndPos[2];
															
				pVelSplineBuffer[pMech->var.usedNrOfSplines][0] = endVel * sin(endtAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][1] = -endVel * cos(endtAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][2] = 0;
				break;
			case ROCKS_PLANE_YZ:
				pPosSplineBuffer[pMech->var.usedNrOfSplines][0] = pMech->var.lastSegmentEndPos[0];
				pPosSplineBuffer[pMech->var.usedNrOfSplines][1] = pTraj->center[0] + endRadius * cos(endtAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][2] = pTraj->center[1] + endRadius * sin(endtAngle);
															
				pVelSplineBuffer[pMech->var.usedNrOfSplines][0] = 0;
				pVelSplineBuffer[pMech->var.usedNrOfSplines][1] = endVel * sin(endtAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][2] = -endVel * cos(endtAngle);
				break;
			case ROCKS_PLANE_ZX:
				pPosSplineBuffer[pMech->var.usedNrOfSplines][0] = pTraj->center[0] + endRadius * cos(endtAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][1] = pMech->var.lastSegmentEndPos[1];
				pPosSplineBuffer[pMech->var.usedNrOfSplines][2] = pTraj->center[1] + endRadius * sin(endtAngle);
			
				pVelSplineBuffer[pMech->var.usedNrOfSplines][0] = endVel * sin(endtAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][1] = 0;
				pVelSplineBuffer[pMech->var.usedNrOfSplines][2] = -endVel * cos(endtAngle);
				break;
			default:
				break;
			}
		}
		else
		{
			//calc segments
			double segTime(pMech->var.splineTime * index);
			double startAngleVel(pMech->var.lastSegmentEndVel / startRadius);
			double currentAngle(startAngle + startAngleVel * segTime + 0.5 * angleAcc * segTime * segTime);
			double currentAngleVel(angleAcc * segTime);
			double transitionalRadius(startRadius + 0.125 * abRadialAcc * time * time);
			double transitionalRadialVel(0.5 * abRadialAcc * time);
			double currentRadius(0.0);
			double currentRadialVel(0.0);
			if (segTime > time * 0.5)
			{
				currentRadius = startRadius + 0.5 * abRadialAcc * segTime * segTime;
				currentRadialVel = abRadialAcc * segTime;
			}
			else
			{
				double accTime(segTime - time * 0.5);
				currentRadius = transitionalRadius + transitionalRadialVel * accTime  - 0.5 * abRadialAcc * accTime * accTime;
				currentRadialVel = transitionalRadialVel - abRadialAcc * accTime;
			}


			switch (pTraj->plane)
			{
			case ROCKS_PLANE_XY:
				pPosSplineBuffer[pMech->var.usedNrOfSplines][0] = pTraj->center[0] + currentRadius * cos(currentAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][1] = pTraj->center[1] + currentRadius * sin(currentAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][2] = pMech->var.lastSegmentEndPos[2];
															
				pVelSplineBuffer[pMech->var.usedNrOfSplines][0] = currentAngleVel * sin(currentAngle) + currentRadialVel * cos(currentAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][1] = -currentAngleVel * cos(currentAngle) + currentRadialVel * sin(currentAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][2] = 0;
				break;										
			case ROCKS_PLANE_YZ:							
				pPosSplineBuffer[pMech->var.usedNrOfSplines][0] = pMech->var.lastSegmentEndPos[0];
				pPosSplineBuffer[pMech->var.usedNrOfSplines][1] = pTraj->center[0] + currentRadius * cos(currentAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][2] = pTraj->center[1] + currentRadius * sin(currentAngle);
															
				pVelSplineBuffer[pMech->var.usedNrOfSplines][0] = 0;
				pVelSplineBuffer[pMech->var.usedNrOfSplines][1] = currentAngleVel * sin(currentAngle) + currentRadialVel * cos(currentAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][2] = -currentAngleVel * cos(currentAngle) + currentRadialVel * sin(currentAngle);
				break;										
			case ROCKS_PLANE_ZX:							
				pPosSplineBuffer[pMech->var.usedNrOfSplines][0] = pTraj->center[0] + currentRadius * cos(currentAngle);
				pPosSplineBuffer[pMech->var.usedNrOfSplines][1] = pMech->var.lastSegmentEndPos[1];
				pPosSplineBuffer[pMech->var.usedNrOfSplines][2] = pTraj->center[1] + currentRadius * sin(currentAngle);
															
				pVelSplineBuffer[pMech->var.usedNrOfSplines][0] = currentAngleVel * sin(currentAngle) + currentRadialVel * cos(currentAngle);
				pVelSplineBuffer[pMech->var.usedNrOfSplines][1] = 0;
				pVelSplineBuffer[pMech->var.usedNrOfSplines][2] = -currentAngleVel * cos(currentAngle) + currentRadialVel * sin(currentAngle);
				break;
			default:
				break;
			}
		}
		
		if (pMech->var.refFramePose2.r.x)
		{
			Roll(pPosSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.startPos, pMech->var.refFramePose2.r.x);
			Roll(pVelSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.refFramePose2.r.x);
		}

		if (pMech->var.refFramePose2.r.y)
		{
			Pitch(pPosSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.startPos, pMech->var.refFramePose2.r.y);
			Pitch(pVelSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.refFramePose2.r.y);
		}

		if (pMech->var.refFramePose2.r.z)
		{
			Yaw(pPosSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.startPos, pMech->var.refFramePose2.r.z);
			Yaw(pVelSplineBuffer[pMech->var.usedNrOfSplines], pMech->var.refFramePose2.r.z);
		}

		pPosSplineBuffer[pMech->var.usedNrOfSplines][0] += pMech->var.refFramePose2.t.x;
		pPosSplineBuffer[pMech->var.usedNrOfSplines][1] += pMech->var.refFramePose2.t.y;
		pPosSplineBuffer[pMech->var.usedNrOfSplines][2] += pMech->var.refFramePose2.t.z;
	}

	//调整mech结构体
	pMech->var.lastSegmentEndVel = endVel;
	pMech->var.lastSplineTime = 0;
	return NYCE_OK;
}
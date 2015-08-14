#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "SpaceAlgorithm.h"

#include <rocksapi.h>

uint32_t mix_Boundary = 0;
ROCKS_PLANE mix_plane = ROCKS_PLANE_XY;
ROCKS_POSE mix_pose;
uint32_t mix_moveType = ROCKS_MOVE_TYPE_LINEAR;
double mix_startPos1, mix_endPos1, mix_startPos2, mix_endPos2, mix_center1, mix_center2, mix_angle, mix_endPos_x, mix_endPos_y, mix_endPos_z;

void ConvertCriclePath(const ROCKS_PLANE &plane, const ROCKS_POSE &pose, const double &startPos1, const double &startPos2, const double &center1, const double &center2, const double &angle, const  double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	double radius = sqrt((center1 - startPos1) * (center1 - startPos1) + (center2 - startPos2) * (center2 - startPos2));
	double beta(0.0);
	CalcRotateAngle(beta, center1, center2, startPos1, startPos2);

	double absoluteAngle(0.0), absoluteVelocity(0.0);
	if (angle>0)
	{
		absoluteAngle = beta - CurrentDistance / radius;
		absoluteVelocity = CurrentVelocity;
	}
	else
	{
		absoluteAngle = beta + CurrentDistance / radius;
		absoluteVelocity = -CurrentVelocity;

	}

	switch(plane)
	{
	case ROCKS_PLANE_XY:
		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[1] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[1] = -absoluteVelocity * cos(absoluteAngle);
		break;
	case ROCKS_PLANE_YZ:
		pPosition[1] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[1] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;
	case ROCKS_PLANE_ZX:
		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;
	default:
		break;
	}

	if (pose.r.x)
	{
		Roll(pPosition, startPos1, startPos2, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, startPos1, startPos2, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, startPos1, startPos2, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

void ConvertCriclePath(const double *const pStartPos, const double &totalAngle, const double &CurrentDistance, const double &CurrentVelocity, const ROCKS_PLANE &plane, const double &radius, const double *const pCenter, double *const pPosition, double *const pVelocity)
{
	double angle(0.0);
	double beta(0.0);
	switch(plane)
	{
	case ROCKS_PLANE_XY:
		CalcRotateAngle(beta, pCenter[0], pCenter[1], pStartPos[0], pStartPos[1]);

		if (totalAngle > 0)
		{
			angle = beta - CurrentDistance / radius;

			pVelocity[0] =  CurrentVelocity * sin(angle);
			pVelocity[1] = -CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}
		else
		{
			angle = beta + CurrentDistance / radius;

			pVelocity[0] = -CurrentVelocity * sin(angle);
			pVelocity[1] =  CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}

		pPosition[0] = pCenter[0] + radius * cos(angle);
		pPosition[1] = pCenter[1] + radius * sin(angle);
		pPosition[2] = pStartPos[2];
		break;
	case ROCKS_PLANE_YZ:
		CalcRotateAngle(beta, pCenter[0], pCenter[1], pStartPos[1], pStartPos[2]);

		if (totalAngle > 0)
		{
			angle = beta - CurrentDistance / radius;

			pVelocity[0] =  0;
			pVelocity[1] =  CurrentVelocity * sin(angle);
			pVelocity[2] = -CurrentVelocity * cos(angle);
		}
		else
		{
			angle = beta + CurrentDistance / radius;

			pVelocity[0] =  0;
			pVelocity[1] = -CurrentVelocity * sin(angle);
			pVelocity[2] =  CurrentVelocity * cos(angle);
		}

		pPosition[0] = pStartPos[0];
		pPosition[1] = pCenter[0] + radius * cos(angle);
		pPosition[2] = pCenter[1] + radius * sin(angle);
		break;
	case ROCKS_PLANE_ZX:
		CalcRotateAngle(beta, pCenter[0], pCenter[1], pStartPos[2], pStartPos[0]);

		if (totalAngle > 0)
		{
			angle = beta - CurrentDistance / radius;

			pVelocity[0] = -CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] =  CurrentVelocity * sin(angle);
		}
		else
		{
			angle = beta + CurrentDistance / radius;

			pVelocity[0] =  CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] = -CurrentVelocity * sin(angle);
		}

		pPosition[0] = pCenter[1] + radius * sin(angle);
		pPosition[1] = pStartPos[1];
		pPosition[2] = pCenter[0] + radius * cos(angle);
		break;
	}

}

void ConverSpiralPath(const ROCKS_PLANE &plane, const ROCKS_POSE &pose, const double &startPos1, const double &startPos2, const double &endPos1, const double &endPos2, const double &center1, const double &center2, const double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	double radius = sqrt((center1 - startPos1) * (center1 - startPos1) + (center2 - startPos2) * (center2 - startPos2));
	double beta(0.0);
	CalcRotateAngle(beta, center1, center2, startPos1, startPos2);

	double absoluteAngle(0.0), absoluteVelocity(0.0);
	if (angle>0)
	{
		absoluteAngle = beta - CurrentDistance / radius;
		absoluteVelocity = CurrentVelocity;
	}
	else
	{
		absoluteAngle = beta + CurrentDistance / radius;
		absoluteVelocity = -CurrentVelocity;
	}

	switch(plane)
	{
	case ROCKS_PLANE_XY:
		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[1] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[1] = -absoluteVelocity * cos(absoluteAngle);
		break;
	case ROCKS_PLANE_YZ:
		pPosition[1] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[1] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;
	case ROCKS_PLANE_ZX:
		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;
	default:
		break;
	}

	if (pose.r.x)
	{
		Roll(pPosition, startPos1, startPos2, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, startPos1, startPos2, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, startPos1, startPos2, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

void ConverLinePath(const ROCKS_PLANE &plane, const ROCKS_POSE &pose, const double &startPos1, const double &endPose1, const double &startPos2, const double &endPose2 , const double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	double distance = sqrt((endPose1 - startPos1) * (endPose1 - startPos1) + (endPose2 - startPos2) * (endPose2 - startPos2));
	double rate1 = (endPose1 - startPos1) / distance;
	double rate2 = (endPose2 - startPos2) / distance;

	switch(plane)
	{
	case ROCKS_PLANE_XY:

		pPosition[0] = CurrentDistance * rate1 + startPos1;
		pPosition[1] = CurrentDistance * rate2 + startPos2;
		pPosition[2] = 0;

		pVelocity[0] = CurrentVelocity * rate1;
		pVelocity[1] = CurrentVelocity * rate2;
		pVelocity[2] = 0;
		break;
	case ROCKS_PLANE_YZ:

		pPosition[0] = 0;
		pPosition[1] = CurrentDistance * rate1 + startPos1;
		pPosition[2] = CurrentDistance * rate2 + startPos2;

		pVelocity[0] = 0;
		pVelocity[1] = CurrentVelocity * rate1;
		pVelocity[2] = CurrentVelocity * rate2;
		break;
	case ROCKS_PLANE_ZX:

		pPosition[0] = CurrentDistance * rate1 + startPos1;
		pPosition[1] = 0;
		pPosition[2] = CurrentDistance * rate2 + startPos2;

		pVelocity[0] = CurrentVelocity * rate1;
		pVelocity[1] = 0;
		pVelocity[2] = CurrentVelocity * rate2;
		break;
	default:
		break;
	}

	if (pose.r.x)
	{
		Roll(pPosition, startPos1, startPos2, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, startPos1, startPos2, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, startPos1, startPos2, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

void ConverLinePath(const double *const pStartPos, const double *const pEndPos, const double &totalDistance, const double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	double rate_x = (pEndPos[0] - pStartPos[0]) / totalDistance;
	double rate_y = (pEndPos[1] - pStartPos[1]) / totalDistance;
	double rate_z = (pEndPos[2] - pStartPos[2]) / totalDistance;

	pPosition[0] = CurrentDistance * rate_x + pStartPos[0];
	pPosition[1] = CurrentDistance * rate_y + pStartPos[1];
	pPosition[2] = CurrentDistance * rate_z + pStartPos[2];

	pVelocity[0] = CurrentVelocity * rate_x;
	pVelocity[1] = CurrentVelocity * rate_y;
	pVelocity[2] = CurrentVelocity * rate_z;
}

void ConvertPathToWorldCoordinate(const ROCKS_MECH* const pMech, uint32_t &index, double *const pPosition, double *const pVelocity)
{
	if (pMech->var.moveType == ROCKS_MOVE_TYPE_CIRCULAR)//check the path type
	{
		ConvertCriclePath(pMech->var.startPos, pMech->var.angle, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index],pMech->var.plane, pMech->var.radius, pMech->var.center, pPosition, pVelocity);
	} 

	if (pMech->var.moveType == ROCKS_MOVE_TYPE_LINEAR)
	{
		ConverLinePath(pMech->var.startPos, pMech->var.endPos, pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines - 1], pMech->var.pPositionSplineBuffer[index],pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
	}

	if (pMech->var.moveType == ROCKS_MOVE_TYPE_MIX)
	{
		if (index == 0)
		{
			mix_Boundary = 0;
		}
		if (index == mix_Boundary)
		{
			mix_Boundary = (uint32_t)(pMech->var.pVelocitySplineBuffer[index + 3] + index + 7);
			switch ((int)(pMech->var.pPositionSplineBuffer[index + 3]) / 256)
			{
			case ROCKS_PLANE_XY:
				mix_plane = ROCKS_PLANE_XY;
				break;
			case ROCKS_PLANE_YZ:
				mix_plane = ROCKS_PLANE_YZ;
				break;
			case ROCKS_PLANE_ZX:
				mix_plane = ROCKS_PLANE_ZX;
				break;
			default:
				break;
			}
			switch ((int)(pMech->var.pPositionSplineBuffer[index + 3]) % 256)
			{
			case ROCKS_MOVE_TYPE_LINEAR://LINE
				mix_moveType = ROCKS_MOVE_TYPE_LINEAR;
				break;
			case ROCKS_MOVE_TYPE_CIRCULAR://CRICLE
				mix_moveType = ROCKS_MOVE_TYPE_CIRCULAR;
				break;
			case ROCKS_MOVE_TYPE_SPIRAL:
				mix_moveType = ROCKS_MOVE_TYPE_SPIRAL;
				break;
			default:
				break;
			}
			mix_endPos_x = pMech->var.pVelocitySplineBuffer[index];
			mix_endPos_y = pMech->var.pPositionSplineBuffer[index];
			mix_endPos_z = pMech->var.pVelocitySplineBuffer[index + 1];
			mix_pose.r.x = pMech->var.pPositionSplineBuffer[index + 1];
			mix_pose.r.y = pMech->var.pPositionSplineBuffer[index + 2];
			mix_pose.r.z = pMech->var.pVelocitySplineBuffer[index + 2];
			mix_startPos1 = pMech->var.pVelocitySplineBuffer[index + 4];
			mix_startPos2 = pMech->var.pPositionSplineBuffer[index + 4];
			switch (mix_moveType)
			{
			case ROCKS_MOVE_TYPE_LINEAR:
				mix_endPos1 = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_endPos2 = pMech->var.pPositionSplineBuffer[index + 5];
				break;
			case ROCKS_MOVE_TYPE_CIRCULAR:
				mix_angle = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_center1 = pMech->var.pVelocitySplineBuffer[index + 6];
				mix_center2 = pMech->var.pPositionSplineBuffer[index + 6];
				break;
			case ROCKS_MOVE_TYPE_SPIRAL:
				mix_endPos1 = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_endPos2 = pMech->var.pPositionSplineBuffer[index + 5];
				mix_center1 = pMech->var.pVelocitySplineBuffer[index + 6];
				mix_center2 = pMech->var.pPositionSplineBuffer[index + 6];
				break;
			default:
				break;
			}
			index += 7;
		}
		switch (mix_moveType)
		{
		case ROCKS_MOVE_TYPE_LINEAR:
			ConverLinePath(mix_plane, mix_pose, mix_startPos1, mix_endPos1, mix_startPos2, mix_endPos2, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
			break;
		case ROCKS_MOVE_TYPE_CIRCULAR:
			ConvertCriclePath(mix_plane, mix_pose, mix_startPos1, mix_startPos2, mix_center1, mix_center2, mix_angle, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
			break;
		case ROCKS_MOVE_TYPE_SPIRAL:
			break;
		default:
			break;
		}
		switch (mix_plane)
		{
		case ROCKS_PLANE_XY://XY-PLANE
			pPosition[2] = mix_endPos_z;
			break;
		case ROCKS_PLANE_YZ://YZ-PLANE
			pPosition[0] = mix_endPos_x;
			break;
		case ROCKS_PLANE_ZX://ZX-PLANE
			pPosition[1] = mix_endPos_y;
			break;
		default:
			break;
		}
	}

	if (pMech->var.refFramePose2.r.x)
	{
		Roll(pPosition, pMech->var.startPos, pMech->var.refFramePose2.r.x);
		Roll(pVelocity, pMech->var.refFramePose2.r.x);
	}

	if (pMech->var.refFramePose2.r.y)
	{
		Pitch(pPosition, pMech->var.startPos, pMech->var.refFramePose2.r.y);
		Pitch(pVelocity, pMech->var.refFramePose2.r.y);
	}

	if (pMech->var.refFramePose2.r.z)
	{
		Yaw(pPosition, pMech->var.startPos, pMech->var.refFramePose2.r.z);
		Yaw(pVelocity, pMech->var.refFramePose2.r.z);
	}

	pPosition[0] += pMech->var.refFramePose2.t.x;
	pPosition[1] += pMech->var.refFramePose2.t.y;
	pPosition[2] += pMech->var.refFramePose2.t.z;
} 
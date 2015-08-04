#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "RotationTransformation.h"

#include <rocksapi.h>

void ConvertCriclePath(ROCKS_PLANE &plane, ROCKS_POSE &pose, double &startPos1, double &startPos2, double &center1, double &center2, double &angle, double &CurrentDistance, double &CurrentVelocity, double *pPosition, double *pVelocity)
{
	double radius = sqrt((center1 - startPos1) * (center1 - startPos1) + (center2 - startPos2) * (center2 - startPos2));
	double relativeAngle = CurrentDistance / radius;
	double alhpa = acos((startPos1 - center1) / radius);
	double beta = (startPos2 - center2) > 0 ? alhpa : - alhpa;
	double absoluteAngle;
	if (angle>0)
	{
		absoluteAngle = beta - relativeAngle;

		switch(plane)
		{
		case ROCKS_PLANE_XY:
			pPosition[0] = center1 + radius * cos(absoluteAngle);
			pPosition[1] = center2 + radius * sin(absoluteAngle);
			pPosition[2] = 0;

			pVelocity[0] =  CurrentVelocity * sin(absoluteAngle);
			pVelocity[1] = -CurrentVelocity * cos(absoluteAngle);
			pVelocity[2] =  0;
			break;
		case ROCKS_PLANE_YZ:
			pPosition[1] = center1 + radius * cos(absoluteAngle);
			pPosition[2] = center2 + radius * sin(absoluteAngle);
			pPosition[0] = 0;

			pVelocity[1] =  CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] = -CurrentVelocity * cos(absoluteAngle);
			pVelocity[0] =  0;
			break;
		case ROCKS_PLANE_ZX:
			pPosition[0] = center1 + radius * cos(absoluteAngle);
			pPosition[2] = center2 + radius * sin(absoluteAngle);
			pPosition[1] = 0;

			pVelocity[0] =  CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] = -CurrentVelocity * cos(absoluteAngle);
			pVelocity[1] =  0;
			break;
		default:
			break;
		}
	}
	else
	{
		absoluteAngle = beta + relativeAngle;

		switch(plane)
		{
		case ROCKS_PLANE_XY:
			pPosition[0] = center1 + radius * cos(absoluteAngle);
			pPosition[1] = center2 + radius * sin(absoluteAngle);
			pPosition[2] = 0;

			pVelocity[0] = -CurrentVelocity * sin(absoluteAngle);
			pVelocity[1] =  CurrentVelocity * cos(absoluteAngle);
			pVelocity[2] =  0;
			break;
		case ROCKS_PLANE_YZ:
			pPosition[1] = center1 + radius * cos(absoluteAngle);
			pPosition[2] = center2 + radius * sin(absoluteAngle);
			pPosition[0] = 0;

			pVelocity[1] = -CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] =  CurrentVelocity * cos(absoluteAngle);
			pVelocity[0] =  0;
			break;
		case ROCKS_PLANE_ZX:
			pPosition[0] = center1 + radius * cos(absoluteAngle);
			pPosition[2] = center2 + radius * sin(absoluteAngle);
			pPosition[1] = 0;

			pVelocity[0] = -CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] =  CurrentVelocity * cos(absoluteAngle);
			pVelocity[1] =  0;
			break;
		default:
			break;
		}
	}


	if (pose.r.x)
	{
		Roll(pPosition, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

void ConvertCriclePath(double *pStartPos, double &totalAngle, double &CurrentDistance, double &CurrentVelocity, ROCKS_PLANE &plane, double &radius, double *pCenter, double *pPosition, double *pVelocity)
{
	double angle;
	double alhpa;
	double beta ;
	double CenterWorldCoordinate[3];
	switch(plane)
	{
	case ROCKS_PLANE_XY:
		//这个只需计算一次！！！
		alhpa = acos((pStartPos[0] - pCenter[0]) / sqrt((pStartPos[0] - pCenter[0]) * (pStartPos[0] - pCenter[0]) + (pStartPos[1] - pCenter[1]) * (pStartPos[1] - pCenter[1])));
		beta = pCenter[1] < 0 ? alhpa : M_PI * 2 - alhpa;

		if (totalAngle > 0)
			angle = beta - CurrentDistance / radius;
		else
			angle = beta + CurrentDistance / radius;

		CenterWorldCoordinate[0] = pCenter[0];
		CenterWorldCoordinate[1] = pCenter[1];
		CenterWorldCoordinate[2] = pStartPos[2];

		pPosition[0] = CenterWorldCoordinate[0] + radius * cos(angle);
		pPosition[1] = CenterWorldCoordinate[1] + radius * sin(angle);
		pPosition[2] = CenterWorldCoordinate[2];

		if (totalAngle > 0)
		{
			pVelocity[0] =  CurrentVelocity * sin(angle);
			pVelocity[1] = -CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}
		else
		{
			pVelocity[0] = -CurrentVelocity * sin(angle);
			pVelocity[1] =  CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}
		break;
	case ROCKS_PLANE_YZ:
		alhpa = acos((pStartPos[1] - pCenter[0]) / sqrt((pStartPos[1] - pCenter[0]) * (pStartPos[1] - pCenter[0]) + (pStartPos[2] - pCenter[1]) * (pStartPos[2] - pCenter[1])));
		beta = pCenter[1] < 0 ? alhpa : M_PI * 2 - alhpa;
		if (totalAngle > 0)
			angle = beta - CurrentDistance / radius;
		else
			angle = beta + CurrentDistance / radius;

		CenterWorldCoordinate[0] = pStartPos[0];
		CenterWorldCoordinate[1] = pCenter[0];
		CenterWorldCoordinate[2] = pCenter[1];

		pPosition[0] = CenterWorldCoordinate[0];
		pPosition[1] = CenterWorldCoordinate[1] + radius * cos(angle);
		pPosition[2] = CenterWorldCoordinate[2] + radius * sin(angle);

		if (totalAngle > 0)
		{
			pVelocity[0] =  0;
			pVelocity[1] =  CurrentVelocity * sin(angle);
			pVelocity[2] = -CurrentVelocity * cos(angle);
		}
		else
		{
			pVelocity[0] =  0;
			pVelocity[1] = -CurrentVelocity * sin(angle);
			pVelocity[2] =  CurrentVelocity * cos(angle);
		} 
		break;
	case ROCKS_PLANE_ZX:
		alhpa = acos((pStartPos[2] - pCenter[0]) / sqrt((pStartPos[2] - pCenter[0]) * (pStartPos[2] - pCenter[0]) + (pStartPos[0] - pCenter[1]) * (pStartPos[0] - pCenter[1])));
		beta = pCenter[1] < 0 ? alhpa : M_PI * 2 - alhpa;
		if (totalAngle > 0)
			angle = beta - CurrentDistance / radius;
		else
			angle = beta + CurrentDistance / radius;

		CenterWorldCoordinate[0] = pCenter[1];
		CenterWorldCoordinate[1] = pStartPos[1];
		CenterWorldCoordinate[2] = pCenter[0];

		pPosition[0] = CenterWorldCoordinate[0] + radius * sin(angle);
		pPosition[1] = CenterWorldCoordinate[1];
		pPosition[2] = CenterWorldCoordinate[2] + radius * cos(angle);

		if (totalAngle > 0)
		{
			pVelocity[0] = -CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] =  CurrentVelocity * sin(angle);
		}
		else
		{
			pVelocity[0] =  CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] = -CurrentVelocity * sin(angle);
		}
		break;
	}

}

void ConverLinePath(ROCKS_PLANE &plane, ROCKS_POSE &pose, double &startPos1, double &endPose1, double &startPos2, double &endPose2 , double &CurrentDistance, double &CurrentVelocity, double *pPosition, double *pVelocity)
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
		Roll(pPosition, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

void ConverLinePath(double *pStartPos, double *pEndPos, double &totalDistance, double &CurrentDistance, double &CurrentVelocity, double *pPosition, double *pVelocity)
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

#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

void Roll(double *pVector, double angle)
{
	double buf =  pVector[1] * cos(angle) + pVector[2] * sin(angle);
	pVector[2] = -pVector[1] * sin(angle) + pVector[2] * cos(angle);
	pVector[1] =  buf;
}

void Pitch(double *pVector, double angle)
{
	double buf = pVector[0] * cos(angle) - pVector[2] * sin(angle);
	pVector[2] = pVector[0] * sin(angle) + pVector[2] * cos(angle);
	pVector[0] = buf;
}

void Yaw(double *pVector, double angle)
{
	double buf =  pVector[0] * cos(angle) + pVector[1] * sin(angle);
	pVector[1] = -pVector[0] * sin(angle) + pVector[1] * cos(angle);
	pVector[0] =  buf;
}

inline const double& CalcRotateAngle(const double &offset_x, const double &offset_y)
{
	if (offset_x == 0) //90 or -90 degree
	{
		if (offset_y == 0)
			return 0.0;
		if ( offset_y > 0) 
			return -M_PI_2;
		else
			return M_PI_2;
	}
	if (offset_x > 0)
	{
		return -atan2(offset_y, offset_x);
	}
	else
	{
		if (offset_y > 0)
		{
			return -M_PI_2 + atan2(offset_y, offset_x);
		}
		if (offset_y == 0)
		{
			return M_PI;
		}
		else
		{
			return M_PI_2 + atan2(offset_y, offset_x);
		}
	}
}

// point1 & point2 have to be different!
// Calculate the Rotate angle base on x axis.
// CW is positive.
inline const double& CalcRotateAngle(const double &point1_x, const double &point1_y, const double &point2_x, const double &point2_y)
{
	return CalcRotateAngle(point2_x - point1_x, point2_y - point1_y);
}
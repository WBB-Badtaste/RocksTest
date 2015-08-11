#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

inline void Roll(double *const pVector, const double &angle)
{
	double buf =  pVector[1] * cos(angle) + pVector[2] * sin(angle);
	pVector[2] = -pVector[1] * sin(angle) + pVector[2] * cos(angle);
	pVector[1] =  buf;
}

inline void Pitch(double *const pVector, const double &angle)
{
	double buf = pVector[0] * cos(angle) - pVector[2] * sin(angle);
	pVector[2] = pVector[0] * sin(angle) + pVector[2] * cos(angle);
	pVector[0] = buf;
}

inline void Yaw(double *const pVector, const double &angle)
{
	double buf =  pVector[0] * cos(angle) + pVector[1] * sin(angle);
	pVector[1] = -pVector[0] * sin(angle) + pVector[1] * cos(angle);
	pVector[0] =  buf;
}

inline void Roll(double *const pVector, const double *const pCenter, const double &angle)
{
	double buf = pCenter[1] + (pVector[1] - pCenter[1]) * cos(angle) + (pVector[2] - pCenter[2]) * sin(angle);
	pVector[2] = pCenter[2] - (pVector[1] - pCenter[1]) * sin(angle) + (pVector[2] - pCenter[2]) * cos(angle);
	pVector[1] = buf;
}

inline void Pitch(double *const pVector, const double *const pCenter, const double &angle)
{
	double buf = pCenter[0] + (pVector[0] - pCenter[0]) * cos(angle) - (pVector[2] - pCenter[2]) * sin(angle);
	pVector[2] = pCenter[2] + (pVector[0] - pCenter[0]) * sin(angle) + (pVector[2] - pCenter[2]) * cos(angle);
	pVector[0] = buf;
}

inline void Yaw(double *const pVector, const double *const pCenter, const double &angle)
{
	double buf = pCenter[0] + (pVector[0] - pCenter[0]) * cos(angle) + (pVector[1] - pCenter[1]) * sin(angle);
	pVector[1] = pCenter[1] - (pVector[0] - pCenter[0]) * sin(angle) + (pVector[1] - pCenter[1]) * cos(angle);
	pVector[0] =  buf;
}

inline void Roll(double *const pVector, const double &center1, const double &center2, const double &angle)
{
	double buf = center1 + (pVector[1] - center1) * cos(angle) + (pVector[2] - center2) * sin(angle);
	pVector[2] = center2 - (pVector[1] - center1) * sin(angle) + (pVector[2] - center2) * cos(angle);
	pVector[1] = buf;
}

inline void Pitch(double *const pVector, const double &center1, const double &center2, const double &angle)
{
	double buf = center1 + (pVector[0] - center1) * cos(angle) - (pVector[2] - center2) * sin(angle);
	pVector[2] = center2 + (pVector[0] - center1) * sin(angle) + (pVector[2] - center2) * cos(angle);
	pVector[0] = buf;
}

inline void Yaw(double *const pVector, const double &center1, const double &center2, const double &angle)
{
	double buf = center1 + (pVector[0] - center1) * cos(angle) + (pVector[1] - center2) * sin(angle);
	pVector[1] = center2 - (pVector[0] - center1) * sin(angle) + (pVector[1] - center2) * cos(angle);
	pVector[0] =  buf;
}

// Calculate the Rotate angle base on x axis.
inline void CalcRotateAngle(double &beta, const double &offset_x, const double &offset_y)
{
	beta = atan2(offset_y, offset_x);
}

// point1 & point2 have to be different!
// Calculate the Rotate angle base on x axis.
inline void CalcRotateAngle(double &beta, const double &point1_x, const double &point1_y, const double &point2_x, const double &point2_y)
{
	CalcRotateAngle(beta, point2_x - point1_x, point2_y - point1_y);
}
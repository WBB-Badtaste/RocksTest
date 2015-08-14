#pragma once

#include <math.h>

void CalcArchimedeSpiralPars(const double *const startPos, const double *const endPos, const double *const center, double *const radius, double &a, double &b)
{
	double offSet1(startPos[0] - center[0]);
	double offSet2(startPos[1] - center[1]);
	double offSet3(endPos[0] - center[0]);
	double offSet4(endPos[1] - center[1]);
	double theta1(atan2(offSet2, offSet1));
	double theta2(atan2(offSet4, offSet3));
	radius[0] = sqrt(offSet2 * offSet2 + offSet1 * offSet1);
	radius[1] = sqrt(offSet4 * offSet4 + offSet3 * offSet3);

	//r=a+b*theta
	b = (radius[0] - radius[1]) / (theta2 - theta1);
	a = radius[1] - b * theta2;
}

void CalcArchimedeSpiralPars(const double &startPos1, const double &startPos2, const double &endPos1, const double &endPos2, const double &center1, const double &center2, double &radius1, double &radius2, double &a, double &b)
{
	double offSet1(startPos1 - center1);
	double offSet2(startPos2 - center2);
	double offSet3(endPos1 - center1);
	double offSet4(endPos2 - center2);
	double theta1(atan2(offSet2, offSet1));
	double theta2(atan2(offSet4, offSet3));
	radius1 = sqrt(offSet2 * offSet2 + offSet1 * offSet1);
	radius2 = sqrt(offSet4 * offSet4 + offSet3 * offSet3);

	//r=a+b*theta
	b = (radius1 - radius2) / (theta2 - theta1);
	a = radius2 - b * theta2;
}

const double CalcArchimedeSpiralArcLen(const double *const radius, const double &a, const double &b)
{
	double fac1(sqrt(radius[0] * radius[0] + b * b));
	double fac2(sqrt(radius[1] * radius[1] + b * b));
	return ((radius[1] / b * fac2 + b * log(radius[1] + fac2)) - ((radius[0] / b * fac1 + b * log(radius[0] + fac1))) / 2.0);
}
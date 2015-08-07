#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include <Windows.h>


typedef struct DeltaMechPars
{
	double e;     // end effector
	double f;     // base
	double re;
	double rf;
}DELTA_MECH_PARS;

// trigonometric constants
const double sqrt3 = sqrt(3.0);
const double sin120 = sqrt3 / 2.0;   
const double cos120 = -0.5;        
const double tan60 = sqrt3;
const double sin30 = 0.5;
const double tan30 = 1 / sqrt3;

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
BOOL DeltaCalcPosForward(const DELTA_MECH_PARS &deltaMechPars, const double* const pJointPos, double* const pCartesianPos) 
{
	double t(deltaMechPars.f - deltaMechPars.e);

	double y1(-(t + deltaMechPars.rf * cos(pJointPos[0])));
	double z1( -deltaMechPars.rf * sin(pJointPos[0]));

	double y2((t + deltaMechPars.rf * cos(pJointPos[1])) * sin30);  
	double x2(y2 * tan60);
	double z2(-deltaMechPars.rf * sin(pJointPos[1]));

	double y3((t + deltaMechPars.rf * cos(pJointPos[2])) * sin30);
	double x3(-y3 * tan60);
	double z3(-deltaMechPars.rf * sin(pJointPos[2]));

	double dnm((y2 - y1) * x3 - (y3 - y1) * x2);

	double w1(y1 * y1 + z1 * z1);
	double w2(x2 * x2 + y2 * y2 + z2 * z2);
	double w3(x3 * x3 + y3 * y3 + z3 * z3);

	// x = (a1*z + b1)/dnm
	double a1((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1));
	double b1(-((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) * 0.5);

	// y = (a2*z + b2)/dnm;
	double a2(-(z2 - z1) * x3 + (z3 - z1) * x2);
	double b2(((w2 - w1) * x3 - (w3 - w1) * x2) * 0.5);

	// a*z^2 + b*z + c = 0
	double a(a1 * a1 + a2 * a2 + dnm * dnm);
	double b(2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm));
	double c((b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - deltaMechPars.re * deltaMechPars.re));

	// discriminant
	double d(b * b - 4.0 * a * c);
	if (d < 0) return FALSE; // non-existing point

	pCartesianPos[2] = -0.5 * (b + sqrt(d)) / a;
	pCartesianPos[0] = (a1 * pCartesianPos[2] + b1) / dnm;
	pCartesianPos[1] = (a2 * pCartesianPos[2] + b2) / dnm;

	return TRUE;
} 

// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
BOOL DeltaCalcAngleYZ(const DELTA_MECH_PARS &deltaMechPars, const double &x, const double &y, const double &z, double &theta) 
{
	// z = a + b*y
	double a((x * x + (y - deltaMechPars.e) * (y - deltaMechPars.e) + z * z + deltaMechPars.rf * deltaMechPars.rf - deltaMechPars.re * deltaMechPars.re - deltaMechPars.f * deltaMechPars.f) / (2 * z));
	double b((-deltaMechPars.f - (y - deltaMechPars.e)) / z);
	// discriminant
	double d(-(a - b * deltaMechPars.f) * (a - b * deltaMechPars.f) + deltaMechPars.rf * (b * b * deltaMechPars.rf + deltaMechPars.rf)); 
	if (d < 0) 
		return FALSE; // non-existing point
	double yj((-deltaMechPars.f - a * b - sqrt(d)) / (b * b + 1)); // choosing outer point
	double zj(a + b * yj);
	theta = atan(zj / (yj + deltaMechPars.f));//range of ¨C¦Ð/2 to ¦Ð/2 radians. good choice.
	return TRUE;
} 

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
BOOL DeltaCalcPosInverse(const DELTA_MECH_PARS &deltaMechPars, const double* const pCartesianPos, double* const pJointPos) 
{
	pJointPos[0] = pJointPos[1] = pJointPos[2] = 0;

	BOOL status = DeltaCalcAngleYZ(deltaMechPars, pCartesianPos[0], pCartesianPos[1], pCartesianPos[2], pJointPos[0]);
	if (status == TRUE) 
		status = DeltaCalcAngleYZ(deltaMechPars, pCartesianPos[0] * cos120 + pCartesianPos[1] * sin120, pCartesianPos[1] * cos120 - pCartesianPos[0] * sin120, pCartesianPos[2], pJointPos[1]);  // rotate coords to +120 deg
	if (status == TRUE) 
		status = DeltaCalcAngleYZ(deltaMechPars, pCartesianPos[0] * cos120 - pCartesianPos[1] * sin120, pCartesianPos[1] * cos120 + pCartesianPos[0] * sin120, pCartesianPos[2], pJointPos[2]);  // rotate coords to -120 deg
	
	return status;
}

//vel IK
BOOL DeltaCalcVelInverse(const DELTA_MECH_PARS &deltaMechPars, const double* const pCartesianPos, const double* const pCartesianVel, const double* const pJointPos, double* const pJointVel)
{
	double wb(deltaMechPars.f);
	double up(deltaMechPars.e);
	double wp(up / 2.0);

	double a(wb - up);
	double b(sqrt3 / 2.0 * (deltaMechPars.e - wb));
	double c(wp - wb / 2.0); 

	double cos_theta1(cos(pJointPos[0]));
	double sin_theta1(sin(pJointPos[0]));
	double cos_theta2(cos(pJointPos[1]));
	double sin_theta2(sin(pJointPos[1]));
	double cos_theta3(cos(pJointPos[2]));
	double sin_theta3(sin(pJointPos[2]));

	pJointVel[0] = (pCartesianPos[0] * pCartesianVel[0] + ((pCartesianPos[1] + a) + deltaMechPars.rf * cos_theta1) * pCartesianVel[1] + (pCartesianPos[2] + deltaMechPars.rf * sin_theta1)* pCartesianVel[2]) / (deltaMechPars.rf * ((pCartesianPos[1] + a) * sin_theta1 -pCartesianPos[2] * cos_theta1));
	pJointVel[1] = ((2.0 * (pCartesianPos[0] + b) - sqrt3 * deltaMechPars.rf * cos_theta2) * pCartesianVel[0] + (2.0 * (pCartesianPos[1] + c) - deltaMechPars.rf * cos_theta2) * pCartesianVel[1] + 2.0 * (pCartesianPos[2] + deltaMechPars.rf * sin_theta2) * pCartesianVel[2]) / (-deltaMechPars.rf * ((sqrt3 * (pCartesianPos[0] + b) + pCartesianPos[1] + c) * sin_theta2 + 2.0 * pCartesianPos[2] * cos_theta2));
	pJointVel[2] = ((2.0 * (pCartesianPos[0] - b) + sqrt3 * deltaMechPars.rf * cos_theta3) * pCartesianVel[0] + (2.0 * (pCartesianPos[1] + c) - deltaMechPars.rf * cos_theta3) * pCartesianVel[1] + 2.0 * (pCartesianPos[2] + deltaMechPars.rf * sin_theta3) * pCartesianVel[2]) / ( deltaMechPars.rf * ((sqrt3 * (pCartesianPos[0] - b) - pCartesianPos[1] - c) * sin_theta3 - 2.0 * pCartesianPos[2] * cos_theta3));
	return TRUE; 
}

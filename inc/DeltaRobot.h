
#include "DeltaInverseKinVel.h"
#include "Geschkinematik.h"

#define _USE_MATH_DEFINES
#include <math.h>

// trigonometric constants
const double sqrt3 = sqrt(3.0);
const double sin120 = sqrt3 / 2.0;   
const double cos120 = -0.5;        
const double tan60 = sqrt3;
const double sin30 = 0.5;
const double tan30 = 1 / sqrt3;

// robot geometry
// (look at M_PIcs above for explanation) 
double e = 65.0;     // end effector
double f = 220.0;     // base
double re = 650.0;
double rf = 330.0;

double mm2pu = 1;//convert mm unit to pu
    
// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_calcForward(double theta1, double theta2, double theta3, double &x0, double &y0, double &z0) {
	double t = f - e;
	
	double y1 = -(t + rf * cos(theta1));
	double z1 = -rf * sin(theta1);
	
	double y2 = (t + rf * cos(theta2)) * sin30;
	double x2 = y2 * tan60;
	double z2 = -rf * sin(theta2);
	
	double y3 = (t + rf * cos(theta3)) * sin30;
	double x3 = -y3 * tan60;
	double z3 = -rf * sin(theta3);
	
	double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
	
	double w1 = y1 * y1 + z1 * z1;
	double w2 = x2 * x2 + y2 * y2 + z2*z2;
	double w3 = x3 * x3 + y3 * y3 + z3*z3;
	
	// x = (a1*z + b1)/dnm
	double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
	double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) * 0.5;
	
	// y = (a2*z + b2)/dnm;
	double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
	double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) * 0.5;
	
	// a*z^2 + b*z + c = 0
	double a = a1 * a1 + a2 * a2 + dnm * dnm;
	double b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
	double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);
	
	// discriminant
	double d = b * b - 4.0 * a * c;
	if (d < 0) return -1; // non-existing point
	
	z0 = -0.5 * (b + sqrt(d)) / a;
	x0 = (a1 * z0 + b1) / dnm;
	y0 = (a2 * z0 + b2) / dnm;
	 
	return 0;
} 
 
// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int delta_calcAngleYZ(double x0, double y0, double z0, double &theta) {
	double y1 = -f; 
	y0 -= e;    // shift center to edge
	// z = a + b*y
	double a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
	double b = (y1 - y0) / z0;
	// discriminant
	double d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf); 
	if (d < 0) return -1; // non-existing point
	double yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
	double zj = a + b * yj;
	theta = atan(zj / (yj - y1));
	return 0;
} 
 
// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(double x0, double y0, double z0, double &theta1, double &theta2, double &theta3) {
	theta1 = theta2 = theta3 = 0;
	int status = delta_calcAngleYZ(x0, y0, z0, theta1);
	if (status == 0) status = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, theta2);  // rotate coords to +120 deg
	if (status == 0) status = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, theta3);  // rotate coords to -120 deg
	return status;
}

//vel IK
int delta_velInverse(double x, double y, double z, double v_x, double v_y, double v_z, double theta1, double theta2, double theta3, double& vel_theta1, double& vel_theta2, double& vel_theta3)
{
	double wb = f;
	double up = e;
	double wp = up / 2;

	double a = wb - up;
	double b = sqrt3 / 2 * (e - wb);
	double c = wp - wb / 2; 

	double cos_theta1 = cos(theta1);
	double sin_theta1 = sin(theta1);
	double cos_theta2 = cos(theta2);
	double sin_theta2 = sin(theta2);
	double cos_theta3 = cos(theta3);
	double sin_theta3 = sin(theta3);

	vel_theta1 = (x * v_x + ((y + a) + rf * cos_theta1) * v_y + (z + rf * sin_theta1)* v_z) / (rf * ((y + a) * sin_theta1 - z * cos_theta1));
	vel_theta2 = ((2 * (x + b) - sqrt3 * rf * cos_theta2) * v_x + (2 * (y + c) - rf * cos_theta2) * v_y + 2 * (z + rf * sin_theta2 * v_z)) / (-rf * ((sqrt3 * (x + b) + y + c) * sin_theta2 + 2 * z * cos_theta2));
	vel_theta3 = ((2 * (x - b) + sqrt3 * rf * cos_theta3) * v_x + (2 * (y + c) - rf * cos_theta3) * v_y + 2 * (z + rf * sin_theta3 * v_z)) / ( rf * ((sqrt3 * (x - b) - y - c) * sin_theta3 - 2 * z * cos_theta3));
	return 0; 
}


#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "glut.h"


class Camera
{
private:
	const double rateDis2Ang;
	const double moveDis;
	double eyeDistance;
	double eyeAlpha;
	double eyeBeta;
	double target[3];
	double eye[3];
	double alphaOffset;
	double betaOffset;
	double disOffset;
	
public:
	Camera(double t[]);
	~Camera(void);

	void ReSet();
	void SetLookAt();
	void Store();
	void MoveAngle(double xDis, double yDis);
	void MoveDistance(double xDis, double yDis, double zDis);
	void MoveFront();
	void MoveBack();
};


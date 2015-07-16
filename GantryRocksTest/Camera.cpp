#include "StdAfx.h"
#include "Camera.h"


Camera::Camera(double t[]):
	rateDis2Ang(M_PI / 100),
	alphaOffset(0),
	betaOffset(0),
	disOffset(0),
	moveDis(30)
{
	target[0] = t[0];
	target[1] = t[1];
	target[2] = t[2];
	ReSet();
}

Camera::~Camera(void)
{
}

void Camera::ReSet()
{
	eyeDistance = 1000;
	eyeBeta = M_PI_4;
	eyeAlpha = M_PI_4;
	SetLookAt();
}

void Camera::SetLookAt()
{
	double cosA = cos(eyeAlpha + alphaOffset);
	int k = cosA <0 ? M_PI : 0;
	eye[0] = target[0] + (eyeDistance + disOffset) * cosA * cos(eyeBeta + betaOffset + k);
	eye[1] = target[1] + (eyeDistance + disOffset) * sin(eyeAlpha + alphaOffset);
	eye[2] = target[2] + (eyeDistance + disOffset) * cosA * sin(eyeBeta + betaOffset + k);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye[0], eye[1], eye[2], target[0], target[1], target[2], 0, 1, 0);  
}

void Camera::Store()
{
	eyeAlpha += alphaOffset;
	eyeBeta += betaOffset;
	eyeDistance += disOffset;
	alphaOffset = 0;
	betaOffset = 0;
	disOffset = 0;
}

void Camera::MoveAngle(double xDis, double yDis)
{
	alphaOffset = yDis * rateDis2Ang;
	betaOffset = xDis * rateDis2Ang;
	SetLookAt();
}

void Camera::MoveFront()
{
	eyeDistance += moveDis;
	SetLookAt();
}

void Camera::MoveBack()
{
	eyeDistance -= moveDis;
	SetLookAt();
}

void Camera::MoveDistance(double xDis, double yDis, double zDis)
{
	double buffer[5];
	buffer[0] = (eyeDistance + disOffset) * cos(eyeAlpha + alphaOffset) * cos(eyeBeta + betaOffset) + xDis;
	buffer[1] = (eyeDistance + disOffset) * sin(eyeAlpha + alphaOffset) + yDis;
	buffer[2] = (eyeDistance + disOffset) * cos(eyeAlpha + alphaOffset) * sin(eyeBeta + betaOffset) + zDis;
	buffer[3] = sqrt(buffer[0] * buffer[0] + buffer[1] * buffer[1] + buffer[2] * buffer[2]);
	disOffset =  buffer[3] - eyeDistance;
	buffer[4] =  asin(buffer[1] / buffer[3]);
	alphaOffset = buffer[4] - eyeAlpha;
	betaOffset = asin(buffer[2] / buffer[3] / cos(buffer[4]));
}
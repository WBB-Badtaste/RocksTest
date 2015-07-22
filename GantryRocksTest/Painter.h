#pragma once

#include "glut.h"
#include <windows.h>  

#define _USE_MATH_DEFINES
#include <math.h>

#define BUFFER_SIZE 1000

class Painter
{
private:
	const double scale;
	double drawPoint[BUFFER_SIZE][3];
	int indexDraw;
	HANDLE  hMutex;

	double coordinate_x_min;
	double coordinate_x_max;
	double coordinate_y_min;
	double coordinate_y_max;
	double coordinate_z_min;
	double coordinate_z_max;

	void DrawCoordinate();
	void DrawPath();
	void DrawSphere(double xx, double yy, double zz, double radius, double M, double N);
public:
	
	Painter(double coordinate[]);
	~Painter(void);

	void AddTcpPoint(double point[]);
	void Draw();
};


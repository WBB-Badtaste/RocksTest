#pragma once


#include "Painter.h"
#include "Camera.h"

enum mouseMotionType
{
	noMotion = 0,
	xRotation = 1,
	yRotation = 2,
	xTranslation = 3,
	yTranslation = 4,
	zTranslation = 5	
};

enum mousebuttonType
{
	noButton = 0,
	leftButton = 1,
	middleButton = 2,
	rightButton = 3,
	twoButton = 4
};

typedef void (*STATEMACHINE_FUNC)(void);

static mouseMotionType mmt;
static mousebuttonType mbt;
static double mmThreshold ;
static int mouseLocation[2];

static Camera* pCamera;
static Painter* pPainter;
static STATEMACHINE_FUNC StateMachine;

class Drawer
{
public:
	Drawer(double target[],double coordinate[],STATEMACHINE_FUNC);
	~Drawer(void);

	void StartUp(int argc,char** argv);

private:

	static void KeyboardFunc(unsigned char key, int x, int y);
	static void MousePassFunc(int button, int state, int x, int y);
	static void MouseMotionFunc(int x, int y);
	static void OpenglDraw();
};


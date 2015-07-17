#include "StdAfx.h"
#include "Drawer.h"

double screenWidth=GetSystemMetrics(SM_CXSCREEN);  
double screenHeight=GetSystemMetrics(SM_CYSCREEN); 

Drawer::Drawer(double target[],double coordinate[],STATEMACHINE_FUNC p,int* state)
{
 	mmt = noMotion;
	mbt = noButton;
	mmThreshold = 2;
	pCamera= new Camera(target);
	pPainter= new Painter(coordinate);
	StateMachine = p;
	pState = state;
}

Drawer::~Drawer(void)
{
	delete pCamera;
	delete pPainter;
}

void Drawer::KeyboardFunc(unsigned char key, int x, int y)
{
	switch(key)
	{
	case 'a':
	case 'A':
		*pState = 5;
		break;
	case 'c':
	case 'C':
		*pState = 7;
		break;
	case 'm':
	case 'M':
		*pState = 8;
		break;
	case 'w':
	case 'W':
		pCamera->MoveFront();
		break;
	case 's':
	case 'S':
		pCamera->MoveBack();		
		break;
	default:
		break;
	}
}

void Drawer::MousePassFunc(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		mouseLocation[0] = x;
		mouseLocation[1] = y;
		switch(button)
		{
		case GLUT_LEFT_BUTTON:
			if (mbt == rightButton)
			{
				if (x - mouseLocation[0] && y - mouseLocation[1])
				{
					pCamera->Store();
				}
				mmt = noMotion;
				mbt = twoButton;
			}
			else
				mbt = leftButton;
			break;
		case GLUT_MIDDLE_BUTTON:
			mbt = middleButton;
			break;
		case GLUT_RIGHT_BUTTON:
			if (mbt == leftButton)
			{
				if (x - mouseLocation[0] && y - mouseLocation[1])
				{
					pCamera->Store();
				}
				mmt = noMotion;
				mbt = twoButton;
			}
			else
				mbt = rightButton;
			break;
		default:
			break;
		}
	}
	if (state == GLUT_UP)
	{
		if (x - mouseLocation[0] && y - mouseLocation[1])
		{
			pCamera->Store();
		}
		mmt = noMotion;
		if (mbt == twoButton)
		{
			if (button == GLUT_LEFT_BUTTON)
			{
				mbt = rightButton;
			}
			if (button == GLUT_RIGHT_BUTTON)
			{
				mbt = leftButton;
			}
		}
		else
		{
			mbt = noButton;
		}
	}
}

void Drawer::MouseMotionFunc(int x, int y)
{
	switch(mmt)
	{
	case noMotion:
		if (mbt == leftButton)
		{
			if (x - mouseLocation[0] > mmThreshold || mouseLocation[0] - x > mmThreshold)
				mmt = xRotation;
			if (y - mouseLocation[1] > mmThreshold || mouseLocation[1] - y > mmThreshold)
				mmt = yRotation;
		}
		if (mbt == rightButton)
		{
			if (x - mouseLocation[0] > mmThreshold || mouseLocation[0] - x > mmThreshold)
				mmt = xTranslation;
			if (y - mouseLocation[1] > mmThreshold || mouseLocation[1] - y > mmThreshold)
				mmt = yTranslation;
		}
		if (mbt == twoButton)
		{
			if (y - mouseLocation[1] > mmThreshold || mouseLocation[1] - y > mmThreshold)
				mmt = zTranslation;
		}
	case xRotation:
		pCamera->MoveAngle(x - mouseLocation[0], 0);
		break;
	case yRotation:
		pCamera->MoveAngle(0, y - mouseLocation[1]);
		break;
	case xTranslation:
		pCamera->MoveDistance(x - mouseLocation[0], 0, 0);
		break;
	case yTranslation:
		pCamera->MoveDistance(0, y - mouseLocation[1], 0);
		break;
	case zTranslation:
		pCamera->MoveDistance(0, 0, y - mouseLocation[1]);
		break;
	default:
		break;
	}
}

void Drawer::OpenglDraw()
{
	StateMachine();
	pPainter->Draw();
}

void Drawer::StartUp(int argc, char **argv)
{
	glutInit(&argc, argv);  // 初始化GLUT
	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE); // 修改了参数为GLUT_SINGLE (单缓冲)和GLUT_RGB(非索引)
	glutInitWindowPosition(100, 100);  // 显示窗口在屏幕的相对位置
	glutInitWindowSize(screenWidth / 2, screenHeight / 2); // 设置显示窗口大小

	glutCreateWindow("DeltaRobotTCPScope"); // 创建窗口，附带标题
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30, screenWidth / screenHeight, 0, 1000);

	glutDisplayFunc(OpenglDraw);
	glutIdleFunc(OpenglDraw);
	glutKeyboardFunc(KeyboardFunc);
	glutMouseFunc(MousePassFunc);
	glutMotionFunc(MouseMotionFunc);
	glutMainLoop(); // GLUT 状态机

	
}

void Drawer::AddTcpPoint(double point[])
{
	pPainter->AddTcpPoint(point);
}


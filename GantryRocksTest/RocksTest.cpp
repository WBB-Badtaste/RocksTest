// GantryRocksTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <process.h>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

#include <nyceapi.h>
#include <sacapi.h>
#include <nhiapi.h>  
#include <rocksapi.h>

#include "DeltaRobot.h"
#include "Drawer.h"

using namespace std;

//#define USE_NODE
#define USE_AXIS
//#define SIM_METHOD

//const double rate_angle2pu = 0.5 / M_PI * 10000;
const double rate_angle2pu = 131072 * 11 / (2 * M_PI);

NYCE_STATUS nyceStatus;
HANDLE hEvStop;
HANDLE hAuto;

HANDLE hAutoPathUsing;
int pathType = 1;

LONG64 times = 0;
LONG64 timesCounter = 0;


double coordinate[6] = {0, 200, -535, -335, -100, 100};
double target[3] = {(coordinate[1] + coordinate[0]) / 2, (coordinate[3] + coordinate[2]) / 2, (coordinate[5] + coordinate[4]) / 2};

Drawer *pDrawer;

ROCKS_KIN_INV_PARS kinPars;
ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccPars;
ROCKS_TRAJ_SEGMENT_START_PARS segStartPars;
ROCKS_TRAJ_SEGMENT_LINE_PARS segLinePars1,segLinePars2,segLinePars3,segLinePars4,segLinePars5;
ROCKS_TRAJ_SEGMENT_ARC_PARS segArcPars1,segArcPars2,segArcPars3,segArcPars4;

double segDistance[10];

// double position[1000000][3];
// double velocity[1000000][3];
// 
// uint32_t realSegNum = 0;

void HandleError(const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusString(nyceStatus)<<"\n"<<endl;
}

void HandleError(NYCE_STATUS Status, const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusString(Status)<<"\n"<<endl;
	cout<<"Run times:"<<timesCounter<<endl;
}

#ifdef USE_NODE
#define NUM_NODE 1
const char *noName[ NUM_NODE ] = { "NY4112_node"};
NHI_NODE noId[ NUM_NODE ];
BOOL noCon[ NUM_NODE ];
void TermNode(void)
{
	int no;
	nyceStatus = NYCE_OK;
	for(no = 0; no < NUM_NODE; ++no)
	{
		if (noCon[no] != TRUE) continue;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiDisconnect(noId[no]);
		if(NyceError(nyceStatus)) HandleError(noName[no]);
		else noCon[no] = FALSE;
	}
}

BOOL InitNode(void)
{
	int no;
	nyceStatus = NYCE_OK;
	for (no = 0; no < NUM_NODE; ++no)
	{
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiConnect(noName[no],&noId[no]);
		if (NyceSuccess(nyceStatus) ) noCon[no] = TRUE;
		if (NyceError(nyceStatus) )
		{
			HandleError(noName[no]);
			TermNode();
			return FALSE;
		}
	}
	return TRUE;
}
#endif // USE_NODE

#ifdef USE_AXIS
#define NUM_AXES 3
#if NUM_AXES == 2
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2"};
#endif
#if NUM_AXES == 3
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3"};
#endif
#if NUM_AXES == 4
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3", "DEF_AXIS_4"};
#endif
SAC_AXIS axId[ NUM_AXES ];
BOOL axCon[ NUM_AXES ];
void TermAxis(void)
{
	int ax;
	SAC_STATE sacState = SAC_NO_STATE;
	SAC_SPG_STATE sacSpgState;
	nyceStatus = NYCE_OK;
	for (ax = 0; ax < NUM_AXES; ax++ )
	{
		if (axCon[ax] != TRUE) continue;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReadState(axId[ ax ], &sacState, &sacSpgState);
		if(NyceSuccess(nyceStatus) && sacState == SAC_MOVING)
		{
			nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacQuickStop(axId[ ax ]);
			if (NyceSuccess(nyceStatus))
			{
				nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_MOTION_STOPPED, 10 );
				if (NyceError(nyceStatus))
				{
					HandleError(axName[ ax ]);
					nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReset(axId[ ax ]);
					nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_RESET, 10 );
				}
			}
		}
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacShutdown(axId[ ax ]);
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_SHUTDOWN, 10 );
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDisconnect(axId[ ax ]);
		if(NyceError(nyceStatus)) HandleError(axName[ ax ]);
		else axCon[ax] = FALSE;
	}
}

BOOL InitAxis(void)
{
	int ax;
	SAC_SPG_STATE sacSpgState;
	SAC_STATE sacState;
	SAC_CONFIGURE_AXIS_PARS axisPars;
	double signal = 0;
	nyceStatus = NYCE_OK;
	for (ax = 0; ax < NUM_AXES; ax++ )
	{
		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacConnect( axName[ ax ], &axId[ ax ] );
		if ( NyceSuccess(nyceStatus)) axCon[ax] = TRUE;

		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReadState( axId[ ax ], &sacState, &sacSpgState);
		if(NyceSuccess(nyceStatus))
		{
			switch (sacState)
			{
			default:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacShutdown( axId[ ax ]);
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_SHUTDOWN, 10 );

			case SAC_IDLE:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacInitialize( axId[ ax ], SAC_USE_FLASH );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_INITIALIZE, 10 );

				goto INACTIVE;

			case SAC_READY:
			case SAC_ERROR:
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReset(axId[ ax ]);
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize(axId[ ax ], SAC_REQ_RESET,10);

			case SAC_INACTIVE:
INACTIVE:		nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacHome( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_HOMING_COMPLETED, 10 );

				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacGetAxisConfiguration( axId[ ax ], &axisPars );

				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacReadVariable(axId[ ax ], SAC_VAR_BLAC_ALIGNED, &signal);
				if (signal == 0)
				{
					if ( NyceSuccess(nyceStatus) && axisPars.motorType == SAC_BRUSHLESS_AC_MOTOR )
					{
						nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacAlignMotor( axId[ ax ] );
						nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_ALIGN_MOTOR, 10 );
					}
				}

			case SAC_FREE:

				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacLock( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_LOCK, 10 );

				break;
			}
		}

		if(NyceError(nyceStatus))
		{                                     
			HandleError(axName[ ax ]);
			TermAxis();
			return FALSE;
		}
	}
	return TRUE;
}
#endif // USE_AXIS

uint32_t posVarIndex[NUM_AXES];

ROCKS_MECH  m_mech;
ROCKS_TRAJ_PATH rocksTrajPath;
ROCKS_POSE rocksPose;
BOOL bRocksTerm = FALSE;
BOOL bTurn = FALSE;
HANDLE hThreadRocks;

NYCE_STATUS RocksKinForwardDelta(struct rocks_mech* pMech, const double pJointPos[], double pMechPos[])
{
	ZeroMemory(pMechPos, sizeof(double) * ROCKS_MECH_MAX_DOF);

	if(delta_calcForward(pJointPos[0] / rate_angle2pu, pJointPos[1] / rate_angle2pu, pJointPos[2] / rate_angle2pu, pMechPos[0], pMechPos[1], pMechPos[2]))
		return -1;
	return NYCE_OK; 
}

NYCE_STATUS RocksKinDeltaPosition(struct rocks_mech* pMech, double pPos[])
{
	double pJointPos[ROCKS_MECH_MAX_DOF];
	NYCE_STATUS status = NYCE_OK;
	//	status = NyceError(status) ? status : NyceReadVariableSet(posVarIndex[0], posVarIndex[NUM_AXES - 1], pJointPos);
	for (uint32_t ax = 0; ax < NUM_AXES; ax++)
	{
		status = NyceError(status) ? status : SacReadVariable(axId[ax], SAC_VAR_SETPOINT_POS, &pJointPos[ax]);
	}
	status = NyceError(status) ? status : RocksKinForwardDelta(pMech, pJointPos, pPos);	
	if (NyceError(status))
	{
		HandleError(status, "RocksKinDeltaPosition");
		return -1;
	}
	return status;
}

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

void ConvertCriclePath(ROCKS_PLANE &plane, ROCKS_POSE &pose, double &startPos1, double &startPos2, double &center1, double &center2, double &angle, double &CurrentDistance, double &CurrentVelocity, double *pPosition, double *pVelocity)
{
	double radius(sqrt((center1 - startPos1) * (center1 - startPos1) + (center2 - startPos2) * (center2 - startPos2)));
	double relativeAngle(CurrentDistance / radius);
	double beta((startPos2 - center2) > 0 ? acos((startPos1 - center1) / radius) : -acos((startPos1 - center1) / radius));
	double absoluteAngle; 
	if (angle>0)
	{
		absoluteAngle = beta - relativeAngle;

		switch(plane)
		{
		case ROCKS_PLANE_XY:
			pPosition[0] =  center1 + radius * cos(absoluteAngle);
			pPosition[1] =  center2 + radius * sin(absoluteAngle);
			pPosition[2] =  0;

			pVelocity[0] =  CurrentVelocity * sin(absoluteAngle);
			pVelocity[1] = -CurrentVelocity * cos(absoluteAngle);
			pVelocity[2] =  0;
			break;
		case ROCKS_PLANE_YZ:
			pPosition[1] =  center1 + radius * cos(absoluteAngle);
			pPosition[2] =  center2 + radius * sin(absoluteAngle);
			pPosition[0] =  0;

			pVelocity[1] =  CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] = -CurrentVelocity * cos(absoluteAngle);
			pVelocity[0] =  0;
			break;
		case ROCKS_PLANE_ZX:
			pPosition[0] =  center1 + radius * cos(absoluteAngle);
			pPosition[2] =  center2 + radius * sin(absoluteAngle);
			pPosition[1] =  0;

			pVelocity[0] =  CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] = -CurrentVelocity * cos(absoluteAngle);
			pVelocity[1] =  0;
			break;
		default: 
			break;
		}
	}
	else
	{
		absoluteAngle = beta + relativeAngle;

		switch(plane)
		{
		case ROCKS_PLANE_XY:
			pPosition[0] =  center1 + radius * cos(absoluteAngle);
			pPosition[1] =  center2 + radius * sin(absoluteAngle);
			pPosition[2] =  0;

			pVelocity[0] = -CurrentVelocity * sin(absoluteAngle);
			pVelocity[1] =  CurrentVelocity * cos(absoluteAngle);
			pVelocity[2] =  0;
			break;
		case ROCKS_PLANE_YZ:
			pPosition[1] =  center1 + radius * cos(absoluteAngle);
			pPosition[2] =  center2 + radius * sin(absoluteAngle);
			pPosition[0] =  0;

			pVelocity[1] = -CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] =  CurrentVelocity * cos(absoluteAngle);
			pVelocity[0] =  0;
			break;
		case ROCKS_PLANE_ZX:
			pPosition[0] =  center1 + radius * cos(absoluteAngle);
			pPosition[2] =  center2 + radius * sin(absoluteAngle);
			pPosition[1] =  0;

			pVelocity[0] = -CurrentVelocity * sin(absoluteAngle);
			pVelocity[2] =  CurrentVelocity * cos(absoluteAngle);
			pVelocity[1] =  0;
			break;
		default:
			break;
		}
	}
	

	if (pose.r.x)
	{
		Roll(pPosition, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

void ConvertCriclePath(double *pStartPos, double &totalAngle, double &CurrentDistance, double &CurrentVelocity, ROCKS_PLANE &plane, double &radius, double *pCenter, double *pPosition, double *pVelocity)
{
	double angle;
	double alhpa;
	double beta ;
	double CenterWorldCoordinate[3];
	switch(plane)
	{
	case ROCKS_PLANE_XY:
		//这个只需计算一次！！！
		alhpa = acos((pStartPos[0] - pCenter[0]) / sqrt((pStartPos[0] - pCenter[0]) * (pStartPos[0] - pCenter[0]) + (pStartPos[1] - pCenter[1]) * (pStartPos[1] - pCenter[1])));
		beta = pCenter[1] < 0 ? alhpa : M_PI * 2 - alhpa;

		if (totalAngle > 0)
			angle = beta - CurrentDistance / radius;
		else
			angle = beta + CurrentDistance / radius;

		CenterWorldCoordinate[0] = pCenter[0];
		CenterWorldCoordinate[1] = pCenter[1];
		CenterWorldCoordinate[2] = pStartPos[2];

		pPosition[0] = CenterWorldCoordinate[0] + radius * cos(angle);
		pPosition[1] = CenterWorldCoordinate[1] + radius * sin(angle);
		pPosition[2] = CenterWorldCoordinate[2];

		if (totalAngle > 0)
		{
			pVelocity[0] =  CurrentVelocity * sin(angle);
			pVelocity[1] = -CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}
		else
		{
			pVelocity[0] = -CurrentVelocity * sin(angle);
			pVelocity[1] =  CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}
		break;
	case ROCKS_PLANE_YZ:
		alhpa = acos((pStartPos[1] - pCenter[0]) / sqrt((pStartPos[1] - pCenter[0]) * (pStartPos[1] - pCenter[0]) + (pStartPos[2] - pCenter[1]) * (pStartPos[2] - pCenter[1])));
		beta = pCenter[1] < 0 ? alhpa : M_PI * 2 - alhpa;
		if (totalAngle > 0)
			angle = beta - CurrentDistance / radius;
		else
			angle = beta + CurrentDistance / radius;

		CenterWorldCoordinate[0] = pStartPos[0];
		CenterWorldCoordinate[1] = pCenter[0];
		CenterWorldCoordinate[2] = pCenter[1];

		pPosition[0] = CenterWorldCoordinate[0];
		pPosition[1] = CenterWorldCoordinate[1] + radius * cos(angle);
		pPosition[2] = CenterWorldCoordinate[2] + radius * sin(angle);

		if (totalAngle > 0)
		{
			pVelocity[0] =  0;
			pVelocity[1] =  CurrentVelocity * sin(angle);
			pVelocity[2] = -CurrentVelocity * cos(angle);
		}
		else
		{
			pVelocity[0] =  0;
			pVelocity[1] = -CurrentVelocity * sin(angle);
			pVelocity[2] =  CurrentVelocity * cos(angle);
		} 
		break;
	case ROCKS_PLANE_ZX:
		alhpa = acos((pStartPos[2] - pCenter[0]) / sqrt((pStartPos[2] - pCenter[0]) * (pStartPos[2] - pCenter[0]) + (pStartPos[0] - pCenter[1]) * (pStartPos[0] - pCenter[1])));
		beta = pCenter[1] < 0 ? alhpa : M_PI * 2 - alhpa;
		if (totalAngle > 0)
			angle = beta - CurrentDistance / radius;
		else
			angle = beta + CurrentDistance / radius;

		CenterWorldCoordinate[0] = pCenter[1];
		CenterWorldCoordinate[1] = pStartPos[1];
		CenterWorldCoordinate[2] = pCenter[0];

		pPosition[0] = CenterWorldCoordinate[0] + radius * sin(angle);
		pPosition[1] = CenterWorldCoordinate[1];
		pPosition[2] = CenterWorldCoordinate[2] + radius * cos(angle);

		if (totalAngle > 0)
		{
			pVelocity[0] = -CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] =  CurrentVelocity * sin(angle);
		}
		else
		{
			pVelocity[0] =  CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] = -CurrentVelocity * sin(angle);
		}
		break;
	}

}

void ConverLinePath(ROCKS_PLANE &plane, ROCKS_POSE &pose, double &startPos1, double &endPose1, double &startPos2, double &endPose2 , double &CurrentDistance, double &CurrentVelocity, double *pPosition, double *pVelocity)
{
	double distance = sqrt((endPose1 - startPos1) * (endPose1 - startPos1) + (endPose2 - startPos2) * (endPose2 - startPos2));
	double rate1 = (endPose1 - startPos1) / distance;
	double rate2 = (endPose2 - startPos2) / distance;

	switch(plane)
	{
	case ROCKS_PLANE_XY:

		pPosition[0] = CurrentDistance * rate1 + startPos1;
		pPosition[1] = CurrentDistance * rate2 + startPos2;
		pPosition[2] = 0;

		pVelocity[0] = CurrentVelocity * rate1;
		pVelocity[1] = CurrentVelocity * rate2;
		pVelocity[2] = 0;
		break;

	case ROCKS_PLANE_YZ:

		pPosition[0] = 0;
		pPosition[1] = CurrentDistance * rate1 + startPos1;
		pPosition[2] = CurrentDistance * rate2 + startPos2;

		pVelocity[0] = 0;
		pVelocity[1] = CurrentVelocity * rate1;
		pVelocity[2] = CurrentVelocity * rate2;
		break;

	case ROCKS_PLANE_ZX:

		pPosition[0] = CurrentDistance * rate1 + startPos1;
		pPosition[1] = 0;
		pPosition[2] = CurrentDistance * rate2 + startPos2;

		pVelocity[0] = CurrentVelocity * rate1;
		pVelocity[1] = 0;
		pVelocity[2] = CurrentVelocity * rate2;
		break;

	default:
		break;
	}

	if (pose.r.x)
	{
		Roll(pPosition, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

void ConverLinePath(double *pStartPos, double *pEndPos, double &totalDistance, double &CurrentDistance, double &CurrentVelocity, double *pPosition, double *pVelocity)
{
	double rate_x = (pEndPos[0] - pStartPos[0]) / totalDistance;
	double rate_y = (pEndPos[1] - pStartPos[1]) / totalDistance;
	double rate_z = (pEndPos[2] - pStartPos[2]) / totalDistance;

	pPosition[0] = CurrentDistance * rate_x + pStartPos[0];
	pPosition[1] = CurrentDistance * rate_y + pStartPos[1];
	pPosition[2] = CurrentDistance * rate_z + pStartPos[2];

	pVelocity[0] = CurrentVelocity * rate_x;
	pVelocity[1] = CurrentVelocity * rate_y;
	pVelocity[2] = CurrentVelocity * rate_z;
}

uint32_t mix_Boundary = 0;
ROCKS_PLANE mix_plane = ROCKS_PLANE_XY;
ROCKS_POSE mix_pose;
ROCKS_MOVE_TYPE mix_moveType = ROCKS_MOVE_TYPE_LINEAR;
double mix_startPos1, mix_endPos1, mix_startPos2, mix_endPos2, mix_center1, mix_center2, mix_angle, mix_endPos_x, mix_endPos_y, mix_endPos_z;

void DeltaPath2WorldCoordinate(ROCKS_MECH* pMech, uint32_t &index, double *pPosition, double *pVelocity)
{
	if (pMech->var.moveType == ROCKS_MOVE_TYPE_CIRCULAR)//check the path type
	{
		ConvertCriclePath(pMech->var.startPos, pMech->var.angle, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index],pMech->var.plane, pMech->var.radius, pMech->var.center, pPosition, pVelocity);
	} 

	if (pMech->var.moveType == ROCKS_MOVE_TYPE_LINEAR)
	{
		ConverLinePath(pMech->var.startPos, pMech->var.endPos, pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines - 1], pMech->var.pPositionSplineBuffer[index],pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
	}

	if (pMech->var.moveType == ROCKS_MOVE_TYPE_MIX)
	{
		if (index == 0)
		{
			mix_Boundary = 0;
		}
		if (index == mix_Boundary)
		{
// 			if (pMech->var.pVelocitySplineBuffer[index] == 0 && pMech->var.pPositionSplineBuffer[index] == 0)//包尾
// 			{
// 				mix_Boundary = 0;
// 				index =  pMech->var.usedNrOfSplines;
// 				return;
// 			}

			mix_Boundary = pMech->var.pVelocitySplineBuffer[index + 3] + index +7;
			switch ((int)(pMech->var.pPositionSplineBuffer[index + 3]) / 256)
			{
			case 0://XY-PLANE
				mix_plane = ROCKS_PLANE_XY;
				break;
			case 1://YZ-PLANE
				mix_plane = ROCKS_PLANE_YZ;
				break;
			case 2://ZX-PLANE
				mix_plane = ROCKS_PLANE_ZX;
				break;
			default:
				break;
			}
			switch ((int)(pMech->var.pPositionSplineBuffer[index + 3]) % 256)
			{
			case 0://LINE
				mix_moveType = ROCKS_MOVE_TYPE_LINEAR;
				break;
			case 1://CRICLE
				mix_moveType = ROCKS_MOVE_TYPE_CIRCULAR;
				break;
			default:
				break;
			}
			mix_endPos_x = pMech->var.pVelocitySplineBuffer[index];
			mix_endPos_y = pMech->var.pPositionSplineBuffer[index];
			mix_endPos_z = pMech->var.pVelocitySplineBuffer[index + 1];
			mix_pose.r.x = pMech->var.pPositionSplineBuffer[index + 1];
			mix_pose.r.y = pMech->var.pPositionSplineBuffer[index + 2];
			mix_pose.r.z = pMech->var.pVelocitySplineBuffer[index + 2];
			mix_startPos1 = pMech->var.pVelocitySplineBuffer[index + 4];
			mix_startPos2 = pMech->var.pPositionSplineBuffer[index + 4];
			switch (mix_moveType)
			{
			case ROCKS_MOVE_TYPE_LINEAR:
				mix_endPos1 = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_endPos2 = pMech->var.pPositionSplineBuffer[index + 5];
				break;
			case ROCKS_MOVE_TYPE_CIRCULAR:
				mix_angle = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_center1 = pMech->var.pVelocitySplineBuffer[index + 6];
				mix_center2 = pMech->var.pPositionSplineBuffer[index + 6];
				break;
			default:
				break;
			}
			index += 7;
		}
		switch (mix_moveType)
		{
		case ROCKS_MOVE_TYPE_LINEAR:
			ConverLinePath(mix_plane, mix_pose, mix_startPos1, mix_endPos1, mix_startPos2, mix_endPos2, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
			break;
		case ROCKS_MOVE_TYPE_CIRCULAR:
			ConvertCriclePath(mix_plane, mix_pose, mix_startPos1, mix_startPos2, mix_center1, mix_center2, mix_angle, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
			break;
		default:
			break;
		}
		switch (mix_plane)
		{
		case ROCKS_PLANE_XY://XY-PLANE
			pPosition[2] = mix_endPos_z;
			break;
		case ROCKS_PLANE_YZ://YZ-PLANE
			pPosition[0] = mix_endPos_x;
			break;
		case ROCKS_PLANE_ZX://ZX-PLANE
			pPosition[1] = mix_endPos_y;
			break;
		default:
			break;
		}
	}

	if (pMech->var.refFramePose2.r.x)
	{
		Roll(pPosition, pMech->var.refFramePose2.r.x);
		Roll(pVelocity, pMech->var.refFramePose2.r.x);
	}

	if (pMech->var.refFramePose2.r.y)
	{
		Pitch(pPosition, pMech->var.refFramePose2.r.y);
		Pitch(pVelocity, pMech->var.refFramePose2.r.y);
	}

	if (pMech->var.refFramePose2.r.z)
	{
		Yaw(pPosition, pMech->var.refFramePose2.r.z);
		Yaw(pVelocity, pMech->var.refFramePose2.r.z);
	}

	pPosition[0] += pMech->var.refFramePose2.t.x;
	pPosition[1] += pMech->var.refFramePose2.t.y;
	pPosition[2] += pMech->var.refFramePose2.t.z;
} 

NYCE_STATUS RocksKinInverseDelta(ROCKS_MECH* pMech, const ROCKS_KIN_INV_PARS* pKin)
{
	uint32_t ax = 0;
	if (m_mech.var.jointBuffersAllocated)
	{
		int i;
		for (i = 0; i < ROCKS_MECH_MAX_NR_OF_JOINTS; i++)
		{
			if (m_mech.var.pJointPositionBufferC[i] != NULL)
			{
				free(m_mech.var.pJointPositionBufferC[i]);
				m_mech.var.pJointPositionBufferC[i] = NULL;
			}
			if (m_mech.var.pJointVelocityBufferC[i] != NULL)
			{
				free(m_mech.var.pJointVelocityBufferC[i]);
				m_mech.var.pJointVelocityBufferC[i] = NULL;
			}
		}
		m_mech.var.jointBuffersAllocated = FALSE;
	}

	while (!pMech->var.jointBuffersAllocated)
	{
		pMech->var.pJointPositionBufferC[ax] = (double*)malloc(pMech->var.maxNrOfSplines * sizeof(double));
		pMech->var.pJointVelocityBufferC[ax] = (double*)malloc(pMech->var.maxNrOfSplines * sizeof(double));
		if(++ax == ROCKS_MECH_MAX_NR_OF_JOINTS) 
		{
			pMech->var.jointBuffersAllocated = TRUE;
			pMech->var.pApplyForwardKinFunc = RocksKinForwardDelta;
			pMech->var.pApplyInverseKinFunc = RocksKinInverseDelta;
			pMech->var.pGetWorldSetpointPosFunc = RocksKinDeltaPosition;
		}
	}

	double pos_joint_x, pos_joint_y, pos_joint_z;//joint angle(JA)
	double vel_joint_x, vel_joint_y, vel_joint_z;//joint angular velocity(JAV)

	uint32_t realSegNum = 0;
	double (*pPosition)[3] = new double[pMech->var.maxNrOfSplines][3];
	double (*pVelocity)[3] = new double[pMech->var.maxNrOfSplines][3];

	for (uint32_t index = 0; index < pMech->var.usedNrOfSplines; ++index)
	{
		DeltaPath2WorldCoordinate(pMech, index, pPosition[realSegNum], pVelocity[realSegNum]);
		realSegNum++;
	}

	pMech->var.usedNrOfSplines = realSegNum - 1;

	for(uint32_t index = 0; index < realSegNum; ++index)
	{
		if(delta_calcInverse(pPosition[index][0], pPosition[index][1], pPosition[index][2], pos_joint_x, pos_joint_y, pos_joint_z) != 0)
			return ROCKS_ERR_NO_VALID_PATH;

		//convert the JA to joint position(JP)
		pMech->var.pJointPositionBufferC[0][index] = pos_joint_x * rate_angle2pu;
		pMech->var.pJointPositionBufferC[1][index] = pos_joint_y * rate_angle2pu;
		pMech->var.pJointPositionBufferC[2][index] = pos_joint_z * rate_angle2pu;

		if(delta_velInverse(pPosition[index][0], pPosition[index][1], pPosition[index][2], pVelocity[index][0], pVelocity[index][1], pVelocity[index][2], pos_joint_x, pos_joint_y, pos_joint_z, vel_joint_x, vel_joint_y, vel_joint_z) != 0)
			return ROCKS_ERR_NO_VALID_PATH;

		//convert the JAV to joint velocity(JV)
		pMech->var.pJointVelocityBufferC[0][index] = vel_joint_x * rate_angle2pu;
		pMech->var.pJointVelocityBufferC[1][index] = vel_joint_y * rate_angle2pu;
		pMech->var.pJointVelocityBufferC[2][index] = vel_joint_z * rate_angle2pu;
	}

	ofstream file("..//xyz&jointDatas.txt");	
	file<<m_mech.var.startPos[0]<< " "<<m_mech.var.startPos[1]<<" "<<m_mech.var.startPos[2]<<endl;
	file<<"|Index|x_pos|y_pos|z_pos|x_vel|y_vel|z_vel|joint1_pos|joint2_pos|joint3_pos|joint1_vel|joint2_vel|joint3_vel|"<<endl<<"|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|"<<endl;
	for (uint32_t i = 0; i < realSegNum; ++i)
	{
		file<<"|"<<i<<"|"<<pPosition[i][0]<<"|"<<pPosition[i][1]<<"|"<<pPosition[i][2]<<"|"<<pVelocity[i][0]<<"|"<<pVelocity[i][1]<<"|"<<pVelocity[i][2]<<"|"<<pMech->var.pJointPositionBufferC[0][i]<<"|"<<pMech->var.pJointPositionBufferC[1][i]<<"|"<<pMech->var.pJointPositionBufferC[2][i]<<"|"<<pMech->var.pJointVelocityBufferC[0][i]<<"|"<<pMech->var.pJointVelocityBufferC[1][i]<<"|"<<pMech->var.pJointVelocityBufferC[2][i]<<"|"<<endl;
	}
	file.close();

	delete []pPosition;
	delete []pVelocity;

	pMech->var.mechStep = ROCKS_MECH_STEP_VALID_INV_KINEMATICS;
	return NYCE_OK;
}

unsigned __stdcall ThreadRocksLoop(void* lpParam)
{
	NYCE_STATUS Status = NYCE_OK;

	while(!bRocksTerm)
	{

		WaitForSingleObject(hAuto, INFINITE);

		WaitForSingleObject(hAutoPathUsing,INFINITE);
		Status = NyceError( Status ) ? Status : RocksTrajLoadPath(&m_mech, &rocksTrajPath);
		ReleaseMutex(hAutoPathUsing);

		// Apply inverse kinematics to get joint splines
		// ---------------------------------------------
		for (int ax = 0; ax < NUM_AXES; ++ax)
		{
			kinPars.pJointPositionBuffer[ ax ] = NULL;
			kinPars.pJointVelocityBuffer[ ax ] = NULL;
		}
		Status = NyceError( Status ) ? Status : RocksKinInverseDelta( &m_mech, &kinPars );

		// Feed splines to the joints
		// --------------------------
		times++;
		cout<<"corrent times:"<<times<<endl;
		Status = NyceError( Status ) ? Status : RocksStream( &m_mech );	

		// Synchronize on motion complete                           
		// ------------------------------
		Status = NyceError( Status ) ? Status : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE);

		if (NyceError( Status ))
		{
			HandleError(Status, "Rocks");
			SetEvent(hEvStop);
			return 0;
		}
		timesCounter++;

	}	

	Status = NyceError( Status ) ? Status : RocksTrajDeletePath( &m_mech, &rocksTrajPath);

	if (m_mech.var.jointBuffersAllocated)
	{
		int i;
		for (i = 0; i < ROCKS_MECH_MAX_NR_OF_JOINTS; i++)
		{
			if (m_mech.var.pJointPositionBufferC[i] != NULL)
			{
				free(m_mech.var.pJointPositionBufferC[i]);
				m_mech.var.pJointPositionBufferC[i] = NULL;
			}
			if (m_mech.var.pJointVelocityBufferC[i] != NULL)
			{
				free(m_mech.var.pJointVelocityBufferC[i]);
				m_mech.var.pJointVelocityBufferC[i] = NULL;
			}
		}
	}

	// Delete mechanism
	// ----------------
	Status = NyceError( Status ) ? Status : RocksMechDelete( &m_mech );

	if (NyceError( Status ))
	{
		HandleError(Status, "Rocks");
	}
	return 0;
}

void RocksPathHandle()
{

	switch(pathType)
	{
	case 0://圆周路径
		// Get path splines
		// ----------------
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccPars );
		break;//case 0://圆周路径
	case 1://门形路径
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech,&segStartPars);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars1);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars1);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars2);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars2);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars3);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars3);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars4);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars4);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars5);

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech,&segStartPars);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars1);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars1);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars2);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars3);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars2);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars4);
		break;
	case 2://优化门型
		
	default:
		break;
	}

	ofstream file("..//TrajectoryDatas.txt");	
	file<<m_mech.var.startPos[0]<< " "<<m_mech.var.startPos[1]<<" "<<m_mech.var.startPos[2]<<endl;
	file<<"|Index|TCPP|TCVP|"<<endl<<"|:-:|:-:|:-:|"<<endl;
	for (uint32_t i = 0; i < m_mech.var.usedNrOfSplines; ++i)
	{
		file<<"|"<<i<<"|"<<m_mech.var.pPositionSplineBuffer[i]<<"|"<<m_mech.var.pVelocitySplineBuffer[i]<<"|"<<endl;
	}
	file.close();

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin( &m_mech, &rocksPose );

	WaitForSingleObject(hAutoPathUsing,INFINITE);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );
	ReleaseMutex(hAutoPathUsing);
}


const double SPLINE_TIME = 0.0002;

const double HOME_SPEED = 300;
const double HOME_OFFEST_X = 60;
const double HOME_OFFEST_Y = 0;
const double HOME_OFFEST_Z = 20;

const double CRICLE_SPEED = 500;
const double CRICLE_CENTER_OFFSET_1 = -60;
const double CRICLE_CENTER_OFFSET_2 = 0;
const double CRICLE_ANGLE = M_PI * 20;
const ROCKS_PLANE CRICLE_PLANE = ROCKS_PLANE_XY;

const double DOOR_SPEED = 500;
const double DOOR_HEIGHT = 20;
const double DOOR_WIDTH = 20;
const double DOOR_FILLET = 40;

const double OPT_DOOR_POINT_1[3] = {-65,0,-220};
const double OPT_DOOR_POINT_2[3] = {-25,0,-170};
const double OPT_DOOR_POINT_3[3] = { 25,0,-170};
const double OPT_DOOR_POINT_4[3] = { 65,0,-220};													

BOOL RocksGotoReadyPosition()
{
	ROCKS_TRAJ_SINE_ACC_PTP_PARS sinePtpPars;

	double readyTcp[3];
	double readyJp[3] = {160000,160000,160000};
	if(delta_calcForward(readyJp[0] / rate_angle2pu, readyJp[1] / rate_angle2pu, readyJp[2] / rate_angle2pu, readyTcp[0], readyTcp[1], readyTcp[2]) != 0)
		return FALSE;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sinePtpPars.startPos);
	sinePtpPars.endPos[0] = readyTcp[0] + HOME_OFFEST_X;
	sinePtpPars.endPos[1] = readyTcp[1] + HOME_OFFEST_Y;
	sinePtpPars.endPos[2] = readyTcp[2] + HOME_OFFEST_Z;
	sinePtpPars.endPos[3] = 0;
	sinePtpPars.endPos[4] = 0;
	sinePtpPars.endPos[5] = 0;
	sinePtpPars.maxVelocity = HOME_SPEED;
	sinePtpPars.maxAcceleration = HOME_SPEED * 10;
	sinePtpPars.splineTime = SPLINE_TIME;
	sinePtpPars.maxNrOfSplines = 0;
	sinePtpPars.pPositionSplineBuffer = NULL;
	sinePtpPars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccPtp(&m_mech,&sinePtpPars);

	for (int ax = 0; ax < NUM_AXES; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

	// Synchronize on motion complete
	// ------------------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE);

	if (NyceError( nyceStatus ))
	{
		HandleError(nyceStatus, "Rocks");
		SetEvent(hEvStop);
		return FALSE;
	}

	return TRUE;
}

BOOL RocksGotoReadyPosition(const double readyTcp[])
{
	ROCKS_TRAJ_SINE_ACC_PTP_PARS sinePtpPars;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sinePtpPars.startPos);
	sinePtpPars.endPos[0] = readyTcp[0];
	sinePtpPars.endPos[1] = readyTcp[1];
	sinePtpPars.endPos[2] = readyTcp[2];
	sinePtpPars.endPos[3] = 0;
	sinePtpPars.endPos[4] = 0;
	sinePtpPars.endPos[5] = 0;
	sinePtpPars.maxVelocity = HOME_SPEED;
	sinePtpPars.maxAcceleration = HOME_SPEED * 10;
	sinePtpPars.splineTime = SPLINE_TIME;
	sinePtpPars.maxNrOfSplines = 0;
	sinePtpPars.pPositionSplineBuffer = NULL;
	sinePtpPars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccPtp(&m_mech,&sinePtpPars);

	for (int ax = 0; ax < NUM_AXES; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

	// Synchronize on motion complete
	// ------------------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE);

	if (NyceError( nyceStatus ))
	{
		HandleError(nyceStatus, "Rocks");
		SetEvent(hEvStop);
		return FALSE;
	}

	return TRUE;
}

BOOL Rocks(void)
{
	// Create mechanism
	// ----------------
	m_mech.nrOfJoints = NUM_AXES; // X1, X2, Y and Z
	m_mech.dof[ 0 ]	  = TRUE;     // X
	m_mech.dof[ 1 ]	  = TRUE;     // Y
	m_mech.dof[ 2 ]	  = TRUE;     // Z
	m_mech.dof[ 3 ]	  = FALSE;    // Rx
	m_mech.dof[ 4 ]	  = FALSE;    // Ry
	m_mech.dof[ 5 ]	  = FALSE;    // Rz
	for ( int ax = 0; ax < NUM_AXES; ax++ )
	{
		m_mech.jointAxisId[ ax ] = axId[ ax ];
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechCreate( &m_mech );

	if (pathType == 0)
	{
		if (!RocksGotoReadyPosition())
			return FALSE;
	}
	else
	{
		if (!RocksGotoReadyPosition(OPT_DOOR_POINT_1))
			return FALSE;
	}

	// Get current position
	// --------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPars.startPos);

	// Define the circle
	// -----------------
	sineAccPars.maxVelocity = CRICLE_SPEED;
	sineAccPars.maxAcceleration = CRICLE_SPEED * 10;
	sineAccPars.splineTime = SPLINE_TIME;
	sineAccPars.center[ 0 ] = sineAccPars.startPos[0] + CRICLE_CENTER_OFFSET_1;
	sineAccPars.center[ 1 ] = sineAccPars.startPos[1] + CRICLE_CENTER_OFFSET_2;
	sineAccPars.angle = CRICLE_ANGLE;
	sineAccPars.plane = CRICLE_PLANE;
	sineAccPars.maxNrOfSplines = 0;
	sineAccPars.pPositionSplineBuffer = NULL;
	sineAccPars.pVelocitySplineBuffer = NULL;

	// Define the rectangle
	// -----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, segStartPars.startPos);

	segStartPars.splineTime = SPLINE_TIME;
	segStartPars.maxNrOfSplines = 0;
	segStartPars.pPositionSplineBuffer = NULL;
	segStartPars.pVelocitySplineBuffer = NULL;

	// 	segLinePars1.plane = ROCKS_PLANE_ZX;
	// 	segLinePars1.endPos[0] = segStartPars.startPos[0] - 10;
	// 	segLinePars1.endPos[1] = segStartPars.startPos[2] - 10;
	// 	segLinePars1.endVelocity = DOOR_SPEED;
	// 	segLinePars1.maxAcceleration = DOOR_SPEED * 10;
	// 
	// 	segLinePars2.plane = ROCKS_PLANE_ZX;
	// 	segLinePars2.endPos[0] = segLinePars1.endPos[0] - 10;
	// 	segLinePars2.endPos[1] = segLinePars1.endPos[1] - 10;
	// 	segLinePars2.endVelocity = 0;
	// 	segLinePars2.maxAcceleration = DOOR_SPEED * 10;
	// 
	// 	segLinePars3.plane = ROCKS_PLANE_ZX;
	// 	segLinePars3.endPos[0] = segLinePars2.endPos[0] + 10;
	// 	segLinePars3.endPos[1] = segLinePars2.endPos[1] + 10;
	// 	segLinePars3.endVelocity = DOOR_SPEED;
	// 	segLinePars3.maxAcceleration = DOOR_SPEED * 10;
	// 
	// 	segLinePars4.plane = ROCKS_PLANE_ZX;
	// 	segLinePars4.endPos[0] = segLinePars3.endPos[0] + 10;
	// 	segLinePars4.endPos[1] = segLinePars3.endPos[1] + 10;
	// 	segLinePars4.endVelocity = 0;
	// 	segLinePars4.maxAcceleration = DOOR_SPEED * 10;


	//door1
// 	segLinePars1.plane = ROCKS_PLANE_ZX;
// 	segLinePars1.endPos[0] = segStartPars.startPos[0] ;
// 	segLinePars1.endPos[1] = segStartPars.startPos[2] - DOOR_HEIGHT;
// 	segLinePars1.endVelocity = DOOR_SPEED;
// 	segLinePars1.maxAcceleration = DOOR_SPEED * 10;
// 
// 	segArcPars1.plane = ROCKS_PLANE_ZX;
// 	segArcPars1.center[0] = segLinePars1.endPos[0] - DOOR_FILLET;
// 	segArcPars1.center[1] = segLinePars1.endPos[1];
// 	segArcPars1.endPos[0] = segLinePars1.endPos[0] - DOOR_FILLET;
// 	segArcPars1.endPos[1] = segLinePars1.endPos[1] - DOOR_FILLET;
// 	segArcPars1.endVelocity = DOOR_SPEED;
// 	segArcPars1.maxAcceleration = DOOR_SPEED * 10;
// 	segArcPars1.positiveAngle = TRUE;
// 
// 	segLinePars2.plane = ROCKS_PLANE_ZX;
// 	segLinePars2.endPos[0] = segArcPars1.endPos[0] - DOOR_WIDTH;
// 	segLinePars2.endPos[1] = segArcPars1.endPos[1];
// 	segLinePars2.endVelocity = DOOR_SPEED;
// 	segLinePars2.maxAcceleration = DOOR_SPEED * 10;
// 
// 	segArcPars2.plane = ROCKS_PLANE_ZX;
// 	segArcPars2.center[0] = segLinePars2.endPos[0] ;
// 	segArcPars2.center[1] = segLinePars2.endPos[1] + DOOR_FILLET;
// 	segArcPars2.endPos[0] = segLinePars2.endPos[0] - DOOR_FILLET;
// 	segArcPars2.endPos[1] = segLinePars2.endPos[1] + DOOR_FILLET;
// 	segArcPars2.endVelocity = DOOR_SPEED;
// 	segArcPars2.maxAcceleration = DOOR_SPEED * 10;
// 	segArcPars2.positiveAngle = TRUE;
// 
// 	segLinePars3.plane = ROCKS_PLANE_ZX;
// 	segLinePars3.endPos[0] = segArcPars2.endPos[0];
// 	segLinePars3.endPos[1] = segArcPars2.endPos[1] + DOOR_HEIGHT;
// 	segLinePars3.endVelocity = DOOR_SPEED;
// 	segLinePars3.maxAcceleration = DOOR_SPEED * 10;
// 
// 	segArcPars3.plane = ROCKS_PLANE_ZX;
// 	segArcPars3.center[0] = segLinePars3.endPos[0] + DOOR_FILLET;
// 	segArcPars3.center[1] = segLinePars3.endPos[1];
// 	segArcPars3.endPos[0] = segLinePars3.endPos[0] + DOOR_FILLET;
// 	segArcPars3.endPos[1] = segLinePars3.endPos[1] + DOOR_FILLET;
// 	segArcPars3.endVelocity = DOOR_SPEED;
// 	segArcPars3.maxAcceleration = DOOR_SPEED * 10;
// 	segArcPars3.positiveAngle = TRUE;
// 
// 	segLinePars4.plane = ROCKS_PLANE_ZX;
// 	segLinePars4.endPos[0] = segArcPars3.endPos[0] + DOOR_WIDTH;
// 	segLinePars4.endPos[1] = segArcPars3.endPos[1];
// 	segLinePars4.endVelocity = DOOR_SPEED;
// 	segLinePars4.maxAcceleration = DOOR_SPEED * 10;
// 
// 	segArcPars4.plane = ROCKS_PLANE_ZX;
// 	segArcPars4.center[0] = segLinePars4.endPos[0];
// 	segArcPars4.center[1] = segLinePars4.endPos[1] - DOOR_FILLET;
// 	segArcPars4.endPos[0] = segLinePars4.endPos[0] + DOOR_FILLET;
// 	segArcPars4.endPos[1] = segLinePars4.endPos[1] - DOOR_FILLET;
// 	segArcPars4.endVelocity = 0;
// 	segArcPars4.maxAcceleration = DOOR_SPEED * 10;
// 	segArcPars4.positiveAngle = TRUE;

	//door2
	double line = sqrt((-138.75 - OPT_DOOR_POINT_2[2]) * (-138.75 - OPT_DOOR_POINT_2[2]) + OPT_DOOR_POINT_2[0] * OPT_DOOR_POINT_2[0]);
	double z = -(138.75 + line * sqrt(4100.0) / 50);
	double center[3] = {0,0,z};

	segLinePars1.plane = ROCKS_PLANE_ZX;
	segLinePars1.endPos[0] = OPT_DOOR_POINT_2[0];
	segLinePars1.endPos[1] = OPT_DOOR_POINT_2[2];
	segLinePars1.endVelocity = DOOR_SPEED;
	segLinePars1.maxAcceleration = DOOR_SPEED * 10;

	segArcPars1.plane = ROCKS_PLANE_ZX;
	segArcPars1.center[0] = center[0];
	segArcPars1.center[1] = center[2];
	segArcPars1.endPos[0] = OPT_DOOR_POINT_3[0];
	segArcPars1.endPos[1] = OPT_DOOR_POINT_3[2];
	segArcPars1.endVelocity = DOOR_SPEED;
	segArcPars1.maxAcceleration = DOOR_SPEED * 10;
	segArcPars1.positiveAngle = TRUE;

	segLinePars2.plane = ROCKS_PLANE_ZX;
	segLinePars2.endPos[0] = OPT_DOOR_POINT_4[0];
	segLinePars2.endPos[1] = OPT_DOOR_POINT_4[2];
	segLinePars2.endVelocity = 0;
	segLinePars2.maxAcceleration = DOOR_SPEED * 10;

	segLinePars3.plane = ROCKS_PLANE_ZX;
	segLinePars3.endPos[0] = OPT_DOOR_POINT_3[0];
	segLinePars3.endPos[1] = OPT_DOOR_POINT_3[2];
	segLinePars3.endVelocity = DOOR_SPEED;
	segLinePars3.maxAcceleration = DOOR_SPEED * 10;

	segArcPars2.plane = ROCKS_PLANE_ZX;
	segArcPars2.center[0] = center[0];
	segArcPars2.center[1] = center[2];
	segArcPars2.endPos[0] = OPT_DOOR_POINT_2[0];
	segArcPars2.endPos[1] = OPT_DOOR_POINT_2[2];
	segArcPars2.endVelocity = DOOR_SPEED;
	segArcPars2.maxAcceleration = DOOR_SPEED * 10;
	segArcPars2.positiveAngle = FALSE;

	segLinePars4.plane = ROCKS_PLANE_ZX;
	segLinePars4.endPos[0] = OPT_DOOR_POINT_1[0];
	segLinePars4.endPos[1] = OPT_DOOR_POINT_1[2];
	segLinePars4.endVelocity = 0;
	segLinePars4.maxAcceleration = DOOR_SPEED * 10;

	// 	segLinePars5.plane = ROCKS_PLANE_ZX;
	// 	segLinePars5.endPos[0] = segArcPars4.endPos[0];
	// 	segLinePars5.endPos[1] = segArcPars4.endPos[1] - 10;
	// 	segLinePars5.endVelocity = 0;
	// 	segLinePars5.maxAcceleration = TCP_ACC;

	// Define the transition matrix 
	// -----------------
	rocksPose.r.x = 0 ;
	rocksPose.r.y = 0;
	rocksPose.r.z = 0;

	double buffer[3];
	buffer[0] = sineAccPars.startPos[0]; 
	buffer[1] = sineAccPars.startPos[1];
	buffer[2] = sineAccPars.startPos[2];
	Roll(buffer,rocksPose.r.x);

	rocksPose.t.x = sineAccPars.startPos[0] - buffer[0]; 
	rocksPose.t.y = sineAccPars.startPos[1] - buffer[1];
	rocksPose.t.z = sineAccPars.startPos[2] - buffer[2];

	RocksPathHandle();

	if (NyceError( nyceStatus ))
	{
		HandleError("Rocks");
		return FALSE;
	}

	return TRUE;
}

void OnInterpolantEvent( NYCE_ID nyceId, NYCE_EVENT eventId, NYCE_EVENT_DATA *pEventData, void *pUserData )
{
	NYCE_STATUS myStatus = NYCE_OK;
	double cartesianPos[ROCKS_MECH_MAX_DOF];
	myStatus = NyceError(myStatus) ? myStatus : RocksKinDeltaPosition(&m_mech, cartesianPos);
	if (NyceError(myStatus))
	{
		HandleError(myStatus, "RocksKinDeltaPosition");
		SetEvent(hEvStop);
		return;
	}
	pDrawer->AddTcpPoint(cartesianPos);
}

int programState = 0;
uint32_t varIndex = 0;
unsigned uThreadRocks = 0;
double postion[3];

void StateHandle()
{
	switch(programState)
	{
	case 0:
		nyceStatus = NYCE_OK;	
		printf("Begin...\n");
#ifdef SIM_METHOD
		nyceStatus = NyceInit(NYCE_SIM);
#else
		nyceStatus = NyceInit(NYCE_NET);
#endif // SIM_METHOD

		if (NyceError(nyceStatus))
		{
			HandleError("System");

			programState = 4;
			break;
		}

#ifdef USE_NODE
		if (!InitNode())
		{
			programState = 4;
			break;
		}
#endif // USE_NODE

#ifdef USE_AXIS
		if (!InitAxis())
		{
			programState = 4;
			break;
		}
#endif // USE_AXIS


#ifdef NT
		Sleep(500);//Some trouble will be happened without this code while using the simulation system.
		//--For example, the error "spline buffer empty" occur.
#endif // NT

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDefineEventEnrolment(axId[0], SAC_EV_INTERPOLANT_STARTED, OnInterpolantEvent, NULL);
		if (NyceError(nyceStatus)) 
		{
			programState = 4;
			break;
		}

		for (int ax = 0; ax <NUM_AXES; ++ax)
		{
			nyceStatus = SacAddVariableToSet(axId[ax], SAC_VAR_SETPOINT_POS, &posVarIndex[ax]);
			if (NyceError(nyceStatus))
			{
				HandleError(axName[ax]);
				programState = 3;
				break;
			}
		}

		hAutoPathUsing = CreateMutex(NULL,NULL,NULL);

		if (!Rocks()) 
		{
			programState = 3;
			break;
		}

		hAuto = CreateEvent(NULL,TRUE,FALSE,NULL);

		hThreadRocks = (HANDLE)_beginthreadex(NULL, NULL, ThreadRocksLoop, NULL, 0,&uThreadRocks);

		hEvStop = CreateEvent(NULL,TRUE,FALSE,NULL);
		Sleep(10);

		programState = 1;

	case 1:
		break;

	case 2:
		CloseHandle(hEvStop);
		CloseHandle(hAuto);
		CloseHandle(hAutoPathUsing);

		bRocksTerm = TRUE;
		cout<<"Wait for Rocks stop...\n"<<endl;
		WaitForSingleObject(hThreadRocks, INFINITE);
		CloseHandle(hThreadRocks);

	case 3:
		for (int ax = 0; posVarIndex[ax] >0; ax++)
		{
			nyceStatus = SacDeleteVariableFromSet(posVarIndex[ax]);
			if (NyceError(nyceStatus))
			{
				HandleError(axName[ax]);
			}
		}

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDeleteEventEnrolment(axId[0], SAC_EV_INTERPOLANT_STARTED, OnInterpolantEvent, NULL);

	case 4:
#ifdef USE_NODE
		TermNode();
#endif // USE_NODE
#ifdef USE_AXIS
		TermAxis();
#endif // USE_AXIS

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();
		if (NyceError(nyceStatus))
		{
			HandleError("Host");
		}
		printf("End.\n");
		break; //case 4

	case 5://自动开关
		if (WaitForSingleObject(hAuto,0) == WAIT_OBJECT_0 )
		{
			ResetEvent(hAuto);
		}
		else
		{
			SetEvent(hAuto);
		}
		programState = 1;
		break;

	case 6://回零
		programState = 1;
		break;

	case 7://切换到无限圆周运动
		pathType = 0;
		RocksPathHandle();
		programState = 1;
		break;

	case 8://切换到无限门型运动
		pathType = 1;
		RocksPathHandle();
		programState = 1;
		break;

	default:
		break;
	}


} 

int _tmain(int argc, char* argv[])
{
	Drawer drawer(target, coordinate, StateHandle, &programState);
	pDrawer = &drawer;
	drawer.StartUp(argc, argv);

// 	programState = 0;
// 	StateHandle();
// 	programState = 5;
// 	StateHandle();

	system("pause");
	return 0;
}

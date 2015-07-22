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
#define SIM_METHOD

//const double rate_angle2pu = 0.5 / M_PI * 10000;
const double rate_angle2pu = 131072 * 11 / (2 * M_PI);

NYCE_STATUS nyceStatus;
HANDLE hEvStop;
HANDLE hAuto;

HANDLE hAutoPathUsing;
int pathType = 0;

LONG64 times = 0;
LONG64 timesCounter = 0;

double coordinate[6] = {0, 200, -535, -335, -100, 100};
double target[3] = {(coordinate[1] + coordinate[0]) / 2, (coordinate[3] + coordinate[2]) / 2, (coordinate[5] + coordinate[4]) / 2};

Drawer *pDrawer;

ROCKS_TRAJ_SINE_ACC_PTP_PARS sinePtpPars;
ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccPars;
ROCKS_TRAJ_SEGMENT_START_PARS segStartPars;
ROCKS_TRAJ_SEGMENT_LINE_PARS segLinePars1,segLinePars2,segLinePars3,segLinePars4,segLinePars5;
ROCKS_TRAJ_SEGMENT_ARC_PARS segArcPars1,segArcPars2,segArcPars3,segArcPars4;

void HandleError(const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusString(nyceStatus)<<"\n"<<endl;
	nyceStatus = NYCE_OK;
}

void HandleError(NYCE_STATUS &Status, const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusString(Status)<<"\n"<<endl;
	cout<<"Run times:"<<timesCounter<<endl;
	Status = NYCE_OK;
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
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacGetAxisConfiguration( axId[ ax ], &axisPars );
				
				if ( NyceSuccess(nyceStatus) && axisPars.motorType == SAC_BRUSHLESS_AC_MOTOR )
				{
					nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacAlignMotor( axId[ ax ] );
					nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_ALIGN_MOTOR, 10 );
				}
			case SAC_FREE:
				

				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacLock( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_LOCK, 10 );

				//回零是危险操作，若回零会使单轴运动则要禁止
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacHome( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_HOMING_COMPLETED, 10 );

				

				
				break;
// 			case SAC_MOVING:
// 				printf("Waiting the motion stop...");
// 				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_MOTION_STOPPED, 30 );
// 				break;
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
ROCKS_KIN_INV_PARS kinPars;
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
	status = NyceError(status) ? status : NyceReadVariableSet(posVarIndex[0], posVarIndex[NUM_AXES - 1], pJointPos);
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

void DeltaPath2WorldCoordinate(ROCKS_MECH* pMech, uint32_t index, double *pPosition, double *pVelocity)
{
	if (pMech->var.moveType == ROCKS_MOVE_TYPE_CIRCULAR)//check the path type
	{
		double angle;
		double alhpa;
		double beta ;
		double CenterWorldCoordinate[3];
		switch(pMech->var.plane)
		{
		case ROCKS_PLANE_XY:
			//这个只需计算一次！！！
			alhpa = acos((pMech->var.startPos[0] - pMech->var.center[0]) / sqrt((pMech->var.startPos[0] - pMech->var.center[0]) * (pMech->var.startPos[0] - pMech->var.center[0]) + (pMech->var.startPos[1] - pMech->var.center[1]) * (pMech->var.startPos[1] - pMech->var.center[1])));
			beta = pMech->var.center[1] < 0 ? alhpa : M_PI * 2 - alhpa;

			if (pMech->var.angle > 0)
				angle = beta - pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;
			else
				angle = beta + pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;

			CenterWorldCoordinate[0] = pMech->var.center[0];
			CenterWorldCoordinate[1] = pMech->var.center[1];
			CenterWorldCoordinate[2] = pMech->var.startPos[2];

			pPosition[0] = CenterWorldCoordinate[0] + pMech->var.radius * cos(angle);
			pPosition[1] = CenterWorldCoordinate[1] + pMech->var.radius * sin(angle);
			pPosition[2] = CenterWorldCoordinate[2];

			if (pMech->var.angle > 0)
			{
				pVelocity[0] =  pMech->var.pVelocitySplineBuffer[index] * sin(angle);
				pVelocity[1] = -pMech->var.pVelocitySplineBuffer[index] * cos(angle);
				pVelocity[2] =  0;
			}
			else
			{
				pVelocity[0] = -pMech->var.pVelocitySplineBuffer[index] * sin(angle);
				pVelocity[1] =  pMech->var.pVelocitySplineBuffer[index] * cos(angle);
				pVelocity[2] =  0;
			}
			break;
		case ROCKS_PLANE_YZ:
			alhpa = acos((pMech->var.startPos[1] - pMech->var.center[0]) / sqrt((pMech->var.startPos[1] - pMech->var.center[0]) * (pMech->var.startPos[1] - pMech->var.center[0]) + (pMech->var.startPos[2] - pMech->var.center[1]) * (pMech->var.startPos[2] - pMech->var.center[1])));
			beta = pMech->var.center[1] < 0 ? alhpa : M_PI * 2 - alhpa;
			if (pMech->var.angle > 0)
				angle = beta - pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;
			else
				angle = beta + pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;

			CenterWorldCoordinate[0] = pMech->var.startPos[0];
			CenterWorldCoordinate[1] = pMech->var.center[0];
			CenterWorldCoordinate[2] = pMech->var.center[1];

			pPosition[0] = CenterWorldCoordinate[0];
			pPosition[1] = CenterWorldCoordinate[1] + pMech->var.radius * cos(angle);
			pPosition[2] = CenterWorldCoordinate[2] + pMech->var.radius * sin(angle);

			if (pMech->var.angle > 0)
			{
				pVelocity[0] =  0;
				pVelocity[1] =  pMech->var.pVelocitySplineBuffer[index] * sin(angle);
				pVelocity[2] = -pMech->var.pVelocitySplineBuffer[index] * cos(angle);
			}
			else
			{
				pVelocity[0] =  0;
				pVelocity[1] = -pMech->var.pVelocitySplineBuffer[index] * sin(angle);
				pVelocity[2] =  pMech->var.pVelocitySplineBuffer[index] * cos(angle);
			} 
			break;
		case ROCKS_PLANE_ZX:
			alhpa = acos((pMech->var.startPos[2] - pMech->var.center[0]) / sqrt((pMech->var.startPos[2] - pMech->var.center[0]) * (pMech->var.startPos[2] - pMech->var.center[0]) + (pMech->var.startPos[0] - pMech->var.center[1]) * (pMech->var.startPos[0] - pMech->var.center[1])));
			beta = pMech->var.center[1] < 0 ? alhpa : M_PI * 2 - alhpa;
			if (pMech->var.angle > 0)
				angle = beta - pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;
			else
				angle = beta + pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;

			CenterWorldCoordinate[0] = pMech->var.center[1];
			CenterWorldCoordinate[1] = pMech->var.startPos[1];
			CenterWorldCoordinate[2] = pMech->var.center[0];

			pPosition[0] = CenterWorldCoordinate[0] + pMech->var.radius * sin(angle);
			pPosition[1] = CenterWorldCoordinate[1];
			pPosition[2] = CenterWorldCoordinate[2] + pMech->var.radius * cos(angle);

			if (pMech->var.angle > 0)
			{
				pVelocity[0] = -pMech->var.pVelocitySplineBuffer[index] * cos(angle);
				pVelocity[1] =  0;
				pVelocity[2] =  pMech->var.pVelocitySplineBuffer[index] * sin(angle);
			}
			else
			{
				pVelocity[0] =  pMech->var.pVelocitySplineBuffer[index] * cos(angle);
				pVelocity[1] = 0;
				pVelocity[2] = -pMech->var.pVelocitySplineBuffer[index] * sin(angle);
			}
			break;
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

// 	if (pMech->var.moveType == ROCKS_MOVE_TYPE_LINEAR)
// 	{
// 		double distance = m_mech.var.pPositionSplineBuffer[m_mech.var.usedNrOfSplines - 1];
// 		double rate_x = (m_mech.var.endPos[0] - m_mech.var.startPos[0]) /  m_mech.var.pPositionSplineBuffer[m_mech.var.usedNrOfSplines - 1];
// 		double rate_y = (m_mech.var.endPos[1] - m_mech.var.startPos[1]) /  m_mech.var.pPositionSplineBuffer[m_mech.var.usedNrOfSplines - 1];
// 		double rate_z = (m_mech.var.endPos[2] - m_mech.var.startPos[2]) /  m_mech.var.pPositionSplineBuffer[m_mech.var.usedNrOfSplines - 1];
// 		
// 		pPosition[0] = pMech->var.pPositionSplineBuffer[index] * rate_x + m_mech.var.startPos[0];
// 		pPosition[1] = pMech->var.pPositionSplineBuffer[index] * rate_y + m_mech.var.startPos[1];
// 		pPosition[2] = pMech->var.pPositionSplineBuffer[index] * rate_z + m_mech.var.startPos[2];
// 																	   
// 		pVelocity[0] = pMech->var.pVelocitySplineBuffer[index] * rate_x;
// 		pVelocity[1] = pMech->var.pVelocitySplineBuffer[index] * rate_y;
// 		pVelocity[2] = pMech->var.pVelocitySplineBuffer[index] * rate_z;
// 	}
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
		pMech->var.pJointPositionBufferC[ax] = (double*)malloc(sizeof(double) * pMech->var.maxNrOfSplines);
		pMech->var.pJointVelocityBufferC[ax] = (double*)malloc(sizeof(double) * pMech->var.maxNrOfSplines);
		if(++ax == pMech->nrOfJoints) 
		{
			pMech->var.jointBuffersAllocated = TRUE;
			pMech->var.pGetWorldSetpointPosFunc = RocksKinDeltaPosition;
			pMech->var.pApplyForwardKinFunc = RocksKinForwardDelta;
			pMech->var.pApplyInverseKinFunc = RocksKinInverseDelta;
		}
	}

	for (uint32_t index = 0; index < pMech->var.usedNrOfSplines; ++index)
	{
		double pos[3],vel[3];
		double pos_joint_x, pos_joint_y, pos_joint_z;//joint angle(JA)
		double vel_joint_x, vel_joint_y, vel_joint_z;//joint angular velocity(JAV)

		DeltaPath2WorldCoordinate(pMech, index, pos, vel);

		delta_calcInverse(pos[0], pos[1], pos[2], pos_joint_x, pos_joint_y, pos_joint_z);

		//convert the JA to joint position(JP)
		pMech->var.pJointPositionBufferC[0][index] = pos_joint_x * rate_angle2pu;
		pMech->var.pJointPositionBufferC[1][index] = pos_joint_y * rate_angle2pu;
		pMech->var.pJointPositionBufferC[2][index] = pos_joint_z * rate_angle2pu;

		delta_velInverse(pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], pos_joint_x, pos_joint_y, pos_joint_z, vel_joint_x, vel_joint_y, vel_joint_z);

		//convert the JAV to joint velocity(JV)
		pMech->var.pJointVelocityBufferC[0][index] = vel_joint_x * rate_angle2pu;
		pMech->var.pJointVelocityBufferC[1][index] = vel_joint_y * rate_angle2pu;
		pMech->var.pJointVelocityBufferC[2][index] = vel_joint_z * rate_angle2pu;

// 		if (index == 0 || index == pMech->var.usedNrOfSplines - 1)
// 		{
// 			pMech->var.pJointPositionBufferC[0][index] = (double)(int)(pMech->var.pJointPositionBufferC[0][index] * 100000) / 100000;
// 			pMech->var.pJointPositionBufferC[1][index] = (double)(int)(pMech->var.pJointPositionBufferC[1][index] * 100000) / 100000;
// 			pMech->var.pJointPositionBufferC[2][index] = (double)(int)(pMech->var.pJointPositionBufferC[2][index] * 100000) / 100000;
// 
// 			pMech->var.pJointVelocityBufferC[0][index] = (double)(int)(pMech->var.pJointVelocityBufferC[0][index] * 100000) / 100000;
// 			pMech->var.pJointVelocityBufferC[1][index] = (double)(int)(pMech->var.pJointVelocityBufferC[1][index] * 100000) / 100000;
// 			pMech->var.pJointVelocityBufferC[2][index] = (double)(int)(pMech->var.pJointVelocityBufferC[2][index] * 100000) / 100000;
// 		}
	}
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

		ofstream file("..//SplineDatas.txt");	
		file<<m_mech.var.startPos[0]<< " "<<m_mech.var.startPos[1]<<" "<<m_mech.var.startPos[2]<<endl;
		file<<"|Index|TCPP|TCVP|A1PJ|A1VJ|A2PJ|A2VJ|A3PJ|A3VJ"<<endl<<"|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:"<<endl;
		for (uint32_t i = 0; i < m_mech.var.usedNrOfSplines; ++i)
		{
			file<<"|"<<i<<"|"<<m_mech.var.pPositionSplineBuffer[i]<<"|"<<m_mech.var.pVelocitySplineBuffer[i]<<"|"<<m_mech.var.pJointPositionBufferC[0][i]<<"|"<<m_mech.var.pJointVelocityBufferC[0][i]<<"|"<<m_mech.var.pJointPositionBufferC[1][i]<<"|"<<m_mech.var.pJointVelocityBufferC[1][i]<<"|"<<m_mech.var.pJointPositionBufferC[2][i]<<"|"<<m_mech.var.pJointVelocityBufferC[2][i]<<endl;
		}
		file.close();

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
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech,&segStartPars);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars1);
//		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars1);
 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars2);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars2);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars3);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars3);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars4);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars4);
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars5);
		break;

	default:
		break;
	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin( &m_mech, &rocksPose );

	WaitForSingleObject(hAutoPathUsing,INFINITE);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );
	ReleaseMutex(hAutoPathUsing);
}

const double TCP_SPEED = 100;
const double TCP_ACC = 1000;
const double SPLINE_TIME = 0.01;

BOOL RocksGotoReadyPosition()
{
	double readyTcp[3];
	double readyJp[3] = {160000,160000,160000};
	if(delta_calcForward(readyJp[0] / rate_angle2pu, readyJp[1] / rate_angle2pu, readyJp[2] / rate_angle2pu, readyTcp[0], readyTcp[1], readyTcp[2]) != 0)
		return FALSE;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sinePtpPars.startPos);
	sinePtpPars.endPos[0] = readyTcp[0];
	sinePtpPars.endPos[1] = readyTcp[1];
	sinePtpPars.endPos[2] = readyTcp[2];
	sinePtpPars.endPos[3] = 0;
	sinePtpPars.endPos[4] = 0;
	sinePtpPars.endPos[5] = 0;
	sinePtpPars.maxVelocity = 1000;
	sinePtpPars.maxAcceleration = 10000;
	sinePtpPars.splineTime = 0.001;
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

// 	if (!RocksGotoReadyPosition())
// 		return FALSE;
	
	// Get current position
	// --------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPars.startPos);

	// Define the circle
	// -----------------
	sineAccPars.maxVelocity = TCP_SPEED;
	sineAccPars.maxAcceleration = TCP_ACC;
	sineAccPars.splineTime = SPLINE_TIME;
	sineAccPars.center[ 0 ] = sineAccPars.startPos[0] + 10;
	sineAccPars.center[ 1 ] = sineAccPars.startPos[1];
	sineAccPars.angle = M_PI * 8;
	sineAccPars.plane = ROCKS_PLANE_XY;
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
	
	segLinePars1.plane = ROCKS_PLANE_XY;
	segLinePars1.endPos[0] = segStartPars.startPos[0] - 20;
	segLinePars1.endPos[1] = segStartPars.startPos[1];
	segLinePars1.endVelocity = TCP_SPEED;
	segLinePars1.maxAcceleration = TCP_ACC;

// 	segArcPars1.plane = ROCKS_PLANE_XY;
// 	segArcPars1.center[0] = segLinePars1.endPos[0];
// 	segArcPars1.center[1] = segLinePars1.endPos[1] - 10;
// 	segArcPars1.endPos[0] = segLinePars1.endPos[0] - 10;
// 	segArcPars1.endPos[1] = segLinePars1.endPos[1] - 10;
// 	segArcPars1.endVelocity = 0;
// 	segArcPars1.maxAcceleration = TCP_ACC ;
// 	segArcPars1.positiveAngle = TRUE;
// 	segArcPars1.originOffset.r.x = 0;
// 	segArcPars1.originOffset.r.y = 0;
// 	segArcPars1.originOffset.r.z = 0;
// 	segArcPars1.originOffset.t.x = 0;
// 	segArcPars1.originOffset.t.y = 0;
// 	segArcPars1.originOffset.t.z = 0;

	segLinePars2.plane = ROCKS_PLANE_XY;
	segLinePars2.endPos[0] = segLinePars1.endPos[0] - 20;
	segLinePars2.endPos[1] = segLinePars1.endPos[1];
	segLinePars2.endVelocity = 0;
	segLinePars2.maxAcceleration = TCP_ACC; 

// 	segArcPars2.plane = ROCKS_PLANE_ZX;
// 	segArcPars2.center[0] = segLinePars2.endPos[0];
// 	segArcPars2.center[1] = segLinePars2.endPos[1] + 2;
// 	segArcPars2.endPos[0] = segLinePars2.endPos[0] - 2;
// 	segArcPars2.endPos[1] = segLinePars2.endPos[1] + 2;
// 	segArcPars2.endVelocity = TCP_SPEED;
// 	segArcPars2.maxAcceleration = TCP_ACC ;
// 	segArcPars2.positiveAngle = TRUE;
// 
// 	segLinePars3.plane = ROCKS_PLANE_ZX;
// 	segLinePars3.endPos[0] = segArcPars2.endPos[0];
// 	segLinePars3.endPos[1] = segArcPars2.endPos[1] + 20;
// 	segLinePars3.endVelocity = TCP_SPEED;
// 	segLinePars3.maxAcceleration = TCP_ACC ;
// 
// 	segArcPars3.plane = ROCKS_PLANE_ZX;
// 	segArcPars3.center[0] = segLinePars3.endPos[0] + 2;
// 	segArcPars3.center[1] = segLinePars3.endPos[1];
// 	segArcPars3.endPos[0] = segLinePars3.endPos[0] + 2;
// 	segArcPars3.endPos[1] = segLinePars3.endPos[1] + 2;
// 	segArcPars3.endVelocity = TCP_SPEED;
// 	segArcPars3.maxAcceleration = TCP_ACC ;
// 	segArcPars3.positiveAngle = TRUE;
// 
// 	segLinePars4.plane = ROCKS_PLANE_ZX;
// 	segLinePars4.endPos[0] = segArcPars3.endPos[0] + 20;
// 	segLinePars4.endPos[1] = segArcPars3.endPos[1];
// 	segLinePars4.endVelocity = TCP_SPEED;
// 	segLinePars4.maxAcceleration = TCP_ACC ;
// 
// 	segArcPars4.plane = ROCKS_PLANE_ZX;
// 	segArcPars4.center[0] = segLinePars4.endPos[0];
// 	segArcPars4.center[1] = segLinePars4.endPos[1] - 2;
// 	segArcPars4.endPos[0] = segLinePars4.endPos[0] + 2;
// 	segArcPars4.endPos[1] = segLinePars4.endPos[1] - 2;
// 	segArcPars4.endVelocity = TCP_SPEED;
// 	segArcPars4.maxAcceleration = TCP_ACC ;
// 	segArcPars4.positiveAngle = TRUE;
// 
// 	segLinePars5.plane = ROCKS_PLANE_ZX;
// 	segLinePars5.endPos[0] = segArcPars4.endPos[0];
// 	segLinePars5.endPos[1] = segArcPars4.endPos[1] - 10;
// 	segLinePars5.endVelocity = 0;
// 	segLinePars5.maxAcceleration = TCP_ACC ;

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

// WORD number;
// CHAR str[1000];
// int counter = 0;
// 
// void processConsoleCommand(HANDLE hConlseInput)
// {
// 	INPUT_RECORD irInBuf[128];
// 	ReadConsoleInput(hConlseInput, irInBuf, 128, (LPDWORD)&number);
// 	for (int i =0; i < number; ++i )
// 	{
// 		if(irInBuf[i].EventType == KEY_EVENT && irInBuf[i].Event.KeyEvent.bKeyDown)
// 		{
// 			if (irInBuf[i].Event.KeyEvent.uChar.AsciiChar == 13)
// 			{
// 				str[counter] = '\0';
// 				counter = 0;
// 				cout<<str<<endl;
// 				if (strcmp(str, "q") || strcmp(str, "Q"))
// 				{
// 					SetEvent(hEvStop);
// 				}
// 				else
// 					cout<<"Please inter 'Q' to terminal.\n"<<endl;	
// 			}
// 			else
// 			{
// 				str[counter] = irInBuf[i].Event.KeyEvent.uChar.AsciiChar;
// 				counter++;
// 			}
// 		}
// 	}
// }

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

		programState = 1;
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
	case 9://移动到准备位置
		programState = 1;
		break;
	default:
		break;
	}

// 	HANDLE h[2];//no need to close there two HANDLE.
// 	h[0] = hEvStop;
// 	h[1] = GetStdHandle(STD_INPUT_HANDLE);
// 	cout<<"Please inter 'Q' to terminal.\n"<<endl;
// 	while(true)
// 	{
// 		switch(WaitForMultipleObjects(2, h, FALSE, INFINITE))8
// 		{
// 		case WAIT_OBJECT_0:
// 			goto stop;
// 		case WAIT_OBJECT_0 + 1:
// 			processConsoleCommand(h[1]); 
// 		default:
// 			break;
// 		}
// 	}
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
// 
// 	system("pause");
	return 0;
}

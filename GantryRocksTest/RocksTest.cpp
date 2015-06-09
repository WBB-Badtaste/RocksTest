// GantryRocksTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <process.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <nyceapi.h>
#include <sacapi.h>
#include <nhiapi.h> 
#include <rocksapi.h>

#include "DeltaRobot.h"

using namespace std;

//#define GET_MOTION_VAR
//#define USE_NODE
#define USE_AXIS
#define SIM_METHOD

NYCE_STATUS nyceStatus;
HANDLE hEvStop;

LONG64 times = 0;
LONG64 timesCounter = 0;

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
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacHome( axId[ ax ] );
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_HOMING_COMPLETED, 10 );
				break;
			case SAC_MOVING:
				printf("Waiting the motion stop...");
				nyceStatus =  NyceError(nyceStatus) ? nyceStatus : SacSynchronize( axId[ ax ], SAC_REQ_MOTION_STOPPED, 30 );
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
	double rate_angle2pu[ROCKS_MECH_MAX_NR_OF_JOINTS];
	for (uint32_t ax = 0; ax < pMech->nrOfJoints; ++ax)//只在当前情况下有效
	{
		rate_angle2pu[ax] = 0.5 / M_PI;
	}
	delta_calcForward(pJointPos[0] / rate_angle2pu[0], pJointPos[1] / rate_angle2pu[1], pJointPos[2] / rate_angle2pu[2], pMechPos[0], pMechPos[1], pMechPos[2]);
	return NYCE_OK; 
}

NYCE_STATUS RocksKinDeltaPosition(struct rocks_mech* pMech, double pPos[])
{
	double *pJointPos = new double[ROCKS_MECH_MAX_DOF];
	NYCE_STATUS nyceStatus = NYCE_OK;
	for (uint32_t ax = 0; ax < pMech->nrOfJoints; ax++)
	{
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReadVariable(axId[ax], SAC_VAR_SETPOINT_POS, &pJointPos[ax]);
	}
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksKinForwardDelta(pMech, pJointPos, pPos);	

// 	pPos[0] = (double)(int)(pPos[0] * 100000) / 100000;
// 	pPos[1] = (double)(int)(pPos[1] * 100000) / 100000;
// 	pPos[2] = (double)(int)(pPos[2] * 100000) / 100000;

	return nyceStatus;
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
// 		double alhpa = acos(-pMech->var.center[0] / sqrt(pMech->var.center[0] * pMech->var.center[0] + pMech->var.center[1] * pMech->var.center[1]));
// 		double beta = pMech->var.center[1] < 0 ? alhpa : M_PI * 2 - alhpa;
// 		double angle;
// 		if (pMech->var.angle > 0)
// 			angle = beta - pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;
// 		else
// 			angle = beta + pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;

		double angle;
		double alhpa;
		double beta ;
		double CenterWorldCoordinate[3];
		switch(pMech->var.plane)
		{
		case ROCKS_PLANE_XY:
// 			CenterWorldCoordinate[0] = pMech->var.startPos[0] + pMech->var.center[0];
// 			CenterWorldCoordinate[1] = pMech->var.startPos[1] + pMech->var.center[1];
// 			CenterWorldCoordinate[2] = pMech->var.startPos[2];

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
// 			CenterWorldCoordinate[0] = pMech->var.startPos[0];
// 			CenterWorldCoordinate[1] = pMech->var.startPos[1] + pMech->var.center[0];
// 			CenterWorldCoordinate[2] = pMech->var.startPos[2] + pMech->var.center[1];

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
// 			CenterWorldCoordinate[0] = pMech->var.startPos[0] + pMech->var.center[1];
// 			CenterWorldCoordinate[1] = pMech->var.startPos[1];
// 			CenterWorldCoordinate[2] = pMech->var.startPos[2] + pMech->var.center[0];

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
} 

NYCE_STATUS RocksKinInverseDelta(ROCKS_MECH* pMech, const ROCKS_KIN_INV_PARS* pKin)
{
	double rate_angle2pu[ROCKS_MECH_MAX_NR_OF_JOINTS];
	uint32_t ax = 0;
	while (!pMech->var.jointBuffersAllocated)
	{
 		pMech->var.pJointPositionBufferC[ax] = (double*)malloc(sizeof(double) * pMech->var.maxNrOfSplines);
 		pMech->var.pJointVelocityBufferC[ax] = (double*)malloc(sizeof(double) * pMech->var.maxNrOfSplines);
		if(++ax == pMech->nrOfJoints) pMech->var.jointBuffersAllocated = TRUE;
	}

	for (ax = 0; ax < pMech->nrOfJoints; ++ax)//只在当前情况下有效
	{
		rate_angle2pu[ax] = 0.5 / M_PI;
	}

	for (uint32_t index = 0; index < pMech->var.usedNrOfSplines; ++index)
	{
		double pos[3],vel[3];
		double pos_joint_x, pos_joint_y, pos_joint_z;//joint angle(JA)
		double vel_joint_x, vel_joint_y, vel_joint_z;//joint angular velocity(JAV)

		DeltaPath2WorldCoordinate(pMech, index, pos, vel);

// 		double angle = pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;
// 		//convert PTCP to WTCP
// 		pos[0] = pMech->var.startPos[0] + pMech->var.radius + pMech->var.radius * cos(M_PI - angle);
// 		pos[1] = pMech->var.startPos[1] + pMech->var.radius * sin(M_PI - angle) * sin(M_PI / 2 + pMech->var.refFramePose2.r.x);
// 		pos[2] = pMech->var.startPos[2] + pMech->var.radius * sin(M_PI - angle) * cos(M_PI / 2 + pMech->var.refFramePose2.r.x);
// 
// 		//convert PTCV to WTCV
// 		vel[0] = pMech->var.pVelocitySplineBuffer[index] * sin(angle);
// 		vel[1] = pMech->var.pVelocitySplineBuffer[index] * cos(angle) * sin(M_PI / 2 + pMech->var.refFramePose2.r.x);
// 		vel[2] = pMech->var.pVelocitySplineBuffer[index] * cos(angle) * cos(M_PI / 2 + pMech->var.refFramePose2.r.x);

		delta_calcInverse(pos[0], pos[1], pos[2], pos_joint_x, pos_joint_y, pos_joint_z);

		//convert the JA to joint position(JP)
		pMech->var.pJointPositionBufferC[0][index] = pos_joint_x * rate_angle2pu[0];
		pMech->var.pJointPositionBufferC[1][index] = pos_joint_y * rate_angle2pu[1];
		pMech->var.pJointPositionBufferC[2][index] = pos_joint_z * rate_angle2pu[2];

		delta_velInverse(pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], pos_joint_x, pos_joint_y, pos_joint_z, vel_joint_x, vel_joint_y, vel_joint_z);

		//convert the JAV to joint velocity(JV)
		pMech->var.pJointVelocityBufferC[0][index] = vel_joint_x * rate_angle2pu[0];
		pMech->var.pJointVelocityBufferC[1][index] = vel_joint_y * rate_angle2pu[1];
		pMech->var.pJointVelocityBufferC[2][index] = vel_joint_z * rate_angle2pu[2];

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
	pMech->var.pApplyForwardKinFunc = RocksKinForwardDelta;
	pMech->var.pApplyInverseKinFunc = RocksKinInverseDelta;
	pMech->var.pGetWorldSetpointPosFunc = RocksKinDeltaPosition;
	return NYCE_OK;
}

#include <fstream>

unsigned __stdcall ThreadRocksLoop(void* lpParam)
{
	NYCE_STATUS Status = NYCE_OK;

	while(!bRocksTerm)
	{
		Status = NyceError( Status ) ? Status : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

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
		Status = NyceError( Status ) ? Status : RocksStream( &m_mech);	

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

	// Delete mechanism
	// ----------------
	Status = NyceError( Status ) ? Status : RocksMechDelete( &m_mech );

	if (NyceError( Status ))
	{
		HandleError(Status, "Rocks");
	}
	return 0;
}

BOOL Rocks(void)
{
	
	ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccPars;
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

	//nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDefineGantry( &m_mech, ROCKS_GANTRY_Y );
	
	// Get current position
	// --------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPars.startPos);

	// Define the circle
	// -----------------
	sineAccPars.maxVelocity = 10000;
	sineAccPars.maxAcceleration = 100000;
	sineAccPars.splineTime = 0.0002;
	sineAccPars.center[ 0 ] = sineAccPars.startPos[0] + 150;
	sineAccPars.center[ 1 ] = sineAccPars.startPos[1];
	sineAccPars.angle = M_PI * 8;
	sineAccPars.plane = ROCKS_PLANE_XY;
	sineAccPars.maxNrOfSplines = 0;
	sineAccPars.pPositionSplineBuffer = NULL;
	sineAccPars.pVelocitySplineBuffer = NULL;

	// Get path splinesc
	// ----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccPars );
	// Apply inverse kinematics to get joint splines
	// ---------------------------------------------
	for (int ax = 0; ax < NUM_AXES; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}

	rocksPose.r.x = M_PI * M_PI_2;
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
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin(&m_mech, &rocksPose);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath);

	if (NyceError( nyceStatus ))
	{
		HandleError("Rocks");
		return FALSE;
	}

	return TRUE;
}

WORD number;
CHAR str[1000];
int counter = 0;

void processConsoleCommand(HANDLE hConlseInput)
{
	INPUT_RECORD irInBuf[128];
	ReadConsoleInput(hConlseInput, irInBuf, 128, (LPDWORD)&number);
	for (int i =0; i < number; ++i )
	{
		if(irInBuf[i].EventType == KEY_EVENT && irInBuf[i].Event.KeyEvent.bKeyDown)
		{
			if (irInBuf[i].Event.KeyEvent.uChar.AsciiChar == 13)
			{
				str[counter] = '\0';
				counter = 0;
				cout<<str<<endl;
				if (strcmp(str, "q") || strcmp(str, "Q"))
				{
					SetEvent(hEvStop);
				}
				else
					cout<<"Please inter 'Q' to terminal.\n"<<endl;	
			}
			else
			{
				str[counter] = irInBuf[i].Event.KeyEvent.uChar.AsciiChar;
				counter++;
			}
		}
	}
}

#ifdef GET_MOTION_VAR
#include <fstream>
#define ADDR "..\\"
uint32_t varIndexs[NUM_AXES];
double vars[NUM_AXES];

void OnInterpolantEvent( NYCE_ID nyceId, NYCE_EVENT eventId, NYCE_EVENT_DATA *pEventData, void *pUserData )
{
	ofstream *pFile = (ofstream *)pUserData;
	NYCE_STATUS myStatus = NYCE_OK;
	double cartesianPos[ROCKS_MECH_MAX_DOF];
	myStatus = NyceError(myStatus) ? myStatus : NyceReadVariableSet(varIndexs[0], varIndexs[NUM_AXES - 1], vars);
	myStatus = NyceError(myStatus) ? myStatus : RocksKinCartesianPosition(&m_mech, cartesianPos);
	if (NyceError(myStatus))
	{
		HandleError(myStatus, "NyceReadVariableSet");
		SetEvent(hEvStop);
		return;
	}
 	for (int i = 0; i <NUM_AXES; ++i)
 	{
		*pFile<<vars[i]<<" ";
 	}
	for (int i = 0; i <NUM_AXES; ++i)
	{
		*pFile<<cartesianPos[i]<<" ";
	}
	*pFile<<endl;
}

#endif // GET_MOTION_VAR

int _tmain(int argc, _TCHAR* argv[])
{
	nyceStatus = NYCE_OK;	
	uint32_t varIndex = 0;
	unsigned uThreadRocks = 0;

#ifdef GET_MOTION_VAR
	SYSTEMTIME time;
	CHAR buffer[30];
	string addrText(ADDR);
	GetSystemTime(&time);
	sprintf_s(buffer, "SampleVars%d-%d-%d.txt", time.wYear, time.wMonth, time.wDay);
	addrText += buffer;
	ofstream file(addrText);
#endif // GET_MOTION_VAR
	
	printf("Begin...\n");
#ifdef SIM_METHOD
	nyceStatus = NyceInit(NYCE_SIM);
#else
	nyceStatus = NyceInit(NYCE_NET);
#endif // SIM_METHOD
	
	if (NyceError(nyceStatus))
	{
		HandleError("System");
		goto end;
	}

#ifdef USE_NODE
	if (!InitNode()) goto end;
#endif // USE_NODE
#ifdef USE_AXIS
	if (!InitAxis()) goto end;
#endif // USE_AXIS
	

#ifdef NT
	Sleep(500);//Some trouble will be happened without this code while using the simulation system.
			   //--For example, the error "spline buffer empty" occur.
#endif // NT

#ifdef GET_MOTION_VAR

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDefineEventEnrolment(axId[0], SAC_EV_INTERPOLANT_STARTED, OnInterpolantEvent, (void * )&file);
	if (NyceError(nyceStatus)) goto end;
	for (int ax = 0; ax <NUM_AXES; ++ax)
	{
		nyceStatus = SacAddVariableToSet(axId[ax], SAC_VAR_AXIS_POS, &varIndex);
		if (NyceError(nyceStatus))
		{
			HandleError(axName[ax]);
			goto term;
		}
		varIndexs[ax] = varIndex;
	}
// 	for (int ax = 0; ax <NUM_AXES; ++ax)
// 	{
// 		nyceStatus = SacAddVariableToSet(axId[ax], SAC_VAR_AXIS_VEL, &varIndex);
// 		if (NyceError(nyceStatus))
// 		{
// 			HandleError(axName[ax]);
// 			goto term;
// 		}
// 		varIndexs[ax + 3] = varIndex;
// 	}

#endif 
	if (!Rocks()) goto term;

	hThreadRocks = (HANDLE)_beginthreadex(NULL, NULL, ThreadRocksLoop, NULL, 0,&uThreadRocks);

	hEvStop = CreateEvent(NULL,TRUE,FALSE,NULL);
	Sleep(10);
	HANDLE h[2];//no need to close there two HANDLE.
	h[0] = hEvStop;
	h[1] = GetStdHandle(STD_INPUT_HANDLE);
	cout<<"Please inter 'Q' to terminal.\n"<<endl;
	while(true)
	{
		switch(WaitForMultipleObjects(2, h, FALSE, INFINITE))
		{
		case WAIT_OBJECT_0:
			goto stop;
		case WAIT_OBJECT_0 + 1:
			processConsoleCommand(h[1]); 
		default:
			break;
		}
	}

stop:
	CloseHandle(hEvStop);

	bRocksTerm = TRUE;
	cout<<"Wait for Rocks stop...\n"<<endl;
	WaitForSingleObject(hThreadRocks, INFINITE);
	CloseHandle(hThreadRocks);
	
term:
#ifdef GET_MOTION_VAR
	for (int ax = 0; ax < NUM_AXES; ++ax)
	{
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDeleteEventEnrolment(axId[ax], SAC_EV_INTERPOLANT_STARTED, OnInterpolantEvent, NULL);
	}

	file.close();
#endif // GET_MOTION_VAR
	
end:
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

	system("pause");
	return 0;
} 

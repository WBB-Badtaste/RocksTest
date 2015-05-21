// GantryRocksTest.cpp : Defines the entry point for the console application.
//


#include "stdafx.h"
#include <iostream>
#include <iomanip>
#include <Windows.h>
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
//#define SIM_METHOD

NYCE_STATUS nyceStatus;
HANDLE hEvStop;

void HandleError(const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusString(nyceStatus)<<"\n"<<endl;
}

void HandleError(NYCE_STATUS Status, const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusString(Status)<<"\n"<<endl;
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
const char *axName[ NUM_AXES ] = { "DEF_AXIS_2", "DEF_AXIS_1", "DEF_AXIS_3"};
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
ROCKS_TRAJ_PATH rocksTrajPath1, rocksTrajPath2;
ROCKS_POSE rocksPose;
BOOL bRocksTerm = FALSE;
BOOL bTurn = FALSE;
HANDLE hThreadRocks;

NYCE_STATUS RocksKinForwardDelta(struct rocks_mech* pMech, const double pJointPos[], double pMechPos[])
{
	ZeroMemory(pMechPos, sizeof(double) * ROCKS_MECH_MAX_DOF);
	double rate_angle2pu[ROCKS_MECH_MAX_NR_OF_JOINTS];
	for (int ax = 0; ax < pMech->nrOfJoints; ++ax)//只在当前情况下有效
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
	for (int ax = 0; ax < pMech->nrOfJoints; ax++)
	{
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReadVariable(axId[ax], SAC_VAR_AXIS_POS, &pJointPos[ax]);
	}
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksKinForwardDelta(pMech, pJointPos, pPos);	
	return nyceStatus;
}

NYCE_STATUS RocksKinInverseDelta(ROCKS_MECH* pMech, const ROCKS_KIN_INV_PARS* pKin)
{
	double rate_angle2pu[ROCKS_MECH_MAX_NR_OF_JOINTS];
	int ax = 0;
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

	int index = 0;
	for (index = 0; index < pMech->var.usedNrOfSplines; ++index)
	{
		double pos_x,pos_y,pos_z,vel_x,vel_y,vel_z;
		double pos_joint_x, pos_joint_y, pos_joint_z;//joint angle(JA)
		double vel_joint_x, vel_joint_y, vel_joint_z;//joint angular velocity(JAV)

		if (pMech->var.moveType == ROCKS_MOVE_TYPE_CIRCULAR)//check the path type
		{
			double s = 2 * M_PI * pMech->var.radius; //circumference
			double angle = pMech->var.pPositionSplineBuffer[index] / pMech->var.radius;

			//convert PTCP to WTCP
			pos_x = pMech->var.startPos[0] + pMech->var.radius - pMech->var.radius * cos(angle);
			pos_y = pMech->var.startPos[1] + pMech->var.radius * sin(angle) * cos(pMech->var.refFramePose2.r.x);
			pos_z = pMech->var.startPos[2] - pMech->var.radius * sin(angle) * sin(pMech->var.refFramePose2.r.x);

			//convert PTCV to WTCV
			vel_x =	pMech->var.pVelocitySplineBuffer[index] * sin(angle);
			vel_y =	pMech->var.pVelocitySplineBuffer[index] * cos(angle) * cos(pMech->var.refFramePose2.r.x);
			vel_z = -pMech->var.pVelocitySplineBuffer[index] * cos(angle) * sin(pMech->var.refFramePose2.r.x);
		} 
		delta_calcInverse(pos_x, pos_y, pos_z, pos_joint_x, pos_joint_y, pos_joint_z);

		//convert the JA to joint position(JP)
		pMech->var.pJointPositionBufferC[0][index] = pos_joint_x * rate_angle2pu[0];
		pMech->var.pJointPositionBufferC[1][index] = pos_joint_y * rate_angle2pu[1];
		pMech->var.pJointPositionBufferC[2][index] = pos_joint_z * rate_angle2pu[2];

		//set precision -> 0.00001 
		pMech->var.pJointPositionBufferC[0][index] = (double)(int)(pMech->var.pJointPositionBufferC[0][index] * 10000000) / 10000000;
		pMech->var.pJointPositionBufferC[1][index] = (double)(int)(pMech->var.pJointPositionBufferC[1][index] * 10000000) / 10000000;
		pMech->var.pJointPositionBufferC[2][index] = (double)(int)(pMech->var.pJointPositionBufferC[2][index] * 10000000) / 10000000;

		delta_velInverse(pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, pos_joint_x, pos_joint_y, pos_joint_z, vel_joint_x, vel_joint_y, vel_joint_z);

		//convert the JAV to joint velocity(JV)
		pMech->var.pJointVelocityBufferC[0][index] = vel_joint_x * rate_angle2pu[0];
		pMech->var.pJointVelocityBufferC[1][index] = vel_joint_y * rate_angle2pu[1];
		pMech->var.pJointVelocityBufferC[2][index] = vel_joint_z * rate_angle2pu[2];

		//set precision -> 0.00001 
		pMech->var.pJointVelocityBufferC[0][index] = (double)(int)(pMech->var.pJointVelocityBufferC[0][index] * 10000000) / 10000000;
		pMech->var.pJointVelocityBufferC[1][index] = (double)(int)(pMech->var.pJointVelocityBufferC[1][index] * 10000000) / 10000000;
		pMech->var.pJointVelocityBufferC[2][index] = (double)(int)(pMech->var.pJointVelocityBufferC[2][index] * 10000000) / 10000000;
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
		Status = NyceError( Status ) ? Status : RocksTrajLoadPath(&m_mech, &rocksTrajPath1);
// 		if (!bTurn)
// 		{
// 			Status = NyceError( Status ) ? Status : RocksTrajLoadPath(&m_mech, &rocksTrajPath1);
// 			bTurn = TRUE;
// 		}
// 		else
// 		{
// 			Status = NyceError( Status ) ? Status : RocksTrajLoadPath(&m_mech, &rocksTrajPath2);
// 			bTurn = FALSE;
// 		}

		//Status = NyceError( Status ) ? Status : RocksKinInverseCartesian( &m_mech, &kinPars );
		Status = NyceError( Status ) ? Status : RocksKinInverseDelta( &m_mech, &kinPars );
		//Status = NyceError( Status ) ? Status : RocksKinInverseGantry( &m_mech, &kinPars );

		ofstream file("..//data3.txt");	
		file<<m_mech.var.startPos[0]<< " "<<m_mech.var.startPos[1]<<" "<<m_mech.var.startPos[2]<<endl;
		for (int i = 0; i < m_mech.var.usedNrOfSplines; ++i)
		{
			file<<i<<" "<<m_mech.var.pPositionSplineBuffer[i]<<" "<<m_mech.var.pJointPositionBufferC[0][i]<<" "<<m_mech.var.pJointVelocityBufferC[0][i]<<" "<<m_mech.var.pJointPositionBufferC[1][i]<<" "<<m_mech.var.pJointVelocityBufferC[1][i]<<" "<<m_mech.var.pJointPositionBufferC[2][i]<<" "<<m_mech.var.pJointVelocityBufferC[2][i]<<endl;
		}


		// Feed splines to the joints
		// --------------------------
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
	}	

	Status = NyceError( Status ) ? Status : RocksTrajDeletePath( &m_mech, &rocksTrajPath1 );
	//Status = NyceError( Status ) ? Status : RocksTrajDeletePath( &m_mech, &rocksTrajPath2 );

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
	ROCKS_TRAJ_SINE_ACC_PTP_PARS sineAccPtpPars1, sineAccPtpPars2;
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

	// Define the circle
	// -----------------
	sineAccPars.maxVelocity = 1700;
	sineAccPars.maxAcceleration = 20000;
	sineAccPars.splineTime = 0.01;
	sineAccPars.center[ 0 ] = 150.0;
	sineAccPars.center[ 1 ] = 0.0;
	sineAccPars.angle = M_PI * 8;
	sineAccPars.plane = ROCKS_PLANE_XY;
	sineAccPars.maxNrOfSplines =  0;
	sineAccPars.pPositionSplineBuffer = NULL;
	sineAccPars.pVelocitySplineBuffer = NULL;

	sineAccPtpPars1.maxVelocity = 20;
	sineAccPtpPars1.maxAcceleration = 200;
	sineAccPtpPars1.splineTime = 0.01;
	sineAccPtpPars1.maxNrOfSplines = 0;
	sineAccPtpPars1.pPositionSplineBuffer = NULL;
	sineAccPtpPars1.pVelocitySplineBuffer = NULL;

	sineAccPtpPars2.maxVelocity = 20;
	sineAccPtpPars2.maxAcceleration = 200;
	sineAccPtpPars2.splineTime = 0.01;
	sineAccPtpPars2.maxNrOfSplines = 0;
	sineAccPtpPars2.pPositionSplineBuffer = NULL;
	sineAccPtpPars2.pVelocitySplineBuffer = NULL;
	
	// Get current position
	// --------------------
	//nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinGantryPosition( &m_mech, sineAccPars.startPos );
	//nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinCartesianPosition( &m_mech, sineAccPars.startPos );
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPars.startPos);
// 	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPtpPars1.startPos);
// 	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPtpPars1.endPos);
// 	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPtpPars2.startPos);
// 	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sineAccPtpPars2.endPos);

	sineAccPtpPars1.endPos[1] = 6;
	sineAccPtpPars2.startPos[1] = 6;

	// Get path splinesc
	// ----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccPars );
	//nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccPtp( &m_mech, &sineAccPtpPars2);
	// Apply inverse kinematics to get joint splines
	// ---------------------------------------------
	for (int ax = 0; ax < NUM_AXES; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}

	rocksPose.r.x = 0;
	rocksPose.r.y = 0;
	rocksPose.r.z = 0;
	rocksPose.t.x = 0;
	rocksPose.t.y = 0;
	rocksPose.t.z = 0;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin(&m_mech, &rocksPose);

// 	ofstream file("..//data.txt");
// 	for (int i = 0; i < m_mech.var.usedNrOfSplines; ++i)
// 	{
// 		file<<i<<" "<<m_mech.var.pPositionSplineBuffer[i]<<" "<<m_mech.var.pVelocitySplineBuffer[i]<<endl;
// 	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath1);

//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath2);
// 	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccPtp( &m_mech, &sineAccPtpPars1);
// 	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath1);

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

//	DeltaInverseKinVelInitialize();
// 	if (!GeschkinematikInitialize())
// 		return -1;

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

//	DeltaInverseKinVelTerminate();
// 	GeschkinematikTerminate();
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

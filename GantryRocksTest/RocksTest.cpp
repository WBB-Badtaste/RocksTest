// GantryRocksTest.cpp : Defines the entry point for the console application.
//


#include "stdafx.h"
#include <iostream>
#include <Windows.h>
#include <string>
#include <process.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <nyceapi.h>
#include <sacapi.h>
#include <nhiapi.h>
#include <rocksapi.h>

using namespace std;

//#define GET_MOTION_VAR
//#define USE_NODE
#define USE_AXIS
#define SIM_METHOD

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
#define NUM_AXES 4
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3", "DEF_AXIS_4"};
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
			case SAC_ERROR:
			case SAC_INACTIVE:
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
			default:
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
BOOL bRocksTerm = FALSE;
HANDLE hThreadRocks;

unsigned __stdcall ThreadRocksLoop(void* lpParam)
{
	NYCE_STATUS Status = NYCE_OK;

	while(!bRocksTerm)
	{
		Status = NyceError( Status ) ? Status : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

		Status = NyceError( Status ) ? Status : RocksKinInverseCartesian( &m_mech, &kinPars );

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

	Status = NyceError( Status ) ? Status : RocksTrajDeletePath(&m_mech, &rocksTrajPath);

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
	//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDefineGantry( &m_mech, ROCKS_GANTRY_X );

	// Define the circle
	// -----------------
	sineAccPars.maxVelocity = 300;//.03272
	sineAccPars.maxAcceleration = 3000.0;
	sineAccPars.splineTime = 0.001;
	sineAccPars.center[ 0 ] = 300.0;
	sineAccPars.center[ 1 ] = 0.0;
	sineAccPars.angle = M_PI * 2;
	sineAccPars.plane = ROCKS_PLANE_XY;
	sineAccPars.maxNrOfSplines =  0;
	sineAccPars.pPositionSplineBuffer = NULL;
	sineAccPars.pVelocitySplineBuffer = NULL;

	// Get current position
	// --------------------
	//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinGantryPosition( &m_mech, sineAccPars.startPos );
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinCartesianPosition( &m_mech, sineAccPars.startPos );

	// Get path splines
	// ----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccPars );

	// Apply inverse kinematics to get joint splines
	// ---------------------------------------------
	for (int ax = 0; ax < NUM_AXES; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}

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

uint32_t varIndexs[NUM_AXES * 2];
double vars[NUM_AXES * 2];

void OnInterpolantEvent( NYCE_ID nyceId, NYCE_EVENT eventId, NYCE_EVENT_DATA *pEventData, void *pUserData )
{
	NYCE_STATUS myStatus = NYCE_OK;
	myStatus = NyceError(myStatus) ? myStatus : NyceReadVariableSet(varIndexs[0], varIndexs[NUM_AXES * 2 - 1], vars);
	if (NyceError(myStatus))
	{
		HandleError(myStatus, "NyceReadVariableSet");
		SetEvent(hEvStop);
		return;
	}
	for (int i = 0; i <NUM_AXES * 2; ++i)
	{
		cout<<vars[i]<<" ";
	}
	cout<<endl;
}

#endif // GET_MOTION_VAR

int _tmain(int argc, _TCHAR* argv[])
{
	nyceStatus = NYCE_OK;	
	uint32_t varIndex = 0;
	unsigned uThreadRocks = 0;
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

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacDefineEventEnrolment(axId[0], SAC_EV_INTERPOLANT_STARTED, OnInterpolantEvent, NULL);
	if (NyceError(nyceStatus)) goto end;

	for (int ax = 0; ax <NUM_AXES; ++ax)
	{
		nyceStatus = SacAddVariableToSet(axId[ax], SAC_VAR_AXIS_VEL, &varIndex);
		if (NyceError(nyceStatus))
		{
			HandleError(axName[ax]);
			goto term;
		}
		varIndexs[ax *2] = varIndex;
		nyceStatus = SacAddVariableToSet(axId[ax], SAC_VAR_AXIS_POS, &varIndex);
		if (NyceError(nyceStatus))
		{
			HandleError(axName[ax]);
			goto term;
		}
		varIndexs[ax *2 + 1] = varIndex;
	}

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

// RocksEx.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <sacapi.h>
#include "ErrorHandle.h"
#include "AxisControl.h"
#include "RocksControl.h"
#include <Windows.h>

#define SIM_METHOD
//#define CARTESIAN

#define NUM_AXES 3
#ifdef CARTESIAN
const char *axName[ NUM_AXES ] = { "DEF_AXIS_4", "DEF_AXIS_5", "DEF_AXIS_6"};
#else
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3"};
#endif
SAC_AXIS axId[NUM_AXES];

NYCE_STATUS nyceStatus(NYCE_OK);

int _tmain(int argc, _TCHAR* argv[])
{
	printf("Begin...\n");

#ifdef SIM_METHOD
	nyceStatus = NyceInit(NYCE_SIM);
#else
	nyceStatus = NyceInit(NYCE_ETH);
#endif // SIM_METHOD

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksExExportSplineDatas(TRUE);


#ifdef CARTESIAN
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES, axId, axName);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksInitCartesian(NUM_AXES, axId);

	const double rate(1000);
	CARTESIAN_COORD cricleCenterOffest;
	cricleCenterOffest.x = -30 * rate;
	cricleCenterOffest.y = 0 * rate;
	cricleCenterOffest.z = -30 * rate;

	double angle = 2.0 * M_PI;

	TRAJ_PARS cricleTrajPars;
	cricleTrajPars.velocity = 100 * rate;
	cricleTrajPars.acceleration = 100 * rate;
	cricleTrajPars.splineTime = 0.0002;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksCricleCartesian(cricleCenterOffest, angle, cricleTrajPars, SAC_INDEFINITE, 0);
#else
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisSanyo(NUM_AXES, axId, axName);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksInitDelta(NUM_AXES, axId);

// 	CARTESIAN_COORD homePos;
// 	homePos.x = 30;
// 	homePos.y = 0;
// 	homePos.z = -160;

	CARTESIAN_COORD homePos;
	homePos.x = -65;
	homePos.y = 0;
	homePos.z = -220;

	TRAJ_PARS homeTrajPars;
	homeTrajPars.velocity = 100;
	homeTrajPars.acceleration = 1000;
	homeTrajPars.splineTime = 0.001;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(homePos, homeTrajPars);

	CARTESIAN_COORD cricleCenterOffest;
	cricleCenterOffest.x = -30;
	cricleCenterOffest.y = 0;
	cricleCenterOffest.z = -30;

	double angle = 2.0 * M_PI;

	TRAJ_PARS cricleTrajPars;
	cricleTrajPars.velocity = 100;
	cricleTrajPars.acceleration = 1000;
	cricleTrajPars.splineTime = 0.0002;

//	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksCricleDelta(cricleCenterOffest, angle, cricleTrajPars, SAC_INDEFINITE, 0);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta();
#endif

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksTerm();

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();

	if (NyceError(nyceStatus))
	{
		HandleError(nyceStatus, "");
	}

	system("pause");
	return 0;
}


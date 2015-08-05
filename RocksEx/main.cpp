// RocksEx.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <sacapi.h>
#include "ErrorHandle.h"
#include "AxisControl.h"
#include "RocksControl.h"
#include <Windows.h>

#define NUM_AXES 3
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3"};
SAC_AXIS axId[NUM_AXES];
NYCE_STATUS nyceStatus(NYCE_OK);

#define SIM_METHOD

int _tmain(int argc, _TCHAR* argv[])
{
	printf("Begin...\n");

#ifdef SIM_METHOD
	nyceStatus = NyceInit(NYCE_SIM);
#else
	nyceStatus = NyceInit(NYCE_ETH);
#endif // SIM_METHOD

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxis(NUM_AXES, axId, axName);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksInit(NUM_AXES, axId);

	//nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksExExportSplineDatas(TRUE);

	CARTESIAN_COORD homePos;
	homePos.x = 30;
	homePos.y = 0;
	homePos.z = -160;

	TRAJ_PARS homePars;
	homePars.velocity = 10;
	homePars.acceleration = 3000;
	homePars.splineTime = 0.0002;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(homePos, homePars, 30);



	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksTerm();

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();

	if (NyceError(nyceStatus))
	{
		HandleError(nyceStatus, "");
	}

	system("pause");
	return 0;
}


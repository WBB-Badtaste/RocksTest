#pragma once

#include "NyceExDefs.h"
#include <nyceapi.h>

const char* NyceGetStatusStringEx(NYCE_STATUS statusCode)
{
	switch (statusCode)
	{
	case ROCKS_ERR_PU_RATE_ERROR:
		return "ROCKS_ERR_PU_RATE_ERROR";
		break;
	case ROCKS_ERR_DELTA_PARS_ERROR:
		return "ROCKS_ERR_DELTA_PARS_ERROR";
		break;
	case ROCKS_ERR_DELTA_TAJ_OVER_WORKSPACE:
		return "ROCKS_ERR_DELTA_TAJ_OVER_WORKSPACE";
		break;
	case ROCKS_ERR_DELTA_TAJ_VEL_ERROR:
		return "ROCKS_ERR_DELTA_TAJ_VEL_ERROR";
		break;
	case ROCKS_ERR_DELTA_JOINT_POS_ERROR:
		return "ROCKS_ERR_DELTA_JOINT_POS_ERROR";
		break;
	case ROCKS_ERR_DELTA_POSTURE_ERROR:
		return "ROCKS_ERR_DELTA_POSTURE_ERROR";
		break;
	case ROCKS_ERR_MAX_ANGLE_ACCELERATION_EXCEEDED:
		return "ROCKS_ERR_MAX_ANGLE_ACCELERATION_EXCEEDED";
		break;
	case ROCKS_ERR_MAX_RADIAL_VELOCITY_EXCEEDED:
		return "ROCKS_ERR_MAX_RADIAL_VELOCITY_EXCEEDED";
		break;
	case ROCKS_ERR_MAX_RADIAL_ACCELERATION_EXCEEDED:
		return "ROCKS_ERR_MAX_RADIAL_ACCELERATION_EXCEEDED";
		break;
	default:
		return NyceGetStatusString(statusCode);
		break;
	}
}
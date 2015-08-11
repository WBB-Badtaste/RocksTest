#pragma once

#include "NyceExDefs.h"
#include <nyceapi.h>

const char* NyceGetStatusStringEx(NYCE_STATUS statusCode)
{
	switch (statusCode)
	{
	case ROCKS_ERR_PU_RATE_ERROR:
		return "角度与PU之间的比例设置错误。";
		break;
	case ROCKS_ERR_DELTA_PARS_ERROR:
		return "Delta机器人的机械参数设置错误。";
		break;
	case ROCKS_ERR_DELTA_TAJ_OVER_WORKSPACE:
		return "给定的轨迹超出Delta机器人工作区。";
		break;
	case ROCKS_ERR_DELTA_TAJ_VEL_ERROR:
		return "给定的轨迹速度有误。";
		break;
	case ROCKS_ERR_DELTA_JOINT_POS_ERROR:
		return "Delta机器人的三个轴位置不匹配。";
		break;
	case ROCKS_ERR_DELTA_POSTURE_ERROR:
		return "Delta机器人的姿势有问题。";
		break;
	default:
		return NyceGetStatusString(statusCode);
		break;
	}
}
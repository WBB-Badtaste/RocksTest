#pragma once

#include "NyceExDefs.h"
#include <nyceapi.h>

const char* NyceGetStatusStringEx(NYCE_STATUS statusCode)
{
	switch (statusCode)
	{
	case ROCKS_ERR_PU_RATE_ERROR:
		return "�Ƕ���PU֮��ı������ô���";
		break;
	case ROCKS_ERR_DELTA_PARS_ERROR:
		return "Delta�����˵Ļ�е�������ô���";
		break;
	case ROCKS_ERR_DELTA_TAJ_OVER_WORKSPACE:
		return "�����Ĺ켣����Delta�����˹�������";
		break;
	case ROCKS_ERR_DELTA_TAJ_VEL_ERROR:
		return "�����Ĺ켣�ٶ�����";
		break;
	case ROCKS_ERR_DELTA_JOINT_POS_ERROR:
		return "Delta�������λ�ò�ƥ�䡣";
		break;
	default:
		return NyceGetStatusString(statusCode);
		break;
	}
}
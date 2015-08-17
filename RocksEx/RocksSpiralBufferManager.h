#pragma once

#include <rocksapi.h>

double (*pPosSplineBuffer)[ROCKS_MECH_MAX_DOF];
double (*pVelSplineBuffer)[ROCKS_MECH_MAX_DOF];
uint32_t spiralBufferSize = 0;
BOOL bSpiralBufferAlloced = FALSE;

void SpiralBufferManage(ROCKS_MECH *pMech, const uint32_t& bufferEnd)
{
	if (pMech->var.usedNrOfSplines == 0)//新的path
	{
		spiralBufferSize = 0;

		if (bSpiralBufferAlloced)//如果程序首次调用spiral轨迹规划，是不用销毁缓冲区的
		{
			delete []pPosSplineBuffer;
			delete []pVelSplineBuffer;
		}

		pPosSplineBuffer = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();
		pVelSplineBuffer = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();

		bSpiralBufferAlloced = TRUE;
	}

	
	//管理原有buffer
	if ( bufferEnd > pMech->var.maxNrOfSplines)
	{
		uint32_t copySize(pMech->var.maxNrOfSplines * sizeof(double));
		pMech->var.maxNrOfSplines = bufferEnd + 512;
		uint32_t mallocSize(pMech->var.maxNrOfSplines * sizeof(double));
		

		double *pPosBuffer = (double *)malloc(mallocSize);
		double *pVelBuffer = (double *)malloc(mallocSize);
		ZeroMemory(pPosBuffer, mallocSize);
		ZeroMemory(pVelBuffer, mallocSize);

		memcpy(pPosBuffer, pMech->var.pPositionSplineBuffer, copySize);
		memcpy(pVelBuffer, pMech->var.pVelocitySplineBuffer, copySize);

		free(pMech->var.pPositionSplineBuffer);
		free(pMech->var.pVelocitySplineBuffer);
		pMech->var.pPositionSplineBuffer = pPosBuffer;
		pMech->var.pVelocitySplineBuffer = pVelBuffer;
	}

	//管理自定义buffer
	if(bufferEnd > spiralBufferSize)
	{
		uint32_t copySize(spiralBufferSize * sizeof(double));
		spiralBufferSize = pMech->var.maxNrOfSplines;

		double (*pPosBuffer)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();
		double (*pVelBuffer)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();

		memcpy(pPosBuffer[0], pPosSplineBuffer[0], copySize);
		memcpy(pPosBuffer[1], pPosSplineBuffer[1], copySize);
		memcpy(pPosBuffer[2], pPosSplineBuffer[2], copySize);
		memcpy(pVelBuffer[0], pVelSplineBuffer[0], copySize);
		memcpy(pVelBuffer[1], pVelSplineBuffer[1], copySize);
		memcpy(pVelBuffer[2], pVelSplineBuffer[2], copySize);

		delete []pPosSplineBuffer;
		delete []pVelSplineBuffer;

		pPosSplineBuffer = pPosBuffer;
		pVelSplineBuffer = pVelBuffer;
	}

	bSpiralBufferAlloced = TRUE;
}
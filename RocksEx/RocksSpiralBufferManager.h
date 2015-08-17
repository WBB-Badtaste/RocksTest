#pragma once

#include <rocksapi.h>

double *pPosSplineBuffer_x;
double *pPosSplineBuffer_y;
double *pPosSplineBuffer_z;
double *pVelSplineBuffer_x;
double *pVelSplineBuffer_y;
double *pVelSplineBuffer_z;
uint32_t spiralBufferSize = 0;
BOOL bSpiralBufferAlloced = FALSE;

void SpiralBufferManage(ROCKS_MECH *pMech, const uint32_t& bufferEnd)
{
	if (pMech->var.usedNrOfSplines == 0)//新的path
	{
		spiralBufferSize = 0;

		uint32_t mallocSize(2777 * sizeof(double));

		if (bSpiralBufferAlloced)//如果程序首次调用spiral轨迹规划，是不用销毁缓冲区的
		{
			free(pPosSplineBuffer_x);
			free(pPosSplineBuffer_y);
			free(pPosSplineBuffer_z);
			free(pVelSplineBuffer_x);
			free(pVelSplineBuffer_y);
			free(pVelSplineBuffer_z);
		}

		pPosSplineBuffer_x = (double *)malloc(mallocSize);
		pPosSplineBuffer_y = (double *)malloc(mallocSize);
		pPosSplineBuffer_z = (double *)malloc(mallocSize);
		pVelSplineBuffer_x = (double *)malloc(mallocSize);
		pVelSplineBuffer_y = (double *)malloc(mallocSize);
		pVelSplineBuffer_z = (double *)malloc(mallocSize);
		ZeroMemory(pPosSplineBuffer_x, mallocSize);
		ZeroMemory(pPosSplineBuffer_y, mallocSize);
		ZeroMemory(pPosSplineBuffer_z, mallocSize);
		ZeroMemory(pVelSplineBuffer_x, mallocSize);
		ZeroMemory(pVelSplineBuffer_y, mallocSize);
		ZeroMemory(pVelSplineBuffer_z, mallocSize);

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
		spiralBufferSize = bufferEnd + 512;
		uint32_t mallocSize(spiralBufferSize * sizeof(double));

		double *pPosBuffer_x = (double *)malloc(mallocSize);
		double *pPosBuffer_y = (double *)malloc(mallocSize);
		double *pPosBuffer_z = (double *)malloc(mallocSize);
		double *pVelBuffer_x = (double *)malloc(mallocSize);
		double *pVelBuffer_y = (double *)malloc(mallocSize);
		double *pVelBuffer_z = (double *)malloc(mallocSize);
		ZeroMemory(pPosBuffer_x, mallocSize);
		ZeroMemory(pPosBuffer_y, mallocSize);
		ZeroMemory(pPosBuffer_z, mallocSize);
		ZeroMemory(pVelBuffer_x, mallocSize);
		ZeroMemory(pVelBuffer_y, mallocSize);
		ZeroMemory(pVelBuffer_z, mallocSize);

		memcpy(pPosBuffer_x, pPosSplineBuffer_x, copySize);
		memcpy(pPosBuffer_y, pPosSplineBuffer_y, copySize);
		memcpy(pPosBuffer_z, pPosSplineBuffer_z, copySize);
		memcpy(pVelBuffer_x, pVelSplineBuffer_x, copySize);
		memcpy(pVelBuffer_y, pVelSplineBuffer_y, copySize);
		memcpy(pVelBuffer_z, pVelSplineBuffer_z, copySize);

		free(pPosSplineBuffer_x);
		free(pPosSplineBuffer_y);
		free(pPosSplineBuffer_z);
		free(pVelSplineBuffer_x);
		free(pVelSplineBuffer_y);
		free(pVelSplineBuffer_z);

		pPosSplineBuffer_x = pPosBuffer_x;
		pPosSplineBuffer_y = pPosBuffer_y;
		pPosSplineBuffer_z = pPosBuffer_z;
		pVelSplineBuffer_x = pVelBuffer_x;
		pVelSplineBuffer_y = pVelBuffer_y;
		pVelSplineBuffer_z = pVelBuffer_z;
	}

	bSpiralBufferAlloced = TRUE;
}
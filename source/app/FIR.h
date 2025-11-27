/*
 * FIR.h
 *
 *  Created on: 3 Nov 2025
 *      Author: jtori
 */

#include <stdint.h>

#ifndef APP_FIR_H_
#define APP_FIR_H_

typedef float Data_t;

typedef struct
{
	uint16_t order;
	Data_t* coeffs;
	Data_t* delay;
	uint16_t index;
} FIR;

FIR* FIR_Create(uint16_t order);
float* FIR_GetCoeffs(const FIR* fir);
uint16_t FIR_GetOrder(const FIR* fir);
void FIR_ComputeBuffer(FIR* fir, const Data_t* bufferIn, Data_t* bufferOut, uint32_t size);

#endif /* APP_FIR_H_ */

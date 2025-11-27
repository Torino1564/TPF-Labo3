/*
 * FIR.c
 *
 *  Created on: 3 Nov 2025
 *      Author: jtori
 */

#include "FIR.h"
#include <stdlib.h>


FIR* FIR_Create(uint16_t order)
{
    FIR* fir = calloc(1, sizeof(FIR));
    fir->order = order;
    fir->coeffs = calloc(order, sizeof(Data_t));
    fir->delay = calloc(order, sizeof(Data_t));
    fir->index = 0;
    return fir;
}


Data_t* FIR_GetCoeffs(const FIR* fir)
{
	return fir->coeffs;
}


uint16_t FIR_GetOrder(const FIR* fir)
{
	return fir->order;
}


void FIR_ComputeBuffer(FIR* fir, const Data_t* bufferIn, Data_t* bufferOut, uint32_t size)
{
    uint16_t order = fir->order;
    float* h = fir->coeffs;
    Data_t* d = fir->delay;
    uint16_t idx = fir->index;

    for (uint32_t n = 0; n < size; n++)
    {
        // Write new sample into delay line
        d[idx] = bufferIn[n];

        // Compute FIR
        Data_t acc = 0.0f;
        int16_t k = idx;

        for (uint16_t i = 0; i < order; i++)
        {
            acc += d[k] * h[i];

            // Circular decrement
            k--;
            if (k < 0)
                k = order - 1;
        }

        // Save result
        bufferOut[n] = acc;

        // Circular increment write index
        idx++;
        if (idx >= order)
            idx = 0;
    }

    fir->index = idx; // store updated write index
}

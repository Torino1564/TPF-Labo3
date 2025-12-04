/*
 * DSP.c
 *
 *  Created on: Nov 28, 2025
 *      Author: jtori
 */

#include "DSP.h"
#include <string.h>
#include <math.h>
#include "lut_tables.h"

static float buffer[MAX_BUFFER_SIZE*2] = {};

bool AutocorrelationFunction(float* pIn, float* pOut, uint32_t w)
{
//	memset(buffer,0 , sizeof(Complex)*MAX_BUFFER_SIZE*2);
//
//	if (pOut == 0)
//	{
//		pOut = pIn;
//	}
//
//	for (uint16_t i = 0; i < w; i++)
//	{
//		const float val = pIn[i];
//		if (val >= 0)
//			buffer[i] = (Complex){val, 0};
//		else
//			buffer[i] = (Complex){-val, M_PI};
//	}
//
//	ComputeFFT(buffer, 0, w);
//
//	// Conjugate:
//	for (uint16_t i = 0; i < w; i++)
//	{
//		buffer[i] = (Complex){pow(buffer[i].absolute_value, 2), 0};
//	}
//
//	ComputeIFFT(buffer, 0, w);
//
//	for (uint16_t i = 0; i < w; i++)
//	{
//		const Complex temp = buffer[i];
//		pOut[i] = fast_cos(temp.phase)*temp.absolute_value;
//	}
//
//	return 1;
}

#include "arm_math.h"
#include "arm_const_structs.h"

bool AutocorrelationFunction_2(float* pIn, float* pOut, uint32_t w)
{
	memset(buffer,0 , sizeof(float)*MAX_BUFFER_SIZE*2);
	float* x = pIn;
	float* X = buffer;
	float* rxx = pOut;

	arm_rfft_fast_instance_f32 S;

	// Initialize FFT
	arm_rfft_fast_init_f32(&S, w);

	// FFT
	arm_rfft_fast_f32(&S, x, X, 0);

	// Compute |X|^2 (power spectrum)
	for (int i = 0; i < w; i += 2) {
	    float re = X[i];
	    float im = X[i+1];
	    X[i]   = re*re + im*im;  // store power in real part
	    X[i+1] = 0.0f;
	}

	// IFFT → autocorrelation
	arm_rfft_fast_f32(&S, X, rxx, 1);
}

/* Compute difference function using a linear iteration and not FFT
 Parameters:
 - pBuffer: Buffer representing the data
 - w: size of the window
 - tau: Value to compute the Autocorrelation
 Note that the function assumes that the buffer will be AT LEAST of size W + Tau
 Return Value: Autocorrelation Value
 */
float LinearAutocorrelation(const float* pBuffer, const uint16_t w, const uint16_t tau)
{
    float result = 0;
    for (uint16_t i = 0; i < w; i++)
    {
        result += pBuffer[i] * pBuffer[i+tau];
    }
    return result;
}

bool DifferenceFunction(float* pIn, float* pOut, uint32_t w, uint32_t max_tau)
{
	static float acf0[MAX_BUFFER_SIZE];

	memset(buffer, 0, sizeof(buffer));
	// ACF t
	bool rv = AutocorrelationFunction_2(pIn, acf0, w);

	for (uint32_t i = 0; i < max_tau; i++)
	{
		float r0 = acf0[0];
		float r2 = acf0[i];
		float r1 = LinearAutocorrelation(pIn+i, w, 0);

		pOut[i] = r0 + r1 - 2*r2;
	}

	return 1;
}

bool CMNDF(float* pIn, uint32_t w)
{
    float runningSum = 0;

    pIn[0] = 1.0f;

    for (uint32_t tau = 1; tau < w; tau++)
    {
        runningSum += pIn[tau];
        pIn[tau] = (pIn[tau] * tau) / runningSum;
    }

    return 1;
}

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

static Complex buffer[MAX_BUFFER_SIZE*2] = {};

bool AutocorrelationFunction(data_t* pIn, data_t* pOut, uint32_t w)
{
	memset(buffer,0 , sizeof(Complex)*MAX_BUFFER_SIZE*2);

	if (pOut == 0)
	{
		pOut = pIn;
	}

	for (uint16_t i = 0; i < w; i++)
	{
		const float val = pIn[i];
		if (val >= 0)
			buffer[i] = (Complex){val, 0};
		else
			buffer[i] = (Complex){-val, M_PI};
	}

	ComputeFFT(buffer, 0, w);

	// Conjugate:
	for (uint16_t i = 0; i < w; i++)
	{
		buffer[i] = (Complex){pow(buffer[i].absolute_value, 2), 0};
	}

	ComputeIFFT(buffer, 0, w);

	for (uint16_t i = 0; i < w; i++)
	{
		const Complex temp = buffer[i];
		pOut[i] = fast_cos(temp.phase)*temp.absolute_value;
	}

	return 1;
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

bool DifferenceFunction(data_t* pIn, data_t* pOut, uint32_t w, uint32_t max_tau)
{
	static data_t acf0[MAX_BUFFER_SIZE];

	memset(buffer, 0, sizeof(buffer));
	// ACF t
	bool rv = AutocorrelationFunction(pIn, acf0, w);

	for (uint32_t i = 0; i < max_tau; i++)
	{
		data_t r0 = acf0[0];
		data_t r2 = acf0[i];
		data_t r1 = LinearAutocorrelation(pIn+i, w, 0);

		pOut[i] = r0 + r1 - 2*r2;
	}

	return 1;
}

bool CMNDF(data_t* pIn, uint32_t w)
{
    data_t runningSum = 0;

    pIn[0] = 1.0f;

    for (uint32_t tau = 1; tau < w; tau++)
    {
        runningSum += pIn[tau];
        pIn[tau] = (pIn[tau] * tau) / runningSum;
    }

    return 1;
}

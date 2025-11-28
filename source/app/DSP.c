/*
 * DSP.c
 *
 *  Created on: Nov 28, 2025
 *      Author: jtori
 */

#include "DSP.h"
#include <string.h>
#include <math.h>

static Complex buffer[MAX_BUFFER_SIZE*2] = {};

bool AutocorrelationFunction(data_t* pIn, data_t* pOut, uint32_t w, uint32_t t)
{
	memset(buffer,0 , sizeof(Complex)*MAX_BUFFER_SIZE*2);

	if (pOut == 0)
	{
		pOut = pIn;
	}

	for (uint16_t i = 0; i < w; i++)
	{
		const float val = pIn[i+t];
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
		pOut[i] = cosf(temp.phase)*temp.absolute_value;
	}

	return 1;
}

data_t DifferenceFunction(data_t* pIn, uint32_t w, uint32_t t, uint32_t tau)
{
	static data_t buffer[MAX_BUFFER_SIZE];

	memset(buffer, 0, sizeof(buffer));

	// ACF t
	bool rv = AutocorrelationFunction(pIn, buffer, w, t);
	float r0 = buffer[0];
	float r2 = buffer[tau];

	rv = AutocorrelationFunction(pIn, buffer, w, t+tau);
	float r1 = buffer[0];

	const float result = r0 + r1 + r2;
	return result;
}

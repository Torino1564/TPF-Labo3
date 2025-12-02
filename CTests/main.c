#include <stdio.h>
#include <stdint.h>
#include "FFT.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

#define BUFFER_SIZE 512

static Complex buffer[BUFFER_SIZE*2] = {};
#define RAND_MAX 32767
typedef float data_t;

bool AutocorrelationFunction(data_t* pIn, data_t* pOut, uint32_t w);
bool DifferenceFunction(data_t* pIn, data_t* pOut, uint32_t w);
float LinearAutocorrelation(const float* pBuffer, const uint16_t w, const uint16_t tau);
bool CMNDF(data_t* pIn, data_t* pOut, uint32_t w);

double generate_white_noise_sample() {
    // Generate a random integer between 0 and RAND_MAX
    int32_t random_int = rand();
    
    // Normalize to a double between 0.0 and 1.0
    double normalized_random = (double)random_int / RAND_MAX;
    
    // Scale and shift to the range [-1.0, 1.0]
    return (normalized_random * 2.0) - 1.0;
}

int main()
{   
    data_t bufferIn[BUFFER_SIZE];

    data_t bufferOut[BUFFER_SIZE];
    data_t buffer[BUFFER_SIZE];
    float fs = 32;

    for (uint16_t i = 0; i < BUFFER_SIZE; i++)
    {
        bufferIn[i] = sinf(2 * M_PI * (fs/BUFFER_SIZE) * i) * (2 << 7) * exp(-0.001f * i);
        printf("Index %d: %f\n", i, bufferIn[i]);
    }

    
    DifferenceFunction(bufferIn, bufferOut, BUFFER_SIZE/4);
    CMNDF(bufferOut, bufferOut, BUFFER_SIZE/4);

    for (uint16_t i = 0; i < BUFFER_SIZE/4; i++)
    {
        printf("Index %d: %f\n", i, bufferOut[i]);
    }
}

/* Compute difference function using a linear iteration and not FFT
 Parameters:
 - pBuffer: Buffer representing the data
 - w: size of the window
 - tau: Value to compute the Autocorrelation
 - This value is not to be used if the parameter is not fullfilled. Make sure the previous buffer is characterized before and only
 before calling this function with the inverse of the entropy value. Integrating this function in a larger system without taking care of this
 may lead to unexpected results.
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

bool CMNDF(data_t* pIn, data_t* pOut, uint32_t w)
{
    data_t runningSum = 0;

    pOut[0] = 1.0f;

    for (uint32_t tau = 1; tau < w; tau++)
    {
        runningSum += pIn[tau];
        pOut[tau] = pIn[tau] * tau / runningSum;
    }

    return 1;
}

bool DifferenceFunction(data_t* pIn, data_t* pOut, uint32_t w)
{
	static data_t acf0[BUFFER_SIZE];

	memset(buffer, 0, sizeof(buffer));

	// ACF t
	bool rv = AutocorrelationFunction(pIn, acf0, w);

	for (uint32_t i = 0; i < w; i++)
	{
		data_t r0 = acf0[0];
		data_t r2 = acf0[i];
		data_t r1 = LinearAutocorrelation(pIn+i, w, 0);

        data_t res = r0 + r1 - 2*r2;

		pOut[i] = res;
	}

	return 1;
}

bool AutocorrelationFunction(data_t* pIn, data_t* pOut, uint32_t w)
{
	memset(buffer,0 , sizeof(Complex)*BUFFER_SIZE*2);

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
		pOut[i] = cosf(temp.phase)*temp.absolute_value;
	}

	return 1;
}
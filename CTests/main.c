#include <stdio.h>
#include <stdint.h>
#include "FFT.h"
#define _USE_MATH_DEFINES
#include <math.h>

#define BUFFER_SIZE 128

int main()
{
    Complex bufferIn[BUFFER_SIZE];
    Complex bufferOut[BUFFER_SIZE];
    float fs = 32;

    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        bufferIn[i] = ComplexReIm(sinf((float)2*M_PI*i/(float)(BUFFER_SIZE/4)), 0);
    }

    ComputeFFT(bufferIn, bufferOut, BUFFER_SIZE);

    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        printf("Abs: %f, Phase: %f\n", bufferIn[i].absolute_value, bufferIn[i].phase);
    }
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

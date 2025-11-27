#include "FFT.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define LOG2_10 3.3219280948873623478703194294894

float log2f(float x)
{
	return log10(x)*LOG2_10;
}

Complex ComplexReIm(complex_type re, complex_type im)
{
    complex_type absolute_value = sqrt((re * re) + (im * im));
    const complex_type arg = fabs(im) / (fabs(re));
    complex_type phase = atan(arg);

    if (re == 0 && im == 0)
    {
        absolute_value = 0;
        phase = 0;
    }

    if (re < 0)
        phase = M_PI - phase;

    if (im < 0)
        phase *= -1;

    Complex temp;
    temp.absolute_value = absolute_value;
    temp.phase = phase;

    return temp;
}

complex_type ComplexRe(Complex a)
{
    return a.absolute_value * cos(a.phase);
}

complex_type ComplexIm(Complex a)
{
    return a.absolute_value * sin(a.phase);
}

complex_type ComplexAbs(Complex a)
{
    return a.absolute_value;
}

complex_type ComplexPhase(Complex a)
{
    return a.phase;
}

Complex ComplexAdd(Complex a, Complex b)
{
    complex_type re1 = ComplexRe(a);
    complex_type re2 = ComplexRe(b);
    
    complex_type im1 = ComplexIm(a);
    complex_type im2 = ComplexIm(b);

    complex_type newRe = re1 + re2;
    complex_type newIm = im1 + im2;

    return ComplexReIm(newRe, newIm);
}

Complex ComplexSubs(Complex a, Complex b)
{
    complex_type re1 = ComplexRe(a);
    complex_type re2 = ComplexRe(b);
    
    complex_type im1 = ComplexIm(a);
    complex_type im2 = ComplexIm(b);

    complex_type newRe = re1 - re2;
    complex_type newIm = im1 - im2;

    return ComplexReIm(newRe, newIm);
}

Complex ComplexMult(Complex a, Complex b)
{
    const complex_type abs = a.absolute_value * b.absolute_value;
    const complex_type phase = a.phase + b.phase;
    
    Complex temp;

    temp.absolute_value = abs;
    temp.phase = phase;
    return temp;
}

Complex ComplexDiv(Complex a, Complex b)
{
    const complex_type abs = a.absolute_value / b.absolute_value;
    const complex_type phase = a.phase - b.phase;
    
    Complex temp;

    temp.absolute_value = abs;
    temp.phase = phase;
    return temp;
}

bool ComplexCompare(Complex a, Complex b, complex_type epsilon)
{
    if (fabs(a.absolute_value - b.absolute_value) < epsilon && fabs(a.phase - b.phase) < epsilon)
    {
        return 1;
    }
    return 0;
}

Complex ComplexMultScalar(Complex a, complex_type b)
{
    const complex_type abs = a.absolute_value * b;
    complex_type phase = a.phase;
    Complex temp;
    if (b < 0)
        phase *= -1;
    temp.phase = phase;
    temp.absolute_value = abs;
    return temp;
}

Complex ComplexExp(Complex a, complex_type b)
{
    const complex_type abs = pow(a.absolute_value, b);
    complex_type phase = a.phase * b;

    Complex temp;
    temp.phase = phase;
    temp.absolute_value = abs;
    return temp;
}

scalar_type BitRev(const scalar_type input, size_t numBits)
	{
		scalar_type result = 0;
		for (unsigned int n = 0; n < numBits / 2; n++)
		{
			scalar_type fromRight = input & (0b1 << n);
			scalar_type shiftedFromRight = fromRight << (numBits - (2*n + 1));

			scalar_type fromLeft = input & (0b1 << (numBits - 1 - n));
			scalar_type shiftedFromLeft = fromLeft >> (numBits - (2*n + 1));

			result ^= shiftedFromLeft ^ shiftedFromRight;
		}

		if (numBits % 2 != 0)
		{
			scalar_type mask = input & (0b1 << (uint32_t)numBits/2);
			result ^= mask;
		}

		return result;
	}

bool IsPowerOf2(const uint32_t n)
{
    const uint32_t size = n;
    return (size & size - 1) == 0;
}

Complex* CalculateTwiddleFactors(const uint64_t numSamples)
{
    Complex* pArray = (Complex*)malloc(sizeof(Complex)*numSamples);

    float deltaPhi = 2 * M_PI / numSamples;
    for (uint64_t i = 0; i < numSamples; i++)
    {
        Complex temp;
        temp.absolute_value = 1;
        temp.phase = -1 * deltaPhi * i;
        pArray[i] = temp;
    }

    return pArray;
}

int ComputeFFT(Complex* in, Complex* out, const uint32_t n)
	{
		/* Data conditioning:
		 * - This first part makes sure the input values are a power of 2
		 * - It then generates a local copy
		 * - Properly orders the data
		*/

		if (!IsPowerOf2(n))
		{
			return -1;
		}

		const float gamma = (uint32_t)log2f(n);

		Complex* W = CalculateTwiddleFactors(n);

		// Initialize loop variables

		// Variables
		float GA = gamma;
		uint32_t mar = n / 2;
		uint32_t gr = 1;

		for (unsigned r = 1; r <= GA; ++r) {
			for (uint32_t g = 0; g < gr; ++g) {
				for (uint32_t m = 0; m < mar; ++m) {
					const uint32_t A_index = m + 2 * mar * g;
					const uint32_t B_index = A_index + n / (1u << r);
					const uint16_t W_index = BitRev(2 * g, GA);

					Complex A = in[A_index];
					Complex B = in[B_index];
					Complex C = W[W_index];

					Complex T =  ComplexMult(B, C);
					in[A_index] = ComplexAdd(A, T);
					in[B_index] = ComplexSubs(A, T);
				}
			}
			mar /= 2;
			gr *= 2;
		}

		// Bit reversal reordering
		for (uint32_t k = 1; k <= n; ++k) {
			uint16_t g = BitRev((unsigned)(k - 1), GA);
			uint16_t I = g + 1;
			if (I > k) {
				Complex temp = in[k - 1];
				in[k - 1] = in[I - 1];
				in[I - 1] = temp;
			}
		}

        free(W);
		return 0;
	}

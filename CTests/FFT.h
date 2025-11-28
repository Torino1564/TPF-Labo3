#include <stdbool.h>
#include <stdint.h>

#define complex_type float
#define scalar_type uint16_t

typedef struct {
    complex_type absolute_value;
    complex_type phase;
} Complex;

Complex ComplexReIm(complex_type re, complex_type im);

complex_type ComplexRe(Complex a);
complex_type ComplexIM(Complex a);
complex_type ComplexAbs(Complex a);
complex_type ComplexPhase(Complex a);
Complex ComplexMult(Complex a, Complex b);
Complex ComplexDiv(Complex a, Complex b);
bool ComplexCompare(Complex a, Complex b, complex_type epsilon);
Complex ComplexMultScalar(Complex a, complex_type b);
Complex ComplexExp(Complex a, complex_type b);

int ComputeFFT(Complex* in, Complex* out, uint32_t n);
int ComputeIFFT(Complex* in, Complex* out, uint32_t n);
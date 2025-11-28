/*
 * DSP.h
 *
 *  Created on: Nov 27, 2025
 *      Author: jtori
 */

#ifndef APP_DSP_H_
#define APP_DSP_H_

#define MAX_BUFFER_SIZE 256

#include "FFT.h"

/*
 * Computes d_t(tau) of a buffer. Assumes that the buffer will be larger or equal to
 * MAX(w + t, w + tau)
 * Returns: true if success
 */

data_t DifferenceFunction(data_t* pIn, uint32_t w, uint32_t t, uint32_t tau);

bool AutocorrelationFunction(data_t* pIn, data_t* pOut, uint32_t w, uint32_t t);

#endif /* APP_DSP_H_ */

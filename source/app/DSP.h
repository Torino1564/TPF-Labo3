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

bool AutocorrelationFunction(data_t* pIn, data_t* pOut, uint32_t w);
bool DifferenceFunction(data_t* pIn, data_t* pOut, uint32_t w);

#endif /* APP_DSP_H_ */

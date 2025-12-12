/*****************************************************************************
  @file     App.c
  @brief    Main Application
  @author   Group 2
  @version  1.0 - coding
 ******************************************************************************/

/*******************************************************************************
 *                                ENCABEZADOS
 ******************************************************************************/

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "hardware.h"
#include "drivers/board.h"
#include "drivers/Timer.h"
#include "drivers/UART.h"
#include "drivers/DMA.h"
#include "drivers/ADC.h"
#include "drivers/gpio.h"
#include "drivers/dac.h"
#include "drivers/CMP.h"
#include "drivers/FTM.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "FIR.h"
#include "FFT.h"
#include "DSP.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/*******************************************************************************
 *                                MACROS
 ******************************************************************************/

#define MAX_STRING_LEN 64

#define SAMPLE_FREQ 5000u
#define SAMPLE_PERIOD_US (float)1000000/(float)SAMPLE_FREQ

#define MAX_TAU 100u
#define WINDOW_SIZE 512
#define ADC_BUFFER_SIZE WINDOW_SIZE
#define FULL_BUFFER_SIZE (ADC_BUFFER_SIZE + MAX_TAU)

#define THRESHOLD 0.15f
#define UPPER_THRESHOLD 0.3f
/*******************************************************************************
 *                                VARIABLES
 ******************************************************************************/

static ADC_Handle adc;
static UART_Handle uart;

static float data[FULL_BUFFER_SIZE];

static uint32_t tau_local_search = 6;

/*******************************************************************************
 *                           	PROTOTIPOS
 ******************************************************************************/


/*******************************************************************************
 *                           	FUNCIONES
 ******************************************************************************/

/* Función de inicialización */
void App_Init (void)
{
	TimerInit();
	// Init ADC
	{
		ADC_Config adc_config = {};
		adc_config.par = 0;
		adc_config.channel = 13;
		adc_config.adcp = PORTNUM2PIN(PB, 3);
		adc_config.contConv = 0;
		adc_config.dmaEnable = 1;
		adc_config.bufferSize = ADC_BUFFER_SIZE;
		adc_config.pitEnable = 1;
		adc_config.sample_period_us = SAMPLE_PERIOD_US;
		adc_config.avgEnable = 0;
		adc_config.avgSet = ADC_Avg4;
		adc_config.resolution = 0;
		adc_config.clkDiv = 3;
		adc_config.adcNum = 0;
		adc = ADC_Init(&adc_config);
	}

	{
		// Inicializacion del UART - FALTA PARIDAD
		UART_Config uart_config = {};
		uart_config.baudRate = 9600;
		uart_config.tx = PORTNUM2PIN(PB, 17);
		uart_config.rx = PORTNUM2PIN(PB, 16);
		uart_config.uartNum = 0;
		uart_config.mode = UART_TRANSCEIVER;
		uart = UART_Init(&uart_config);
	}

	gpioMode(PORTNUM2PIN(PC, 10), OUTPUT);
}

#include "arm_math.h"

void App_Run (void)
{
	size_t size = 0;
	if (ADC_GetBackBufferCopy(adc, (data+MAX_TAU), &size))
	{
		gpioWrite(PORTNUM2PIN(PC, 10), 1);
		float data_out[MAX_TAU] = {};

//		static const float F = 100;
//		for (uint32_t i = 0; i < FULL_BUFFER_SIZE; i++)
//		{
//			data[i] = arm_cos_f32(2*M_PI*i*(F/SAMPLE_FREQ));
//		}

		//ticks start = Now();
		DifferenceFunction(data, data_out, WINDOW_SIZE, MAX_TAU);

		CMNDF(data_out, MAX_TAU);

		// Threshold decide
		uint16_t index = 0;

		uint32_t tau = 0;
		uint32_t tau_threshold = 0;
		float minimum = 999;
		for (uint32_t i = 0; i < MAX_TAU; i++)
		{
			float current = data_out[i];
			if (current < minimum)
			{
				tau = i;
				minimum = current;
			}
			if (current < THRESHOLD)
			{
				tau_threshold = i;

				// Search for the local minimum
				int32_t lower_limit = i - tau_local_search;
				lower_limit = lower_limit < 0 ? 0 : lower_limit;

				int32_t upper_limit = i + tau_local_search;
				upper_limit = upper_limit >= MAX_TAU ? MAX_TAU : upper_limit;
				float local_min = current;

				for (uint32_t j = lower_limit; j < upper_limit; j++)
				{
					if (data_out[j] < local_min)
					{
						local_min = data_out[j];
						tau_threshold = j;
					}
				}
				break;
			}
		}
		if (tau_threshold != 0)
		{
			tau = tau_threshold;
		}
		else if (minimum > UPPER_THRESHOLD)
		{
			tau = 0;
		}

		char buf[MAX_STRING_LEN] = {};
		if (tau != 0)
		{
			uint16_t freq = (uint16_t)(SAMPLE_FREQ / tau);
			sprintf(buf, "%d\n", freq);
		}
		else
		{
			sprintf(buf, "NA\n");
		}

		UART_WriteString(uart, buf);

#define OFFSET ((FULL_BUFFER_SIZE)-MAX_TAU)
		// Guarda los ultimos valores de la tanda nueva al principio del buffer para ser utilizados como datos viejos
		memcpy(data, data+OFFSET, MAX_TAU*sizeof(float));

		gpioWrite(PORTNUM2PIN(PC, 10), 0);
	}
}

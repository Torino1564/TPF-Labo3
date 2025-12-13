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
#include "drivers/Button.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "FIR.h"
#include "FFT.h"
#include "DSP.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "KPS.h"

/*******************************************************************************
 *                                MACROS
 ******************************************************************************/

#define MAX_STRING_LEN 64

#define SAMPLE_FREQ 5000u
#define SAMPLE_PERIOD_US (float)1000000/(float)SAMPLE_FREQ

#define MAX_TAU 200u
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

enum BUTTON_PINS {
	MI = PORTNUM2PIN(PA, 1),
	SI = PORTNUM2PIN(PB, 23),
	SOL = PORTNUM2PIN(PA, 2),
	RE = PORTNUM2PIN(PC, 16),
	LA = PORTNUM2PIN(PC, 17),
	MI6 = PORTNUM2PIN(PB, 9)
};

static uint32_t notes[] = {
	E1, B2, G3, D4, A5, E6
};

#define STRINGS 6

uint16_t buttonIds[STRINGS] = {0};

/*******************************************************************************
 *                           	PROTOTIPOS
 ******************************************************************************/

/*******************************************************************************
 *                           	FUNCIONES
 ******************************************************************************/

/* Función de inicialización */
void App_Init (void)
{
	// Systick Init
	TimerInit();
	gpioInitInterrupts();

	// Init ADC
	{
		ADC_Config adc_config = {};
		adc_config.par = 0;
		adc_config.channel = 1;
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

	// Init Buttons
	buttonIds[0] = NewButton(MI, 0);
	buttonIds[1] = NewButton(SI, 0);
	buttonIds[2] = NewButton(SOL, 0);
	buttonIds[3] = NewButton(RE, 0);
	buttonIds[4] = NewButton(LA, 0);
	buttonIds[5] = NewButton(MI6, 0);

	gpioMode(PORTNUM2PIN(PC, 10), OUTPUT);

	// Init KPS
	{
		KPS_Config cfg = {
				.sample_frequency = 20000,
				.feedback_const1 = 500,
				.feedback_const2 = 500
		};
		KPS_Init(&cfg);
	}
}

#include "arm_math.h"

void App_Run (void)
{
	// DSP
	size_t size = 0;
	if (ADC_GetBackBufferCopy(adc, (data+MAX_TAU), &size))
	{
		gpioWrite(PORTNUM2PIN(PC, 10), 1);
		float data_out[MAX_TAU] = {};

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
			if (current < 0)
				continue;
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
		float real_tau = tau;
		// Quadratic interpolation
		if (tau != 0 && tau != (MAX_TAU-1))
		{
			float x_offset = 0;
			quadratic_interp_min_x(data_out[tau-1], data_out[tau], data_out[tau+1], &x_offset);
			real_tau += x_offset;
		}

		char buf[MAX_STRING_LEN] = {};
		if (tau != 0)
		{
			uint16_t freq = (uint16_t)((float)SAMPLE_FREQ / real_tau);
			sprintf(buf, "%d\n\r", freq);
		}
		else
		{
			sprintf(buf, "NA\n\r");
		}

		UART_WriteString(uart, buf);

#define OFFSET ((FULL_BUFFER_SIZE)-MAX_TAU)
		// Guarda los ultimos valores de la tanda nueva al principio del buffer para ser utilizados como datos viejos
		memcpy(data, data+OFFSET, MAX_TAU*sizeof(float));

		gpioWrite(PORTNUM2PIN(PC, 10), 0);
	}


	for (uint32_t i = 0; i < STRINGS; i++)
	{
		if (readButtonStatus(buttonIds[i]))
		{
			readButtonData(buttonIds[i]);
			KPS_SendNote(notes[i]);
		}
	}

}

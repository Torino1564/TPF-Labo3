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

#define SAMPLE_FREQ 2000u
#define SAMPLE_PERIOD_US (float)1000000/(float)SAMPLE_FREQ

#define MAX_TAU 40u
#define WINDOW_SIZE 256
#define ADC_BUFFER_SIZE WINDOW_SIZE
#define FULL_BUFFER_SIZE (ADC_BUFFER_SIZE + MAX_TAU)

#define THREHSOLD 0.1
/*******************************************************************************
 *                                VARIABLES
 ******************************************************************************/

static ADC_Handle adc;
static UART_Handle uart;

static float data[FULL_BUFFER_SIZE];

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
}
#define TEMPORAL_SMOOTHING_SAMPLES 40
float tau_data[TEMPORAL_SMOOTHING_SAMPLES][MAX_TAU];
uint16_t temporal_buffer_index = 0;

void App_Run (void)
{
	size_t size = 0;
	if (ADC_GetBackBufferCopy(adc, (data+MAX_TAU), &size))
	{
		float (*data_out)[MAX_TAU] = &tau_data[temporal_buffer_index++];
		ticks start = Now();
		DifferenceFunction(data, *data_out, WINDOW_SIZE, MAX_TAU);

		CMNDF(*data_out, MAX_TAU);

		// Threshold decide



		for (uint32_t i = 0; i < MAX_TAU; i++)
		{

		}

		// Guarda los ultimos valores de la tanda nueva al principio del buffer para ser utilizados como datos viejos
		memcpy(data, data+WINDOW_SIZE, MAX_TAU*sizeof(float));
		volatile ticks ms = Now() - start;
		__NOP();

		temporal_buffer_index %= TEMPORAL_SMOOTHING_SAMPLES;
	}
}

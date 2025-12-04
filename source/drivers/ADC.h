/*
 * ADC.h
 *
 *  Created on: 28 oct. 2025
 *      Author: plaju
 */

#ifndef DRIVERS_ADC_H_
#define DRIVERS_ADC_H_

/*******************************************************************************
								INCLUDES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"
#include "hardware.h"

/*******************************************************************************
								DEFINICIONES
 ******************************************************************************/

enum ADC_Mode {
	SINGLE_ENDED,
	DIFFERENTIAL
};

enum ADC_Resolution {
	LOW_RESOLUTION,
	HIGH_RESOLUTION,
	MED_RESOLUTION,
	UHIGH_RESOLUTION
};

typedef enum {
	ADC_Avg4 	= 0b00,
	ADC_Avg8 	= 0b01,
	ADC_Avg16 	= 0b10,
	ADC_Avg32	= 0b11
} ADC_Average;

typedef int8_t ADC_Handle;

/* Cosas que son configurables del ADC
 * pines de entrada:
 * Resolucion: 	puede ser de 8, 10, 12 o 16 bits para single ended o 9 , 11, 13 o 16 bits
 * 				para differential
 * Modo:		diferencial / single ended
 * Instancia:	la kinetis posee 2 instancias: ADC0 y ADC1
 * Par o canal:	single 24 canales, diff 4 canales - hay una banda de canales >:$
 * Conversion:  singular / continua
 *
 * */

typedef struct {
	// config de registros
	ADC_Type* pRegisters;	// puntero para llegar a los registros
	bool defaultConfig;
	uint8_t resolution:2;	// 0=8bit,1=12bit,2=10bit,3=16bit
	uint8_t mode:1;			// 0 = single-ended, 1 = diferencial
	uint8_t par:1;			// 0 = MUX A, 1 = MUX B
	uint8_t channel;		// Canal (0–23)
	uint8_t clkDiv:2;		// divisor (0=1,1=2,2=4,3=8)
	uint8_t clkSel:2;		// 00 bus clk, 01 bus/2, 10 alt clk, 11 asy clk
	// reg SC2
	uint8_t refVol:1;		// 0 3.3v y 0v, 1 para otros
	// reg SC3
	uint8_t calEnable:1;	// 1 para habilitar la calibracion
	uint8_t contConv:1;		// 0 for a single conversion, 1 for continous conversion
	uint8_t avgEnable:1;	// 1 si se desea que la conversion se obtenga de un promedio de samples
	ADC_Average avgSet:2;		// 00 4 samples, 01 8 samples, 10 16 samples, 11 32 samples; c/s ocupa un ciclo de ADC clock
	uint32_t sample_period_us;
	// instancia
	uint8_t adcNum;			// number of adc instance to be used
	uint32_t bufferSize;

	// pit
	bool pitEnable;			// usar pit
	uint8_t pitNum;			// Pit number

	// dma
	bool dmaEnable;			//

	// pines
	pin_t adcp;				// si modo en single-ended, se usa el adcp como el input
	pin_t adcn;				// si es modo diff, adcp y adcn son usados de forma diff

	// callback
	void (*callback)(void); // cuando termina la conversion llama al callback
} ADC_Config;

/*******************************************************************************
								FUNCIONES
 ******************************************************************************/

// inicializacion
ADC_Handle ADC_Init(ADC_Config* pConfig);
bool ADC_GetBackBuffer(uint8_t adcNum, uint16_t** pBuffer, size_t* size);
bool ADC_GetBackBufferCopy(uint8_t adcNum, float* pBuffer, size_t* size);

// para realizar una sola conversion
void ADC_SingleRead(uint8_t adcNum);

#endif /* DRIVERS_ADC_H_ */

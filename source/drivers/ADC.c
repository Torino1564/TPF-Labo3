/*
 * ADC.c
 *
 *  Created on: 28 oct. 2025
 *      Author: plaju
 */


/* Registers Description
 *
 * ADCx_SC1: status ang control 1
 * 		bit 7 	- COCO: Conversion Complete Flag
 * 		bit 6 	- AIEN: Interrupt Enable - 1 enable
 * 		bit 5 	- DIFF: Differential Mode Enable - 1 to differential
 * 		bit 4:0 - ADCH: Input Channel Select:
 * 			# con DIFF = 0: del 0 al 23 selecciona que par del modulo utiliza.
 * 			# con DIFF = 1:
 * 			11000, 11001 y 11100 estan reservados (24, 25 y 28)
 * 			11010: sensor de temperatura
 * 			11011: bandgap
 * 			11101 y 11110 casos especiales
 * 			11111: modulo deshabilitado
 *
 * ADCx_CFG1: configuration 1
 * 		bit 7 	- ADLPC: Low-Power Config - 0 normal / 1 lowpower at expense of clock speed
 * 		bit 6:5	- ADIV:	 Clock Divide Select - 00 = 1, 01 = 2, 10 = 4 y 11 = 8.
 * 						clock rate = input clock/adiv
 * 		bit 4	- ADLSMP: Sample Time Config: 0 short sample time / 1 long
 * 		bit 3:2 - MODE: resolution 00 low, 11 UHIGH
 * 		bit 1:0 - ADICLK: Input Clock Select -
 * 			00 : bus clock
 * 			01 : bus clock divided by 2
 * 			10 : alternate clock
 * 			11 : asynchronous clock
 *
 * ADCx_CFG2: configuration 2
 * 		bit 4	- MUXSEL: para seleccionar canal A o B
 * 		bit 3	- ADACKEN:
 * 		bit 2 	- ADHSC: 0 normal conversion, 1 highspeed conversion. Highpeed conversion
 * 					add 2 ADCLK cycles to the total conversion time
 * 		bit 1:0 -
 *
 * ADCx_Rn: guarda el valor convertido. Si es single guarda 0 para rellenar, si es diff
 * 			guarda sign bits para rellenar.
 *
 * 		bit15:0	- data
 *
 * CLOCK INTERNO RECOMENDADO:
 * En la datasheet el fabricante recomienda que el clock interno del ADC se encuentre
 * entre los valores de 2MHz y 18MHz para garantizar precision y linealidad.
 * El clock interno es el que se obtiene con CFG1.ADVCLK / CFG1.ADIV
 *
 * SPEED OF CONVERSION:
 * El apartado ADLSMP del registro CFG1 sirve para setear el tiempo de sample: short o
 * long sample time.
 *
 * 0 = Normal: 	8 bits de resolucion 	-> 9 ciclos de clock
 * 				10 bits de resolucion 	-> 13 ciclos de clock
 * 				12 bits de resol		-> 15 ciclos de clock
 * 				16 bits de resol		-> 25 ciclos de clock
 *
 * 1 = Hi-Speed:8 bits de resolucion 	-> 13 ciclos de clock
 * 				10 bits de resolucion 	-> 25 ciclos de clock
 * 				12 bits de resol		-> 25 ciclos de clock
 * 				16 bits de resol		-> 41 ciclos de clock
 *
 * Conociendo la cantidad de ciclos se puede conocer el periodo de muestreo
 * */

#include "ADC.h"
#include "DMA.h"
#include <stdlib.h>
#include "hardware.h"

/*******************************************************************************
								DEFINICIONES
 ******************************************************************************/

#define MAX_ADC_INSTANCES 2
#define TRUE 1
#define FALSE 0
#define DEFAULT_BUFFER_SIZE 128

/*******************************************************************************
								STRUCTS
 ******************************************************************************/

typedef struct {
	ADC_Config config;			// all the configuration of the instance
	DMA_Channel dma;

	// buffers
	uint16_t* pBuffers[2];
	bool bufSel;				// el buffer seleccionado es el que está siendo completado con samples
	uint16_t bufLen;
	uint16_t bufIndex;
	bool newBufData;
}ADC;

/*******************************************************************************
								VARIABLES
 ******************************************************************************/

static ADC * pInstances[MAX_ADC_INSTANCES];

/*******************************************************************************
								PROTOTIPO
 ******************************************************************************/

void ADC_InitDMA(uint8_t adcNum);
void ADC_MajorLoopCallback(void* pData);

/*******************************************************************************
								FUNCIONES
 ******************************************************************************/

ADC_Handle ADC_Init(ADC_Config * pConfig)
{
	// 1 - valor de instancia válido?
	if(!(pConfig->adcNum == 0 || pConfig->adcNum == 1))
	{
		return -1;	// negativo -> devuelvo error
	}

	// 2 - pregunto sobre la inicializacion de la instancia
	if(pInstances[pConfig->adcNum] != 0)
	{
		return -1;	// instancia ya inicializada
	}

	// 3 - creo en el heap la instancia del adc y la guardo en el arreglo de instancias
	pInstances[pConfig->adcNum] = (ADC*)calloc(1, sizeof(ADC));

	// 4 - tomo el puntero a la instancia para manejar
	ADC * adc = pInstances[pConfig->adcNum];

	// 5 - guardo el pConfig en el puntero de config de la instancia para hacer la configuracion
	adc->config = *pConfig;

	// 6 - uso una variable para que el codigo sea mas legible
	ADC_Config * cfg = &adc->config;

	// 7 - dependiendo de la instancia hay que:
	if(cfg->adcNum == 0)
	{
		// 7.1 - guardar el puntero a los registros
		cfg->pRegisters = ADC0;

		// 7.2 - hacer el clock gating
		SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

		// 7.3 - activar las interrupciones
		NVIC_EnableIRQ(ADC0_IRQn);
	}
	else if(cfg->adcNum == 1)
	{
		// 7.1 - guardar el puntero a los registros
		cfg->pRegisters = ADC1;

		// 7.2 - hacer el clock gating
		SIM->SCGC3 |= SIM_SCGC3_ADC1_MASK;

		// 7.3 - activar las interrupciones
		NVIC_EnableIRQ(ADC1_IRQn);
	}

	// 8 - pin configuration - en este apartado se asume que el pConfig fue enviado correctamente
	if(cfg->mode == SINGLE_ENDED)
	{
		gpioMux(cfg->adcp, ALT0);
	}
	else
	{
		gpioMux(cfg->adcp, ALT0);
		gpioMux(cfg->adcn, ALT0);
	}

	// 9 - la configuracion es por default?
	if(cfg->defaultConfig)
	{
		// por defecto se trabaja en single ended
		cfg->mode = SINGLE_ENDED;
		cfg->adcn = 0;							// por defecto no se usa este pin
		// reg CFG1
		cfg->clkSel = 0b00;						// clock base por defecto en clock cpu
		cfg->clkDiv = 0b11;						// clock divider en 8 -> un clock del adc de 12,5 MHz
		cfg->resolution = MED_RESOLUTION;		// resolucion por defecto en la maxima, 16bits
		// reg SC2
		cfg->refVol = 0;						// voltajes de referencia 3,3V y 0V
		// reg SC3
		cfg->calEnable = 0;						// sin calibracion
		//cfg->contConv = 0;					// conversion simple
		//cfg->bufferSize = DEFAULT_BUFFER_SIZE;	// 128 uint16_ts
	}

	// 10 - completo el registro de configuracion 1: resolucion, clock select y clock divider
	cfg->pRegisters->CFG1 = ADC_CFG1_MODE(cfg->resolution) | ADC_CFG1_ADICLK(cfg->clkSel) | ADC_CFG1_ADIV(cfg->clkDiv);

	// 11 - registro de configuracion - CFG2: MUX si es canal a o b
	cfg->pRegisters->CFG2 = ADC_CFG2_MUXSEL(cfg->par);

	// 12 - registro de referencia de voltaje
	cfg->pRegisters->SC2 = ADC_SC2_REFSEL(cfg->refVol);

	// 13 - reg config 3: calibration, continous conversion, average enable and average set
	cfg->pRegisters->SC3 = ADC_SC3_CAL(cfg->calEnable) | ADC_SC3_ADCO(cfg->contConv)
						| ADC_SC3_AVGE(cfg->avgEnable) | ADC_SC3_AVGS(cfg->avgSet);

	// 14 - SC1: canal y modo de comparacion
	cfg->pRegisters->SC1[cfg->par] = ADC_SC1_DIFF(cfg->mode) | ADC_SC1_ADCH(0b11111);

	// 15 - asignacion de buffer
	adc->pBuffers[0] = (uint16_t*)calloc(cfg->bufferSize * 2, sizeof(uint16_t));
	adc->pBuffers[1] = &adc->pBuffers[0][cfg->bufferSize];
	adc->bufLen = cfg->bufferSize;
	adc->bufIndex = 0;

	// 16 - hay que activas las interrupciones
	if(cfg->dmaEnable)
	{
		cfg->pRegisters->SC2 |= ADC_SC2_DMAEN(1);
		ADC_InitDMA(cfg->adcNum);
	}
	else
	{
		cfg->pRegisters->SC1[cfg->par] |= ADC_SC1_AIEN(1);
	}

	if (cfg->pitEnable)
	{
		SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
		PIT->CHANNEL[cfg->adcNum].LDVAL = (uint32_t)(cfg->sample_period_us * 1000u / 20u);
		PIT->CHANNEL[cfg->adcNum].TCTRL |= PIT_TCTRL_TEN(1);
		cfg->pRegisters->SC2 |= ADC_SC2_ADTRG(1); //  Trigger por hardware
		if (cfg->adcNum == 0)
			SIM->SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0x4 + cfg->pitNum) | SIM_SOPT7_ADC0ALTTRGEN(1);
		else
			SIM->SOPT7 |= SIM_SOPT7_ADC1TRGSEL(0x4 + cfg->pitNum);

	}

	// Enable module
	adc->config.pRegisters->SC1[0] = ADC_SC1_ADCH(adc->config.channel);

	//adc->config.pRegisters->SC1[0] |= ADC_SC1_AIEN(1);

	return adc->config.adcNum;
}


void ADC_SingleRead(uint8_t adcNum)
{
	ADC * adc = pInstances[adcNum];
	while(adc->config.pRegisters->SC2 & ADC_SC2_ADACT_MASK)
	{
		__NOP();
	}
	adc->config.pRegisters->SC1[adc->config.par] = (adc->config.pRegisters->SC1[adc->config.par] & ~ADC_SC1_ADCH_MASK) | ADC_SC1_ADCH(adc->config.channel);

}

void ADC_SwapBufferCallback(void* pData)
{
	uint8_t adcNum = *(uint8_t*)pData;
	ADC* pADC = pInstances[adcNum];

	pADC->bufSel = !pADC->bufSel;
	pADC->newBufData = 1;
}

void ADC_HalfLoopCallback(void* pData)
{
	ADC_SwapBufferCallback(pData);
}

void ADC_MajorLoopCallback(void* pData)
{
	ADC_SwapBufferCallback(pData);
}

void ADC_InitDMA(uint8_t adcNum)
{
	ADC* pADC = pInstances[adcNum];

	DMA_Config dma_config = {};
	dma_config.trigger = TRUE;
	//dma_config.source = 58;
	dma_config.source = adcNum == 0 ? 40 : 41;
	dma_config.trigger_period_us = pADC->config.sample_period_us;														//83u = 12KHz
	dma_config.skipPITInitialization = 1;
	dma_config.source_address = (uint32_t)&(pADC->config.pRegisters->R[0]);	// hay que ver el tema de R0
	dma_config.destination_address = (uint32_t)pADC->pBuffers[pADC->bufIndex];
	dma_config.source_transfer_data_size = DMA_TDS_16bit;
	dma_config.destination_transfer_data_size = DMA_TDS_16bit;
	dma_config.minor_loop_num_bytes = 2;
	dma_config.source_minor_loop_enabled = FALSE;
	dma_config.destination_minor_loop_enabled = TRUE;
	dma_config.minor_loop_offset = 2;
	dma_config.source_major_loop_completion_offset = 0;
	dma_config.destination_major_loop_completion_offset = -((pADC->config.bufferSize * 2 * sizeof(uint16_t))-2);
	dma_config.major_loop_number_iterations = pADC->config.bufferSize * 2; // This is twice the size for the ping pong bufer
	dma_config.enable_finish_irq = 1;
	dma_config.finish_loop_callback = &ADC_MajorLoopCallback;
	dma_config.finish_irs_user_data = (void*)&pADC->config.adcNum;
	dma_config.enable_half_irq = 1;
	dma_config.half_loop_callback = &ADC_HalfLoopCallback;
	dma_config.half_irs_user_data = (void*)&pADC->config.adcNum;

	pADC->dma = DMA_InitChannel(&dma_config);

	return;
}

bool ADC_GetBackBuffer(uint8_t adcNum, uint16_t** pBuffer, size_t* size)
{
	ADC* pADC = pInstances[adcNum];
	if(!(pADC->newBufData))
	{
		*pBuffer = 0;
		*size = 0;
		return false;
	}
	pADC->newBufData = 0;	// apagit
	*pBuffer = pInstances[adcNum]->pBuffers[pADC->bufSel];
	*size = pADC->config.bufferSize;

	return true;
}

bool ADC_GetBackBufferCopy(uint8_t adcNum, float* pBuffer, size_t* size)
{
	ADC* pADC = pInstances[adcNum];
	if(!(pADC->newBufData))
	{
		*size = 0;
		return false;
	}
	pADC->newBufData = 0;	// apagit

	*size = pADC->config.bufferSize;
	uint16_t* pInternalBuffer = pInstances[adcNum]->pBuffers[pADC->bufSel];

	float resolution = 0;
	switch (pInstances[adcNum]->config.resolution)
	{
	case 0:
		resolution = 1 << 8;
		break;
	case 1:
		resolution = 1 << 12;
		break;
	case 2:
		resolution = 1 << 10;
		break;
	case 3:
	default:
		resolution = 1 << 16;
		break;
	}

	resolution = 1/resolution;

	for (uint32_t i = 0; i < *size; i++)
	{
		pBuffer[i] = pInternalBuffer[i] * resolution ;
	}

	return true;
}

void ADCx_IRQHandler(uint8_t adcNum)
{
	// cada vez que entra acá es porque terminó una conversión
	// creo que lo conveniente sería, leer el valor convertido y ademas, llamar a
	// algun callback guardado en la instancia

	ADC * adc = pInstances[adcNum];

	gpioToggle(PORTNUM2PIN(PC, 16));
	uint16_t val = adc->config.pRegisters->R[adc->config.par];

	if (adc->bufIndex < adc->bufLen)
	{
		adc->pBuffers[!adc->bufSel][adc->bufIndex++] = val;
	}
	else if (adc->bufIndex == adc->bufLen)
	{
		adc->bufIndex = 0;
		adc->bufSel = !adc->bufSel;
		adc->newBufData = TRUE;
	}

	return;
}

void ADC0_IRQHandler(void)
{
	ADCx_IRQHandler(0);
}

void ADC1_IRQHandler(void)
{
	ADCx_IRQHandler(1);
}

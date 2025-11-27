/*
 * CMP.c
 *
 *  Created on: 4 nov. 2025
 *      Author: plaju
 */


/*
 * REGISTROS:
 *
 * CMPx_CR0: registro de control 0
 *
 * 		bit 6-4: FILTER_CNT - Filter Sample Count: el numero de samples consecutivas que deben coincidir para que se tome
 * 						como nueva salida del comparador.
 * 						- 000: Filter Disable. CR0[FILTER_CNT] = 0 & CR1[SE] = 1 no son compatibles!!
 * 						- 001 - 111: cantidad de samples
 * 		bit 1-0: HYSTCTR - Comparator Hysteresis Control: define el nivel de histeresis. Se puede setear del nivel 0 al 3,
 * 						cada nivel es especifico del device. Hay que ver la datasheet para los valores exactos.
 * 						- 00 - 11: nivel 0 - 3
 *
 * CMPx_CR1: registro de control 1
 *
 * 		bit 7: SE - Sample Enable: habilita la posibilidad de tomar muestras para tomar una decision del comparador.
 * 				Si en cualquier momento se desean setear ambos CR1[SE] & CR1[WE], se va a setear
 * 				el CR1[SE] y se va a clearear el CR1[SE].
 * 				0 - sampling mode is not selected - modo continuo: el comparador evalua continuamente sus entradas
 * 				1 - sampling mode is selected
 *		bit 6: WE - Windowing Enable: habilita la posiblidad de usar el comparador en modo ventana. Esto es utilizar dos
 *				comparadores para definir un rango de tensión. Un comparador detecta si la señal es
 *				mayor que un umbral inferior y el otro comparador detecta si la señal es menor
 *				que un umbral superior. Combinados sabes si la señal se encuentra "dentro de la ventana"
 *				0 - windowing mode is not selected
 *				1 - windowing mode is selected.
 *		bit 4: PMODE - Power Mode Select: con este apartado se puede elegir el modo de consumo del periferico
 *				0 - Low Speed Comparison -> CMP slower propagation delay and lower current consumption
 *				1 - High Speed Comparison -> CMP faster output propagatiuon delay and higher current.
 *		bit 3: INV - comparator output polarity invertion: con este apartado se puede dar vuelta la salida del comparador
 *				significando que:
 *				0 - si gana la pata mas -> la salida es 1 logico
 *				1 - si gana la pata menos -> la salida es un 1 logico
 *		bit 2: COS - Comparator Output Select: se puede elegir si el filtro es bypassed o no
 *				0 - la salida es filtrada
 *				1 - la salida es la señal sin filtrar
 *		bit 1: OPE - Output Pin Enable: the comparator needs to own the pin, unless this bit has no effect
 *				0 - CMPO is not available
 *				1 - CMPO is available
 *		bit 0: EN - Comparator Module Enable: sirve para deshabilitar el modulo, esto implica que el comparador analogico
 *				se encuenta apagado -> no consume potencia. Por otro lado, si en la multiplexacion de las entradas al
 *				comparador analogico se multiplexa tanto la pata positiva como la negativa al mismo input, entonces
 *				la deshabilitacion se hace de forma automática.
 *				0 - Analog comparator is disabled
 *				1 - Analog comparator is enabled
 *
 * CMPx_FPR: registro del periodo de sampleo
 * 		bit 7-1: FILT_PER - Filter Sample Period: este registro configura el periodo de muestreo del filtro. NO EL PERIODO
 * 				DE SAMPLEO DEL COMPARADOR!
 * 				- Si CR1[SE] = 0: modo de comparación continuo: si el filtro está activado, este registro define cada
 * 				cuantos ciclos de bus clock el filtro evalua la salida del comparador.
 * 				- Si CR1[SE] = 1: modo de comparacion por samples: el periodo de samples se toma de una señal externa y el
 * 				filtro no usa su muestreo interno.
 *
 * CPMx_SCR: registro de status y control
 * 		bit 6: DMAEN - DMA Enable Control: el DMA es triggereado cuando CFR o CFF is set
 * 				0 - DMA is disabled
 * 				1 - DMA is enabled
 * 		bit 4: IER - Comparator Interrupt Enable Rising : interrupt enable, triggereado cuando CFR is set
 * 				0 - Interrupt rising is disabled
 * 				1 - Interrupt rising is enabled
 * 		bit 3: IEF - Comparator Interrupt Enable Falling: interrupt enable, triggereado cuando CFF is set
 * 				0 - Interrupt falling is disabled
 * 				1 - Interrupt falling is enabled
 * 		bit 2: CFR - Analog Comparator Flag Rising: detecta un rising edge en la salida COUT
 * 		bit 1: CFF - Analog Comparator Flag Falling: detecta un falling edge en la salida COUT
 * 		bit 0: COUT - Analog Comparator Output:
 *
 * CPMx_DACCR: registro del control del DAC - el comparador posee un DAC de 6bits
 * 		bit 7: DACEN - DAC Enable: habilita el DAC
 * 				0 - DAC is disabled
 * 				1 - DAC is enabled
 * 		bit 6: VRSEL - Supply Voltage Reference Source Select
 * 				0 - Vin1 is selected as resistor ladder network supply reference
 * 				1 - Vin2 is selected instead
 * 		bit 5-0: VOSEL - DAC Output Voltage Select: selecciona un nivel de salida de 1 a 64 distintos niveles
 *
 * CPMx_MUXCR: registro de control de mux - controla las entradas al comparador
 * 		bit 7: PSTM - Pass Through Mode Enable: habilita el pass through mode
 * 		bit 5-3: PSEL - Plus Input Mux Control: determina que input is selected para la entrada positiva del comparador
 * 		bit 2-0: MSEL - Minus Input Mux Control: determina que input is selected para la entrada negativa del comparador
 * 				000-111 -> IN0-IN7 - ver la datasheet
 *
 *				CMP0:	PC6 -> IN0
 *						PC7 -> IN1
 *						PC8 -> IN2
 *						PC9 -> IN3
 *				CMP1:	PC2 -> IN0
 *						PC3 -> IN1
 *				CMP2:	PA12 -> IN0
 *						PA13 -> IN1
 * */

/*******************************************************************************
								INCLUDES
 ******************************************************************************/

#include <stdlib.h>
#include "CMP.h"
#include "gpio.h"
#include "hardware.h"

/*******************************************************************************
								MACROS
 ******************************************************************************/

#define MAX_CMP_INSTANCES 3
#define TRUE 1
#define FALSE 0

#define BUS_CLOCK 50000000u

/*******************************************************************************
								STRUCTS
 ******************************************************************************/

typedef struct{
	CMP_Config config;
	CMP_Type * regs;
}CMP;

/*******************************************************************************
								VARIABLES
 ******************************************************************************/

static CMP_Type * pRegisters[MAX_CMP_INSTANCES] = CMP_BASE_PTRS;

static IRQn_Type pInterrupts[MAX_CMP_INSTANCES] = CMP_IRQS;

static CMP * pInstances[MAX_CMP_INSTANCES] = {};


 /*******************************************************************************
 								FUNCIONES
  ******************************************************************************/

CMP_Handle CMP_Init(CMP_Config * config)
{
	// pregunto si el numero de la instancia es valido
	if(!(config->cmpNum >= 0 && config->cmpNum <= MAX_CMP_INSTANCES))
	{
		return -1;
	}

	// pregunto si ya esta inicializado
	if(pInstances[config->cmpNum] != 0)
	{
		return -1;
	}

	// guardo en el heap
	pInstances[config->cmpNum] = (CMP*)calloc(1, sizeof(CMP));

	// uso una variable para agilizar la programacion
	CMP* cmp = pInstances[config->cmpNum];

	// los valores del puntero de config recibido los guardo en mi static en heap
	cmp->config = *config;

	// uso una variable para agilizar la programacion
	CMP_Config * cfg = &cmp->config;

	// guardo en la struct el puntero hacia los registros
	cmp->regs = pRegisters[cfg->cmpNum];

	// uso una variable para agilizar la programacion

	CMP_Type* regs = cmp->regs;

	// hago el clock gating de la instancia
	SIM->SCGC4 |= SIM_SCGC4_CMP_MASK;

	switch(cfg->mode)
	{
	case CMP_INPUT_CROSSING_MODE:
		/* Cuando elegimos este modo queremos que la salida del comparador tenga flancos cada vez que el input positivo pasa
		 * a través de un valor fijo que se introduce en el input negativo.
		 * */

		// momentaneamente uso una variable para config el gpio de entrada
		pin_t cmp_input_pairs[MAX_CMP_INSTANCES][2] =  {{PORTNUM2PIN(PC,8), PORTNUM2PIN(PC,7)}, 	// CMP0 {IN0, IN1}
														{PORTNUM2PIN(PC,2), PORTNUM2PIN(PC,3)},		// CMP1 {IN0, IN1}
														{PORTNUM2PIN(PA,12), PORTNUM2PIN(PA,13)}};	// CMP2	{IN0, IN1}
		// registro MUXCR
		cfg->positive_input = IN2;
		cfg->negative_input = IN1;

		// configuro como entrada al CMPx
		gpioMux(cmp_input_pairs[cfg->cmpNum][0], ALT0);
		gpioMux(cmp_input_pairs[cfg->cmpNum][cfg->negative_input], ALT0);

		// momentaneamente uso una variable para config el gpio de salida
		pin_t cmp_output[MAX_CMP_INSTANCES] = { PORTNUM2PIN(PC, 5), PORTNUM2PIN(PC, 4), 0};

		// configuro como salida del CMPx
		if(cfg->analog_comp_output_enable)
		{
			gpioMux(cmp_output[cfg->cmpNum], ALT6);
		}
		break;

	default:	// el resto de modos no está implementado
		break;
	}


	// hay que habilitar las interrupciones?
	if(cfg->falling_int_enable || cfg->rising_int_enable)
	{
		// habilito las interrupciones de la instancia
		NVIC_EnableIRQ(pInterrupts[cfg->cmpNum]);
	}

	// registro CR0
	regs->CR0 = CMP_CR0_FILTER_CNT(cfg->filter_sample_count)	//
				|CMP_CR0_HYSTCTR(cfg->hysteresis_level);		//

	// registro CR1
	regs->CR1 = CMP_CR1_EN(cfg->analog_comp_enable)				//
				|CMP_CR1_OPE(cfg->analog_comp_output_enable)	//
				|CMP_CR1_COS(!cfg->filter_enable)				//
				|CMP_CR1_INV(cfg->invert_polarity)				//
				|CMP_CR1_PMODE(cfg->power_mode)					//
				|CMP_CR1_WE(cfg->windowing_enable)				//
				|CMP_CR1_SE(cfg->sampling_enable);				//

	// registro FPR
	regs->FPR = CMP_FPR_FILT_PER(cfg->filter_sampling_period_us * 1000 / 20u);	//

	// registro SCR
	regs->SCR = CMP_SCR_DMAEN(cfg->dma_enable)					//
				|CMP_SCR_IER(cfg->rising_int_enable)			//
				|CMP_SCR_IEF(cfg->falling_int_enable);			//

	// registro DACCR
	regs->DACCR = CMP_DACCR_DACEN(cfg->adc_enable)				//
				|CMP_DACCR_VRSEL(cfg->adc_output)				//
				|CMP_DACCR_VOSEL(cfg->adc_voltage_ref);			//

	// registro MUXCR
	regs->MUXCR = CMP_MUXCR_PSTM(cfg->pass_through_mode)
				|CMP_MUXCR_PSEL(cfg->positive_input)
				|CMP_MUXCR_MSEL(cfg->negative_input);

	uint8_t reg = regs->MUXCR;
	(void)reg;
	return config->cmpNum;
}










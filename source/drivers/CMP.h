/*
 * CMP.h
 *
 *  Created on: 4 nov. 2025
 *      Author: plaju
 */

#ifndef DRIVERS_CMP_H_
#define DRIVERS_CMP_H_

/*******************************************************************************
								INCLUDES
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
								DEFINICIONES
 ******************************************************************************/

enum CMP_INPUTS{
	IN0,
	IN1,
	IN2,
	IN3,
	IN4,
	IN5,
	IN6,
	IN7
};

enum CMP_FUNCTIONAL_MODES {
	CMP_DISABLED,
	CMP_CONTINOUS_MODE,
	CMP_CROSSING_MODE,
	CMP_INPUT_CROSSING_MODE
};	// existen mas, pero no se, puse algunas

typedef int8_t CMP_Handle;

typedef struct {
	uint8_t cmpNum;
	uint8_t mode;
	double crossing_value;

	// registro CR0
	uint8_t filter_sample_count:3;		// CR0[FILTER_CNT]
	uint8_t	hysteresis_level:2;			// CR0[HYSTCTR]

	// registro CR1
	bool sampling_enable;				//
	bool windowing_enable;				//
	bool power_mode;					// 0 low speed, low power, 1 high speed, high power
	bool invert_polarity;				// 0 para polaridad normal, 1 para invertida
	bool filter_enable;					// 0 para saltear el filtro, 1 para no saltearlo
	bool analog_comp_output_enable;		// 0 CMPO not enabled, 1 CMPO enabled
	bool analog_comp_enable;			// 0 para apagar el comparador, es decir, no consume potencia

	// registro FPR
	uint8_t filter_sampling_period_us;		// DEL FILTRO NO DEL COMP

	// registro SCR
	bool dma_enable;
	bool rising_int_enable;
	bool falling_int_enable;

	// registro DACCR
	bool adc_enable;
	bool adc_voltage_ref;
	uint8_t adc_output;					// resolucion de 6bits

	// registro MUXCR
	bool pass_through_mode;				// en 1 pone la pata positiva directamente a la salida, y la pata negativa no participa
	uint8_t positive_input;
	uint8_t negative_input;

}CMP_Config;

/*******************************************************************************
								FUNCIONES
 ******************************************************************************/

CMP_Handle CMP_Init(CMP_Config * config);

void CMP_ChangeMode(uint8_t mode);

#endif /* DRIVERS_CMP_H_ */

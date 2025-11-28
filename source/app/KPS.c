/*
 * KPS.c
 *
 *  Created on: 27 nov. 2025
 *      Author: plaju
 */

/****************************HEADERS****************************/

#include "KPS.h"
#include "drivers/Timer.h"
#include <stdlib.h>

/****************************MACROS****************************/

#define MAX_DELAY 4096

/****************************ESTRUCTURAS****************************/

typedef struct{
	KPS_Config kps_config;

	// buffer param
	uint16_t kps_index;
	uint16_t kps_order;

	// timer param
	service_id timer;
}KPS;

/****************************VARIABLES****************************/

KPS * pKPS = NULL;
int16_t kps_buffer[MAX_DELAY];

/***********************FUNCIONES PRIV****************************/

void KPS_Processing(void*);

/**************************FUNCIONES*****************************/

int8_t KPS_Init(KPS_Config * config)
{
	if(pKPS != NULL)
	{
		return -1;	// ya inicializado
	}

	// guardo en el heap un espacio para la config del KPS
	pKPS = (KPS*)calloc(1, sizeof(KPS));

	// guardo la config
	pKPS->kps_config = *config;

	// punero a void auxiliar
	void* aux;

	// registro un timer con frecuencia de sampleo
	pKPS->timer = TimerRegisterPeriodicInterruption(KPS_Processing, US_TO_TICKS(1000000/pKPS->kps_config.sample_frequency), aux);


	return 1;
}

void KPS_SendNote(uint16_t note)
{
	// calculamos el valor de N
	pKPS->kps_order = (uint16_t)(pKPS->kps_config.sample_frequency / note);

	// lleno el buffer con ruido blanco
	for(int i = 0 ; i < pKPS->kps_order ; i++)
	{
		kps_buffer[i] = (rand() % 4096) - 2048;
	}

	// se comienza en 0 el buffer
	pKPS->kps_index = 0;


}

void KPS_Processing(void*)
{
	// tomo las
	int32_t ZN = kps_buffer[(pKPS->kps_index + pKPS->kps_order) % pKPS->kps_order];
	int32_t ZN_1 = kps_buffer[(pKPS->kps_index + pKPS->kps_order + 1) % pKPS->kps_order];

	ZN = (pKPS->kps_config.feedback_const1 * ZN)/1000;
	ZN_1 = (pKPS->kps_config.feedback_const2 * ZN_1)/1000;

	int16_t y = kps_buffer[pKPS->kps_index] + ZN + ZN_1;

	// el siguiente conjunto if else if es para limitar a los 12 bits del DAC
	if(y > 2047)
	{
		y = 2047;
	}
	else if (y < -2048)
	{
		y = -2048;
	}

	// guardo la muestra procesada en el buffer
	kps_buffer[pKPS->kps_index] = y;

	// avanzo en el buffer hasta llegar al maximo y vuelvo (buffer circular)
	pKPS->kps_index = (pKPS->kps_index + 1) % pKPS->kps_order;
}

/*
 * KPS.c
 *
 *  Created on: 27 nov. 2025
 *      Author: plaju
 */

/****************************HEADERS****************************/

#include "KPS.h"
#include "drivers/Timer.h"
#include "drivers/dac.h"
#include "drivers/gpio.h"
#include <stdlib.h>

/****************************MACROS****************************/

#define N_MAX 383	// fs = 100k y DOn = 261
#define MAX_DELAY 4096

/****************************ESTRUCTURAS****************************/

typedef struct{
	KPS_Config kps_config;

	// buffer param
	uint16_t kps_index;
	uint16_t kps_order;

	// timer param
	service_id timer;

	// status
	bool isPlaying;
} KPS;

/****************************VARIABLES****************************/

KPS* pKPS = NULL;
int32_t kps_buffer[N_MAX+1];
ticks note_ticks;

int32_t buffer[N_MAX+1] = {-2048, -346, -128, -380, 2047, -512, -2048, 197, 30, -2048, -92, 2047, -128, 102, 1023, 31, 310, -2048, -512, 159, -320, -256, 52, 2047, 0, -57, -64, 386, -1024, 159, 255, -256, 255, 206, -435, 384, 61, -2048, 2047, 2047, -128, 511, 127, 127, -346, 98, 390, -108, -1024, -256, -2048, 2047, -2048, -126, 87, -30, 1023, -256, 399, 246, 0, 2047, -128, 255, -512, -512, -2048, 31, 2047, 2047, 2047, 2047, -2048, -58, 255, 159, -2048, 0, -1024, 2047, 0, -123, 112, -128, 62, -27, 2047, -256, 646, -252, -2, 2047, 207, -347, 326, -113, 116, -2048, 2047, 255, 2047, -380, 0, 639, 239, 255, 511, -1024, 2047, -384, 2047, -113, 415, -128, 54, 238, 383, -256, 511, -2048, 487, 96, 255, -128, -2048, -128, 79, -98, 255, -128, 2047, 223, -128, -1024, 0, 31, -242, -32, 98, -2048, 103, 2047, -256, 2047, 2047, -2048, -219, 511, 646, 2047, -2048, 487, -235, -576, 71, 197, 2047, 1023, 197, -64, -64, -250, -2048, -12, 310, 197, 1023, 0, 2047, 2047, 101, 199, -2048, 2047, 100, 5, 207, 0, -2, -2048, -512, -11, 0, 229, -252, 255, 2047, -2048, -2048, 495, -128, 198, -2048, -346, -128, -380, 2047, -512, -2048, 197, 30, -2048, -92, 2047, -128, 102, 1023, 31, 310, -2048, -512, 159, -320, -256, 52, 2047, 0, -57, -64, 386, -1024, 159, 255, -256, 255, 206, -435, 384, 61, -2048, 2047, 2047, -128, 511, 127, 127, -346, 98, 390, -108, -1024, -256, -2048, 2047, -2048, -126, 87, -30, 1023, -256, 399, 246, 0, 2047, -128, 255, -512, -512, -2048, 31, 2047, 2047, 2047, 2047, -2048, -58, 255, 159, -2048, 0, -1024, 2047, 0, -123, 112, -128, 62, -27, 2047, -256, 646, -252, -2, 2047, 207, -347, 326, -113, 116, -2048, 2047, 255, 2047, -380, 0, 639, 239, 255, 511, -1024, 2047, -384, 2047, -113, 415, -128, 54, 238, 383, -256, 511, -2048, 487, 96, 255, -128, -2048, -128, 79, -98, 255, -128, 2047, 223, -128, -1024, 0, 31, -242, -32, 98, -2048, 103, 2047, -256, 2047, 2047, -2048, -219, 511, 646, 2047, -2048, 487, -235, -576, 71, 197, 2047, 1023, 197, -64, -64, -250, -2048, -12, 310, 197, 1023, 0, 2047, 2047, 101, 199, -2048, 2047, 100, 5, 207, 0, -2, -2048, -512, -11, 0, 229, -252, 255, 2047, -2048, -2048, 495, -128, 198};

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

	// inicializo el DAC
	Dac_Init();

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

	if (pKPS->isPlaying)
	{
		TimerUnregisterPeriodicInterruption(pKPS->timer);
	}

	pKPS->isPlaying = true;

	// registro un timer con frecuencia de sampleo
	pKPS->timer = TimerRegisterPeriodicInterruption(&KPS_Processing, US_TO_TICKS(1000000/pKPS->kps_config.sample_frequency), 0);

	note_ticks = Now();

	return;
}

void KPS_Processing(void*)
{
	ticks actual_ticks = Now();

	// si pasaron mas de 3 segundos
	if(actual_ticks - note_ticks > MS_TO_TICKS(3000))
	{
		TimerUnregisterPeriodicInterruption(pKPS->timer);
		pKPS->isPlaying = false;
		//Dac_Write12(0);
		return;
	}

 	int16_t index_zn = (pKPS->kps_index + 2) % (pKPS->kps_order+1);
 	int16_t index_zn_1 = (pKPS->kps_index + 1) % (pKPS->kps_order+1);

	int64_t ZN = kps_buffer[index_zn];
	int64_t ZN_1 = kps_buffer[index_zn_1];

	ZN = (pKPS->kps_config.feedback_const1 * ZN)/1000;
	ZN_1 = (pKPS->kps_config.feedback_const2 * ZN_1)/1000;


	int32_t y = (int32_t)ZN + (int32_t)ZN_1;


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

	pKPS->kps_index++;

	pKPS->kps_index %= (pKPS->kps_order+1);

//	if(pKPS->kps_index == 0)
//	{
//		TimerUnregisterPeriodicInterruption(pKPS->timer);
//		Dac_Write12(0);
//		return;
//	}

	y+=2048;                 //Restore DC component

	Dac_Write12((uint16_t)y);
}

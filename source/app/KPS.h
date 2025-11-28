/*
 * KPS.h
 *
 *  Created on: 27 nov. 2025
 *      Author: plaju
 */

#ifndef APP_KPS_H_
#define APP_KPS_H_

#include <stdint.h>

//Note Frecuency
#define DOn       261 //.63
#define DOSOn     277 //.18
#define REn       293 //.66
#define RESOSn    311 //.13
#define MIn       329 //.63
#define FAn       349 //.23
#define SOLn      392 //
#define SOLSOSn   415 //.3
#define LAn       440 //
#define LASOSn    466 //
#define SIn       492 //.88
#define DOn1      261 //.63
#define DOSOn1    277 //.18
#define REn1      293 //.66
#define RESOSn1   311 //.13
#define MIn1      329 //.63
#define FAn1      349 //.23
#define SOLn1     392 //
#define SOLSOSn1  415 //.3
#define LAn1      440 //
#define LASOSn1   466 //
#define SIn1      492 //.88
#define DO8n     523 //.25

/******************************structs****************************************/

typedef struct{
	uint32_t sample_frequency;
	uint32_t feedback_const1;
	uint32_t feedback_const2;
}KPS_Config;

/********************** external functions declaration ***********************/

// esta funcion inicializa todos los perifericos necesarios para realizar una salida de KPS
// ADC, DAC, DMA con PIT, FIR,
int8_t KPS_Init(KPS_Config * config);

// esta funcion envía por el DAC la nota requerida
void KPS_SendNote(uint16_t note);

#endif /* APP_KPS_H_ */

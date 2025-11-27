/*****************************************************************************
  @file     Timer.h
  @brief    Driver de temporizadores
  @author   jtori
 ******************************************************************************/

#ifndef DRIVERS_TIMER_H_
#define DRIVERS_TIMER_H_

/*******************************************************************************
 *                                ENCABEZADOS
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "Callback.h"

/*******************************************************************************
 *                                OBJETOS
 ******************************************************************************/
typedef unsigned long long ticks;
typedef uint32_t service_id;

/*******************************************************************************
 *                                MACROS
 ******************************************************************************/
#define TICKS_PER_SECOND (ticks)1000u
#define MS_TO_TICKS(x) (ticks)((x) * TICKS_PER_SECOND/1000)
#define US_TO_TICKS(x) (ticks)((x) * ( TICKS_PER_SECOND)/1000000)

/*******************************************************************************
 *                               PROTOTIPOS
 ******************************************************************************/
bool TimerInit();
service_id TimerRegisterPeriodicInterruption(callback* pCallback, ticks deltaT, void* user_data);
void TimerSetUserData(service_id serviceId, void* user_data);
void TimerSetEnable(service_id serviceId, bool enable);
bool TimerUnregisterPeriodicInterruption(service_id serviceId);
ticks Now();
void Sleep(ticks dt);

#endif /* DRIVERS_TIMER_H_ */

/*****************************************************************************
  @file     Timer.c
  @brief    Driver de temporizadores
  @author   jtori
 ******************************************************************************/

/*******************************************************************************
 *                                ENCABEZADOS
 ******************************************************************************/
#include "Timer.h"
#include "SysTick.h"
#include <stdlib.h>

/*******************************************************************************
 *                                  MACROS
 ******************************************************************************/
#define INITIAL_SERVICE_CAPACITY 16u

/*******************************************************************************
 *                                  OBJETOS
 ******************************************************************************/
typedef struct
{
	callback* pCallback;
	ticks tickInterval;
	ticks tickCount;
	void* user_data;
	bool enable;
} PeriodicService;

/*******************************************************************************
 *                                 VARIABLES
 ******************************************************************************/
static PeriodicService* pServices;
static uint32_t registeredServicesCount;
static uint32_t maxCapacity;
static ticks current_ticks = 0;

/*******************************************************************************
 *                                FUNCIONES
 ******************************************************************************/
void TimerPISR()
{
	for (int i = 0; i < registeredServicesCount; i++)
	{
		PeriodicService* pService = &pServices[i];
		if (!pService->enable)
			continue;
		if (pService->tickCount == 0)
		{
			pService->tickCount = pService->tickInterval;
			pService->pCallback(pService->user_data);
		}
		else
		{
			pService->tickCount--;
		}
	}
	current_ticks++;
}

bool TimerInit()
{
	// Init systick
	return SysTick_Init(&TimerPISR, (uint64_t)TICKS_PER_SECOND);
}

service_id TimerRegisterPeriodicInterruption(callback* pCallback, ticks deltaT, void* user_data)
{
	if (maxCapacity == 0)
	{
		pServices = (PeriodicService*)malloc(sizeof(PeriodicService) * INITIAL_SERVICE_CAPACITY);
		maxCapacity = INITIAL_SERVICE_CAPACITY;
	}

	if (registeredServicesCount >= maxCapacity)
	{
		maxCapacity = maxCapacity << 1;
		pServices = (PeriodicService*)realloc(pServices, sizeof(PeriodicService) * maxCapacity);
	}

	PeriodicService* pService = &pServices[registeredServicesCount];

	pService->pCallback = pCallback;
	pService->tickInterval = deltaT - 1;
	pService->tickCount = 0;
	pService->user_data = user_data;
	pService->enable = 1;

	return registeredServicesCount++;
}

bool TimerUnregisterPeriodicInterruption(service_id serviceId)
{
	if (serviceId >= registeredServicesCount)
	{
		return false;
	}

	PeriodicService temp = pServices[registeredServicesCount--];
	pServices[serviceId] = temp;
	return true;
}


void TimerSetEnable(service_id serviceId, bool enable)
{
	PeriodicService* pService = &pServices[serviceId];
	pService->enable = enable;
	pService->tickCount = pService->tickInterval;
}

void TimerSetUserData(service_id serviceId, void* user_data)
{
	PeriodicService* pService = &pServices[serviceId];
	pService->user_data = user_data;
}

ticks Now()
{
	return current_ticks;
}

void Sleep(ticks dt)
{
	const ticks start = Now();
	ticks current = Now();
	while (current - start < dt)
	{
		current = Now();
	}
	return;
}

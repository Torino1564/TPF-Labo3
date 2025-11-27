/***************************************************************************//**
  @file     hardware.h
  @brief
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 * Acá van los defines que se vayan a utilizar en nuestra aplicación. Relación
 * entre los pines, la kinetis y los drivers
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

#include "fsl_device_registers.h"
#include "core_cm4.h"
#include <stdbool.h>
#include <stdint.h>

#define __CORE_CLOCK__  100000000U
#define __FOREVER__     for(;;)
#define __ISR__         void __attribute__ ((interrupt))

void hw_Init (void);

void hw_EnableInterrupts (void);
void hw_DisableInterrupts (void);

#endif /* _HARDWARE_H_ */

/*
 * FTM.h
 *
 *  Created on: Nov 2, 2025
 *      Author: jtori
 */

#ifndef DRIVERS_FTM_H_
#define DRIVERS_FTM_H_

#include "hardware.h"
#include <stdint.h>
#include <stddef.h>

typedef uint8_t FTM_Handle;


typedef enum
{
	FTM_mInputCapture,
	FTM_mOutputCompare,
	FTM_mPulseWidthModulation,
	FTM_mCenterPulseWidthModulation
} FTM_Mode_t;

typedef enum
{
	FTM_eRising 		= 0x01,
	FTM_eFalling 		= 0x02,
	FTM_eEither 		= 0x03,
} FTM_Edge_t;

typedef enum
{
	FTM_eToggle 		= 0x01,
	FTM_eClear 			= 0x02,
	FTM_eSet 			= 0x03,
} FTM_Effect_t;

typedef enum
{
	FTM_lAssertedHigh	= 0x02,
	FTM_lAssertedLow 	= 0x03,
} FTM_Logic_t;

typedef enum
{
	FTM_PSC_x1		= 0x00,
	FTM_PSC_x2		= 0x01,
	FTM_PSC_x4		= 0x02,
	FTM_PSC_x8		= 0x03,
	FTM_PSC_x16		= 0x04,
	FTM_PSC_x32		= 0x05,
	FTM_PSC_x64		= 0x06,
	FTM_PSC_x128	= 0x07,

} FTM_Prescal_t;


#define FTM_CH_0 0
#define FTM_CH_1 1
#define FTM_CH_2 2
#define FTM_CH_3 3
#define FTM_CH_4 4
#define FTM_CH_5 5
#define FTM_CH_6 6
#define FTM_CH_7 7

typedef FTM_Type *FTM_t;
typedef uint16_t FTM_Data_t;
typedef uint32_t FTM_Channel_t; /* FTM0/FTM3: Channel 1-8; FTM1/FTM2: Channel 1-2 */

typedef struct {
	FTM_Handle ftmNum;
	FTM_Mode_t work_mode;
	FTM_Prescal_t prescaler;
	FTM_Channel_t channel;

	// only modulation
	uint32_t frequency;
	float duty_cycle;

	// only demodulation
	FTM_Edge_t edge_detection;
	uint16_t bufferSize;
	bool externalInputEnable;
	uint8_t internalInputNumber;

	// IRQ Callback
	bool enableCallback;
	void (*callback)(void*);
	void* user_data;
} FTM_Config;

/*******************************************************************************
								PROTOTIPOS
 ******************************************************************************/

FTM_Handle FTM_Init (FTM_Config * config);

bool FTM_GetBackBuffer(uint8_t ftmNum, uint16_t** pBuffer, size_t* size);

void        FTM_SetPrescaler 				 (FTM_Handle, FTM_Prescal_t);
FTM_Prescal_t FTM_GetPrescaler(FTM_Handle);

void     	FTM_SetModulus 					 (FTM_Handle, FTM_Data_t);
FTM_Data_t 	FTM_GetModulus 					 (FTM_Handle);
void 		FTM_Sync						 (FTM_Handle);
void 		FTM_StartClock					 (FTM_Handle);
void 		FTM_StopClock					 (FTM_Handle);

void 		FTM_SetOverflowMode   			 (FTM_Handle, bool);
bool 		FTM_IsOverflowPending 			 (FTM_Handle);
void 		FTM_ClearOverflowFlag 			 (FTM_Handle);

void        FTM_SetWorkingMode				 (FTM_Handle, FTM_Channel_t, FTM_Mode_t);
FTM_Mode_t   FTM_GetWorkingMode				 (FTM_Handle, FTM_Channel_t);
void        FTM_SetInputCaptureEdge 		 (FTM_Handle, FTM_Channel_t, FTM_Edge_t);
FTM_Edge_t   FTM_GetInputCaptureEdge 		 (FTM_Handle, FTM_Channel_t);
void        FTM_SetOutputCompareEffect 	 	 (FTM_Handle, FTM_Channel_t, FTM_Effect_t);
FTM_Effect_t FTM_GetOutputCompareEffect 		 (FTM_Handle, FTM_Channel_t);
void        FTM_SetPulseWidthModulationLogic (FTM_Handle, FTM_Channel_t, FTM_Logic_t);
FTM_Logic_t  FTM_GetPulseWidthModulationLogic (FTM_Handle, FTM_Channel_t);

void        FTM_SetCounter 					 (FTM_Handle, FTM_Channel_t, FTM_Data_t);
FTM_Data_t   FTM_GetCounter 					 (FTM_Handle, FTM_Channel_t);

uint16_t	FTM_GetModValue					 (FTM_Handle);

void 		FTM_SetInterruptMode   			 (FTM_Handle, FTM_Channel_t, bool);
bool 		FTM_IsInterruptPending 			 (FTM_Handle, FTM_Channel_t);
void 		FTM_ClearInterruptFlag 			 (FTM_Handle, FTM_Channel_t);


#endif /* DRIVERS_FTM_H_ */

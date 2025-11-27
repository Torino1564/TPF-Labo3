
#include "FTM.h"
#include <stdlib.h>
#include "gpio.h"

#define NUM_MODULES 4
#define DEFAULT_BUFFER_SIZE 128

/*******************************************************************************
								STRUCTS
 ******************************************************************************/

typedef struct {
	FTM_Config ftm_config;

	// Square period
	uint16_t modValue;

	// buffers
	uint16_t* pBuffers[2];
	bool bufSel;
	uint16_t bufLen;
	uint16_t bufIndex;
	bool newBufData;
} FTM;

/*******************************************************************************
								VARIABLES
 ******************************************************************************/

static FTM * pModules[NUM_MODULES] = {};

static FTM_Type * pRegisters[NUM_MODULES] = FTM_BASE_PTRS;

static IRQn_Type Interrupts[NUM_MODULES] = FTM_IRQS;

FTM_Handle FTM_Init (FTM_Config * config)
{
	// evaluo si la instancia es valida
	if (config->ftmNum < 0 || config->ftmNum >= NUM_MODULES)
	{
		return -1;
	}

	// evaluo si no se encuentra inicializada already
	if (pModules[config->ftmNum] != 0)
	{
		return -1;
	}

	// guardo en el heap la config
	pModules[config->ftmNum] = (FTM*)calloc(1, sizeof(FTM));

	// apunto al heap
	FTM* ftm = pModules[config->ftmNum];

	// guardo la config
	ftm->ftm_config = *config;

	// segun la instancia clockeo
	switch (ftm->ftm_config.ftmNum)
	{
	case 0:
		SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
		FTM0->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;
		break;
	case 1:
		SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;
		FTM1->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;
		// Configuro el SIM para la entrada interna
		if (!config->externalInputEnable && config->work_mode == FTM_mInputCapture)
			SIM->SOPT4 |= SIM_SOPT4_FTM1CH0SRC(config->internalInputNumber);
		break;
	case 2:
		SIM->SCGC6 |= SIM_SCGC6_FTM2_MASK;
		SIM->SCGC3 |= SIM_SCGC3_FTM2_MASK;
		FTM2->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;
		if (!config->externalInputEnable && config->work_mode == FTM_mInputCapture)
			SIM->SOPT4 |= SIM_SOPT4_FTM2CH0SRC(config->internalInputNumber);
		break;
	case 3:
		SIM->SCGC3 |= SIM_SCGC3_FTM3_MASK;
		FTM3->PWMLOAD = FTM_PWMLOAD_LDOK_MASK | 0x0F;
		break;
	}

	// segun la instancia habilito las interrupciones
	NVIC_EnableIRQ(Interrupts[ftm->ftm_config.ftmNum]);

	FTM_Type * pReg = pRegisters[config->ftmNum];

	switch(ftm->ftm_config.work_mode)
	{
	case FTM_mInputCapture:

		// deshabilito el write protection
		pReg->MODE |= FTM_MODE_WPDIS_MASK;

		// deshabilito la ftm para configuracion segura
		pReg->SC = 0;	// del reg SC el apartado CLKS en especial si esta en 00 deshabilita la ftm
						// es write protected
		pReg->CNT = 0;	// reseteo el valor del contador de la ftm

		FTM_SetPrescaler(config->ftmNum, ftm->ftm_config.prescaler);
		FTM_SetModulus(config->ftmNum, 0xFFFF);

		// lo ponemos en free running
		pReg->MODE |= FTM_MODE_FTMEN_MASK;

		FTM_StartClock(config->ftmNum);

		FTM_SetWorkingMode(config->ftmNum, FTM_CH_0, FTM_mInputCapture);

		FTM_SetInputCaptureEdge(config->ftmNum, FTM_CH_0, config->edge_detection);

		FTM_SetInterruptMode(config->ftmNum, FTM_CH_0, true);

		ftm->pBuffers[0] = (uint16_t *)calloc(ftm->ftm_config.bufferSize, sizeof(uint16_t));
		ftm->pBuffers[1] = (uint16_t *)calloc(ftm->ftm_config.bufferSize, sizeof(uint16_t));
		ftm->bufLen = ftm->ftm_config.bufferSize;
		ftm->bufIndex = 0;

		break;

	case FTM_mOutputCompare:

		break;

	case FTM_mCenterPulseWidthModulation:
	case FTM_mPulseWidthModulation:

		// deshabilito el write protection
		pReg->MODE |= FTM_MODE_WPDIS_MASK;

		// deshabilito la ftm para configuracion segura
		pReg->SC = 0;	// del reg SC el apartado CLKS en especial si esta en 00 deshabilita la ftm
						// es write protected
		pReg->CNT = 0;	// reseteo el valor del contador de la ftm

		// calculo el mejor valor de prescaler y mod segun la frecuencia de PWM

		uint32_t bus_clock = 50000000u;
		uint8_t prescaler_bits = 0;
		uint32_t prescaler_value = 1;
		uint16_t mod_value = 0;

		for(prescaler_bits = 0 ; prescaler_bits < 8 ; prescaler_bits++)
		{
			prescaler_value = 1 << prescaler_bits;
			mod_value = (bus_clock/(prescaler_value * ftm->ftm_config.frequency)) - 1;
			ftm->modValue = mod_value;
			// pregunto si el valor del prescaler tiene buena resolucion, a ojo, mayor que 100
			if(mod_value > 100)
			{
				break;
			}
		}

		// Enable PWM sw sync
		pReg->SYNCONF |= FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_SWWRBUF_MASK;
		pReg->COMBINE |= FTM_COMBINE_SYNCEN0_MASK;

		pReg->SYNC = FTM_SYNC_CNTMAX_MASK;

		if (ftm->ftm_config.work_mode == FTM_mCenterPulseWidthModulation)
		{
			pReg->SC |= FTM_SC_CPWMS_MASK;
		}

		// configuro con el prescaler obtenido
		FTM_SetPrescaler(config->ftmNum, prescaler_bits);

		// configuro el mod, que seria el periodo del PWM
		FTM_SetModulus(config->ftmNum, (FTM_Data_t)mod_value);

		// habilito la interrupcion por overflow
		FTM_SetOverflowMode(config->ftmNum, 1);

		// habilito las caracteristicas avanzadas
		pReg->MODE |= FTM_MODE_FTMEN_MASK;

		// configuro el modo de trabajo
		FTM_SetWorkingMode(config->ftmNum, ftm->ftm_config.channel, ftm->ftm_config.work_mode);

		// configuro la logica de pulso
		FTM_SetPulseWidthModulationLogic(config->ftmNum, ftm->ftm_config.channel, FTM_lAssertedHigh);

		// completo en CV una fraccion de MOD. Si CV es 0.5MOD, DC = 50%, si CV es 0.2MOD, DC = 20%
		FTM_SetCounter(config->ftmNum, ftm->ftm_config.channel, (FTM_Data_t)(mod_value*ftm->ftm_config.duty_cycle));

		// empiezo a contar clockardos
		FTM_StartClock(config->ftmNum);

		break;

	default:

		break;
	}


	return ftm->ftm_config.ftmNum;
}

// setters

void FTM_SetPrescaler (FTM_Handle ftm, FTM_Prescal_t data)
{
	FTM_Type* pFTM = pRegisters[ftm];
	pFTM->SC = (pFTM->SC & ~FTM_SC_PS_MASK) | FTM_SC_PS(data);
}

FTM_Prescal_t FTM_GetPrescaler(FTM_Handle ftm)
{
	FTM* pFTM = pModules[ftm];
	return pFTM->ftm_config.prescaler;
}

void FTM_SetModulus (FTM_Handle ftm, FTM_Data_t data)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CNTIN = 0X00;
	pReg->CNT = 0X00;
	pReg->MOD = FTM_MOD_MOD(data);
}

FTM_Data_t FTM_GetModulus (FTM_Handle ftm)
{
	FTM_Type* pReg = pRegisters[ftm];
	return pReg->MOD & FTM_MOD_MOD_MASK;
}


void FTM_Sync(FTM_Handle ftm)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->SYNC |= FTM_SYNC_SWSYNC_MASK;
	//pReg->PWMLOAD = FTM_PWMLOAD_CH0SEL_MASK | FTM_PWMLOAD_LDOK_MASK;
}

void FTM_StartClock (FTM_Handle ftm)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->SC |= FTM_SC_CLKS(0x01);
}

void FTM_StopClock (FTM_Handle ftm)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->SC &= ~FTM_SC_CLKS(0x01);
}

void FTM_SetOverflowMode (FTM_Handle ftm, bool mode)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->SC = (pReg->SC & ~FTM_SC_TOIE_MASK) | FTM_SC_TOIE(mode);
}

bool FTM_IsOverflowPending (FTM_Handle ftm)
{
	FTM_Type* pReg = pRegisters[ftm];
	return pReg->SC & FTM_SC_TOF_MASK;
}

void FTM_ClearOverflowFlag (FTM_Handle ftm)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->SC &= ~FTM_SC_TOF_MASK;
}

void FTM_SetWorkingMode (FTM_Handle ftm, FTM_Channel_t channel, FTM_Mode_t mode)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CONTROLS[channel].CnSC = (pReg->CONTROLS[channel].CnSC & ~(FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK)) |
			                      (FTM_CnSC_MSB((mode >> 1) & 0X01) | FTM_CnSC_MSA((mode >> 0) & 0X01));
}

FTM_Mode_t FTM_GetWorkingMode (FTM_Handle ftm, FTM_Channel_t channel)
{
	FTM_Type* pReg = pRegisters[ftm];
	return (pReg->CONTROLS[channel].CnSC & (FTM_CnSC_MSB_MASK | FTM_CnSC_MSA_MASK)) >> FTM_CnSC_MSA_SHIFT;
}

void FTM_SetInputCaptureEdge (FTM_Handle ftm, FTM_Channel_t channel, FTM_Edge_t edge)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CONTROLS[channel].CnSC = (pReg->CONTROLS[channel].CnSC & ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) |
				                  (FTM_CnSC_ELSB((edge >> 1) & 0X01) | FTM_CnSC_ELSA((edge >> 0) & 0X01));
}

FTM_Edge_t FTM_GetInputCaptureEdge (FTM_Handle ftm, FTM_Channel_t channel)
{
	FTM_Type* pReg = pRegisters[ftm];
	return (pReg->CONTROLS[channel].CnSC & (FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) >> FTM_CnSC_ELSA_SHIFT;
}

void FTM_SetOutputCompareEffect (FTM_Handle ftm, FTM_Channel_t channel, FTM_Effect_t effect)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CONTROLS[channel].CnSC = (pReg->CONTROLS[channel].CnSC & ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) |
				                  (FTM_CnSC_ELSB((effect >> 1) & 0X01) | FTM_CnSC_ELSA((effect >> 0) & 0X01));
}

FTM_Effect_t FTM_GetOutputCompareEffect (FTM_Handle ftm, FTM_Channel_t channel)
{
	FTM_Type* pReg = pRegisters[ftm];
	return (pReg->CONTROLS[channel].CnSC & (FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) >> FTM_CnSC_ELSA_SHIFT;
}

void FTM_SetPulseWidthModulationLogic (FTM_Handle ftm, FTM_Channel_t channel, FTM_Logic_t logic)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CONTROLS[channel].CnSC = (pReg->CONTROLS[channel].CnSC & ~(FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) |
				                  (FTM_CnSC_ELSB((logic >> 1) & 0X01) | FTM_CnSC_ELSA((logic >> 0) & 0X01));
}

FTM_Logic_t FTM_GetPulseWidthModulationLogic (FTM_Handle ftm, FTM_Channel_t channel)
{
	FTM_Type* pReg = pRegisters[ftm];
	return (pReg->CONTROLS[channel].CnSC & (FTM_CnSC_ELSB_MASK | FTM_CnSC_ELSA_MASK)) >> FTM_CnSC_ELSA_SHIFT;
}

void FTM_SetCounter (FTM_Handle ftm, FTM_Channel_t channel, FTM_Data_t data)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CONTROLS[channel].CnV = FTM_CnV_VAL(data);
}

FTM_Data_t FTM_GetCounter (FTM_Handle ftm, FTM_Channel_t channel)
{
	FTM_Type* pReg = pRegisters[ftm];
	return pReg->CONTROLS[channel].CnV & FTM_CnV_VAL_MASK;
}

void FTM_SetInterruptMode (FTM_Handle ftm, FTM_Channel_t channel, bool mode)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CONTROLS[channel].CnSC = (pReg->CONTROLS[channel].CnSC & ~FTM_CnSC_CHIE_MASK) | FTM_CnSC_CHIE(mode);
}

bool FTM_IsInterruptPending (FTM_Handle ftm, FTM_Channel_t channel)
{
	FTM_Type* pReg = pRegisters[ftm];
	return pReg->CONTROLS[channel].CnSC & FTM_CnSC_CHF_MASK;
}

void FTM_ClearInterruptFlag (FTM_Handle ftm, FTM_Channel_t channel)
{
	FTM_Type* pReg = pRegisters[ftm];
	pReg->CONTROLS[channel].CnSC &= ~FTM_CnSC_CHF_MASK;
}


bool FTM_GetBackBuffer(uint8_t ftmNum, uint16_t** pBuffer, size_t* size)
{
	FTM* pFTM = pModules[ftmNum];
	if(!(pFTM->newBufData))
	{
		*pBuffer = 0;
		*size = 0;
		return false;
	}
	pFTM->newBufData = 0;	// apagit
	*pBuffer = pModules[ftmNum]->pBuffers[pFTM->bufSel];
	*size = pFTM->ftm_config.bufferSize;

	return true;
}

uint16_t FTM_GetModValue(FTM_Handle ftm)
{
	return pModules[ftm]->modValue;
}

// interruptions

void FTM_IRQHandler_Internal(FTM_Handle ftmNum)
{
	FTM* pFTM = pModules[ftmNum];
	switch(pModules[ftmNum]->ftm_config.work_mode)
	{
	case FTM_mInputCapture:
	{
		FTM * ftm = pModules[ftmNum];

		FTM_Type * regs = pRegisters[ftmNum];

		pFTM->pBuffers[pFTM->bufSel][pFTM->bufIndex++] = regs->CNT;
		regs->CNT = 0;

		if(pFTM->bufIndex > pFTM->ftm_config.bufferSize - 1)
		{
			pFTM->bufIndex = 0;
			pFTM->bufSel = !pFTM->bufSel;
			pFTM->newBufData = 1;
		}
		gpioToggle(PORTNUM2PIN(PD, 0));

		regs->CONTROLS[0].CnSC &= ~FTM_CnSC_CHF_MASK;
	}
	break;

	case FTM_mPulseWidthModulation:
	{
		FTM_ClearOverflowFlag(ftmNum);

		break;
	}
	default:
		break;
	}
	if (pFTM->ftm_config.enableCallback)
		pFTM->ftm_config.callback(pFTM->ftm_config.user_data);
}

void FTM0_IRQHandler(void)
{
    FTM_IRQHandler_Internal(0);
}

void FTM1_IRQHandler(void)
{
    FTM_IRQHandler_Internal(1);
}

void FTM2_IRQHandler(void)
{
    FTM_IRQHandler_Internal(2);
}

void FTM3_IRQHandler(void)
{
    FTM_IRQHandler_Internal(3);
}

/*
 * DMA.c
 *
 *  Created on: Oct 22, 2025
 *      Author: jtori
 */


#include "DMA.h"
#include "gpio.h"
#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
#include <hardware.h>

#define NUM_CHANNELS 16

typedef struct {
	uint8_t index;

	// 0 means always, else source number
	uint8_t source;

	// PIT info
	bool trigger;
	uint32_t frequency_KHz;

	DMA_Config dma_config;

	bool enabled;

} DMAChannel;

static const int IRQns[][16] = DMA_CHN_IRQS;

static DMAChannel* pDMAChannels[16] = {};
static bool globalDMAInitialization = false;

typedef struct {                                         /* offset: 0x1000, array step: 0x20 */
	__IO uint32_t SADDR;                             /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
	__IO uint16_t SOFF;                              /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
	__IO uint16_t ATTR;                              /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
	union {                                          /* offset: 0x1008, array step: 0x20 */
	  __IO uint32_t NBYTES_MLNO;                       /**< TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20 */
	  __IO uint32_t NBYTES_MLOFFNO;                    /**< TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
	  __IO uint32_t NBYTES_MLOFFYES;                   /**< TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20 */
	};
	__IO uint32_t SLAST;                             /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
	__IO uint32_t DADDR;                             /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
	__IO uint16_t DOFF;                              /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
	union {                                          /* offset: 0x1016, array step: 0x20 */
	  __IO uint16_t CITER_ELINKNO;                     /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
	  __IO uint16_t CITER_ELINKYES;                    /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
	};
	__IO uint32_t DLAST_SGA;                         /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
	__IO uint16_t CSR;                               /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
	union {                                          /* offset: 0x101E, array step: 0x20 */
	  __IO uint16_t BITER_ELINKNO;                     /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
	  __IO uint16_t BITER_ELINKYES;                    /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
	};
} TCD;


static TCD TCD_Configuration(const DMA_Config* pConfig)
{
	TCD tcd = {};
	// Adresses
	tcd.SADDR |= DMA_SADDR_SADDR(pConfig->source_address);
	tcd.DADDR |= DMA_DADDR_DADDR(pConfig->destination_address);

	// Transfer Configuration
	tcd.ATTR |= DMA_ATTR_SSIZE(pConfig->source_transfer_data_size) | DMA_ATTR_DSIZE(pConfig->destination_transfer_data_size);

	// Minor Loop Configuration (Minor loops are enabled)
	if (pConfig->source_minor_loop_enabled || pConfig->destination_minor_loop_enabled)
	{
		// Offset enabled
		//assert(pConfig->minor_loop_num_bytes <= 0x3FF);
		tcd.NBYTES_MLOFFYES |= DMA_NBYTES_MLOFFYES_NBYTES(pConfig->minor_loop_num_bytes) |
				DMA_NBYTES_MLOFFYES_MLOFF(pConfig->minor_loop_offset) |
				DMA_NBYTES_MLOFFYES_DMLOE(pConfig->destination_minor_loop_enabled) |
				DMA_NBYTES_MLOFFYES_SMLOE(pConfig->source_minor_loop_enabled);

	}
	else
	{
		// Offset disabled
		//assert(pConfig->minor_loop_num_bytes <= 0x3FFFFFFF);
		tcd.NBYTES_MLOFFNO |= DMA_NBYTES_MLOFFNO_NBYTES(pConfig->minor_loop_num_bytes);
	}

	// Major Loop Configuration
	tcd.SLAST |= DMA_SLAST_SLAST(pConfig->source_major_loop_completion_offset);
	// SGA is not enabled
	tcd.DLAST_SGA |= DMA_DLAST_SGA_DLASTSGA(pConfig->destination_major_loop_completion_offset);

	// Enable Interrupts if enabled
	tcd.CSR |= DMA_CSR_INTMAJOR(pConfig->enable_finish_irq) | DMA_CSR_INTHALF(pConfig->enable_half_irq);

	// For now, Linking isnt supported TODO: add linking
	tcd.CITER_ELINKNO |= DMA_CITER_ELINKNO_CITER(pConfig->major_loop_number_iterations);
	tcd.BITER_ELINKNO |= DMA_BITER_ELINKNO_BITER(pConfig->major_loop_number_iterations);

	return tcd;
}

DMA_Channel DMA_InitChannel(const DMA_Config* pConfig)
{
	DMA_Type* pDMA = DMA0;
	if (!globalDMAInitialization)
	{
		SIM->SCGC6 |= SIM_SCGC6_PIT_MASK | SIM_SCGC6_DMAMUX_MASK;
		SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
		pDMA->CR = 0;
		pDMA->CR |= DMA_CR_EMLM(1);
		//PIT->MCR = PIT_MCR_FRZ(1);
		PIT->MCR = 0;
		globalDMAInitialization = 1;
	}

	for (int i = 0; i < NUM_CHANNELS; i++)
	{
		if (pConfig->trigger && i > 4)
		{
			// There are no more triggered channels available
			return -1;
		}

		if (pConfig->forcePitNumber)
		{
			if (i != pConfig->pitNumber)
				continue;
		}

		DMAChannel* pChannel = pDMAChannels[i];
		if (pChannel != NULL)
			continue;

		pDMAChannels[i] = (DMAChannel*)calloc(1, sizeof(DMAChannel));
		pChannel = pDMAChannels[i];
		// Here we are sure the pointer is to an empty channel

		// MUX configuration
		pChannel->source = pConfig->source;
		pChannel->trigger = pConfig->trigger;
		volatile uint8_t* pCHCFG = &DMAMUX->CHCFG[i];
		*pCHCFG = 0;
		*pCHCFG |= DMAMUX_CHCFG_SOURCE(pConfig->source) | DMAMUX_CHCFG_ENBL(1) | DMAMUX_CHCFG_TRIG(pConfig->trigger);

		// TODO: PIT configuration SACAR INTERRUPT
		if (pChannel->trigger & !pConfig->skipPITInitialization)
		{
			// Assume oscillator running at 50Mhz ~ 20ns
			PIT->CHANNEL[i].LDVAL = (uint32_t)(pConfig->trigger_period_us * 1000u / 20u);
			PIT->CHANNEL[i].TCTRL |= PIT_TCTRL_TEN(1);
		}

		if (pConfig->enable_finish_irq || pConfig->enable_half_irq)
			NVIC_EnableIRQ(IRQns[0][i]);

		// TCD Configuration
		(*(TCD*)&pDMA->TCD[i]) = TCD_Configuration(pConfig);

		// Guardo la configuracion
		pChannel->dma_config = *pConfig;

		// Enable Channel
		DMA_SetEnable(i, 1);

		return i;
	}
	// There are no more channels available
	return -1;
}

void DMA_SetEnable(DMA_Channel channel, bool status)
{
	uint16_t erq = DMA0->ERQ;
	erq &= ~(1 << channel);
	DMA0->ERQ = erq | (status << channel);
}

DMA_Config DMA_GetConfig(DMA_Channel channel)
{
	return pDMAChannels[channel]->dma_config;
}

void DMA_Reconfig(DMA_Channel channel, const DMA_Config* dma_config)
{
	pDMAChannels[channel]->dma_config = * dma_config;

	DMA_Type* pDMA = DMA0;

	(*(TCD*)&pDMA->TCD[channel]) = TCD_Configuration(dma_config);

	return;
}

void DMA_DestroyChannel(DMA_Channel channel)
{

}


void DMA_SetDestinationAddress(DMA_Channel channel, uint32_t daddr)
{
	DMA0->CERQ = channel;
	DMA0->TCD[channel].DADDR = daddr;
	DMA0->SERQ = channel;
}


void DMA_SetSourceAddress(DMA_Channel channel, uint32_t saddr)
{
	//DMA0->CERQ = channel;
	DMA0->TCD[channel].SADDR = saddr;
	//DMA0->SERQ = channel;
}

void DMAx_IRQCallback(uint8_t channel)
{
	DMAChannel* pChannel = pDMAChannels[channel];

	if (pChannel->dma_config.enable_half_irq && DMA0->TCD[channel].CITER_ELINKNO == pChannel->dma_config.major_loop_number_iterations / 2)
	{
		pChannel->dma_config.half_loop_callback(pChannel->dma_config.half_irs_user_data);
	}
	else if (pChannel->dma_config.enable_finish_irq)
	{
		pChannel->dma_config.finish_loop_callback(pChannel->dma_config.finish_irs_user_data);
	}

	DMA0->CINT = DMA_CINT_CINT(channel);
}

#define NUMCHANNELS \
	X(0) \
	X(1) \
	X(2) \
	X(3) \
	X(4) \
	X(5) \
	X(6) \
	X(7) \
	X(8) \
	X(9) \
	X(10) \
	X(11) \
	X(12) \
	X(13) \
	X(14) \
	X(15)

#define X(c) \
__ISR__ DMA##c##_IRQHandler()\
{\
	DMAx_IRQCallback(c);\
}
NUMCHANNELS
#undef X

/*
 * DMA.h
 * @brief DMA Driver
 *
 *  Created on: Oct 22, 2025
 *  Author: Joaquin Torino
 */

#ifndef DRIVERS_DMA_H_
#define DRIVERS_DMA_H_

#include <stdbool.h>
#include <stdint.h>

enum DMA_TransferDataSize {
	DMA_TDS_8bit 	= 0b000,
	DMA_TDS_16bit 	= 0b001,
	DMA_TDS_32bit	= 0b010,
	DMA_TDS_16bytes	= 0b100,
	DMA_TDS_32bytes = 0b101,
};

// Config parameter struct
typedef struct {
	// Mux configuration
	bool trigger;
	uint16_t source;
	bool always;
	bool skipPITInitialization;
	bool forcePitNumber;
	uint8_t pitNumber;
	uint32_t trigger_period_us;

	// Address configuration
	uint32_t source_address;
	uint32_t destination_address;

	// Transfer Configuration
	uint8_t source_transfer_data_size;
	uint8_t destination_transfer_data_size;

	// Minor Loop configuration
	uint32_t minor_loop_num_bytes;
	bool source_minor_loop_enabled;
	bool destination_minor_loop_enabled;
	int32_t minor_loop_offset;

	// Major Loop Configuration
	int32_t source_major_loop_completion_offset;
	int32_t destination_major_loop_completion_offset;
	uint32_t major_loop_number_iterations;

	// Callbacks
	bool enable_finish_irq;
	void (*finish_loop_callback)(void*);
	void *finish_irs_user_data;

	// Half
	bool enable_half_irq;
	void (*half_loop_callback)(void*);
	void *half_irs_user_data;

	// TODO: Channel linking Configuration

} DMA_Config;

typedef int16_t DMA_Channel;

DMA_Channel DMA_InitChannel(const DMA_Config* pConfig);
DMA_Config DMA_GetConfig(DMA_Channel channel);
void DMA_SetEnable(DMA_Channel channel, bool status);
void DMA_Reconfig(DMA_Channel channel, const DMA_Config* dma_config);
void DMA_SetDestinationAddress(DMA_Channel channel, uint32_t daddr);
void DMA_SetSourceAddress(DMA_Channel channel, uint32_t saddr);
void DMA_DestroyChannel(DMA_Channel channel);

#endif /* DRIVERS_DMA_H_ */

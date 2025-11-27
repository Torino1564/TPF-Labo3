/*
 * dac.c
 *
 *  Created on: 4 nov. 2025
\ */

#include "dac.h"
#include "DMA.h"
#include <stddef.h>

#define DMAMUX_SRC_PIT0  (58u)
#define DMAMUX_SRC_PIT1  (59u)
#define DMAMUX_SRC_PIT2  (60u)
#define DMAMUX_SRC_PIT3  (61u)

#ifndef DAC_PIT_BUS_HZ
#define DAC_PIT_BUS_HZ   (50000000u)
#endif

static inline uint8_t _dmamux_src_for_pit(uint8_t pit)
{
    switch (pit & 3u) {
        case 0: return DMAMUX_SRC_PIT0;
        case 1: return DMAMUX_SRC_PIT1;
        case 2: return DMAMUX_SRC_PIT2;
        default:return DMAMUX_SRC_PIT3;
    }
}

static volatile bool     s_dma_running = false;
static volatile uint8_t  s_pit_ch      = 0;
static DMA_Channel       s_dma_ch      = -1;
static void (*s_user_cb)(void*)         = NULL;

void Dac_Init(void)
{
    SIM->SCGC2 |= SIM_SCGC2_DAC0_MASK;
    DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK | DAC_C0_DACTRGSEL(0);  // ref=VDDA
    DAC0->C1 = 0;
    DAC0->C2 = 0;
    Dac_Write12(0);
}

void Dac_Write12(uint16_t code12)
{
    if (code12 > 4095u) code12 = 4095u;
    DAC0->DAT[0].DATL = (uint8_t)(code12 & 0xFFu);
    DAC0->DAT[0].DATH = (uint8_t)((code12 >> 8) & 0x0Fu);
}

// Utilidad: configurar PITx a sample_rate_hz
static void _pit_cfg_and_start(uint8_t pit_channel, uint32_t sample_rate_hz)
{
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

    uint32_t ld = (DAC_PIT_BUS_HZ / sample_rate_hz);
    if (ld == 0) ld = 1;
    PIT->CHANNEL[pit_channel].LDVAL = ld - 1u;
    PIT->CHANNEL[pit_channel].TFLG  = PIT_TFLG_TIF_MASK;
    PIT->CHANNEL[pit_channel].TCTRL = PIT_TCTRL_TEN_MASK;   // no hace falta TIE para DMA
}

static void _pit_stop(uint8_t pit_channel)
{
    PIT->CHANNEL[pit_channel].TCTRL = 0;
    PIT->CHANNEL[pit_channel].TFLG  = PIT_TFLG_TIF_MASK;
}

bool Dac_DmaIsRunning(void) { return s_dma_running; }

void Dac_DmaAbort(void)
{
    if (s_dma_running) {
        _pit_stop(s_pit_ch);
        if (s_dma_ch >= 0) {
            DMA_DestroyChannel(s_dma_ch);
            s_dma_ch = -1;
        }
        s_dma_running = false;
    }
}

/* usa PIT0 por defecto */
bool Dac_DmaStart(const uint16_t* samples12,
                  uint32_t length,
                  uint32_t sample_rate_hz,
                  void (*on_complete)(void*))
{
    return Dac_DmaStartEx(samples12, length, sample_rate_hz, 0u, on_complete, 0);
}

bool Dac_DmaStartEx(const uint16_t* samples12,
                    uint32_t length,
                    uint32_t sample_rate_hz,
                    uint8_t  pit_channel,
                    void (*on_complete)(void*),
					void* on_complete_user_data)
{
    if (!samples12 || length == 0 || sample_rate_hz == 0) return false;

    // Si había una en curso, aborto
    Dac_DmaAbort();

    // Configura PIT elegido y guarda
    s_pit_ch  = (pit_channel & 3u);
    s_user_cb = on_complete;

    DMA_Config dc = (DMA_Config){0};

    // Fuente de requests: PITx vía DMAMUX
    dc.trigger = true;
    dc.source  = _dmamux_src_for_pit(s_pit_ch);
    dc.always  = true;                                  // solo cuando PIT dispare

    // Direcciones
    dc.source_address      = (uint32_t)samples12;        // RAM (sube)
    dc.destination_address = (uint32_t)&DAC0->DAT[0].DATL; // registro DAC (fijo)

    // Tamaños de transfer
    dc.source_transfer_data_size      = DMA_TDS_16bit;
    dc.destination_transfer_data_size = DMA_TDS_16bit;

    // Minor loop: 1 muestra = 2 bytes
    dc.minor_loop_num_bytes           = 2u;

    dc.source_minor_loop_enabled      = true;            // SMLOE=1
    dc.destination_minor_loop_enabled = false;           // DMLOE=0
    dc.minor_loop_offset              = +2;              // MLOFF=+2

    dc.forcePitNumber = 1;
    dc.pitNumber = pit_channel;

    // Major loop: 'length' muestras; SLAST = -(length*2) => vuelve SADDR al inicio (stream cíclico)
    dc.major_loop_number_iterations             = length;
    dc.source_major_loop_completion_offset      = -(int32_t)(2u * (length-1));
    dc.destination_major_loop_completion_offset = 0;

    if (on_complete)
    {
		dc.enable_finish_irq = 1;
		dc.finish_loop_callback = on_complete;
		dc.finish_irs_user_data = on_complete_user_data;
    }

    s_dma_ch = DMA_InitChannel(&dc);
    if (s_dma_ch < 0) {
        s_dma_running = false;
        return false;
    }

    // Arrancar PIT (desde acá salen los requests periódicos al DMA)
    _pit_cfg_and_start(s_pit_ch, sample_rate_hz);

    s_dma_running = true;
    return true;
}

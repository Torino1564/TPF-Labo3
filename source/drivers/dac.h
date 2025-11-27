#ifndef DRIVERS_DAC_H_
#define DRIVERS_DAC_H_

#include <stdbool.h>
#include <stdint.h>
#include "hardware.h"
#include "DMA.h"

void Dac_Init(void);
void Dac_Write12(uint16_t code12);

bool Dac_DmaStart(const uint16_t* samples12,
                  uint32_t length,
                  uint32_t sample_rate_hz,
                  void (*on_complete)(void*));

bool Dac_DmaStartEx(const uint16_t* samples12,
                    uint32_t length,
                    uint32_t sample_rate_hz,
                    uint8_t  pit_channel,
                    void (*on_complete)(void*),
        			void* on_complete_user_data);

void Dac_DmaAbort(void);
bool Dac_DmaIsRunning(void);

#endif /* DRIVERS_DAC_H_ */

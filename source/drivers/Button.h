/*
 * Button.h
 *
 *  Created on: Sep 3, 2025
 *      Author: jtori
 */

#ifndef DRIVERS_BUTTON_H_
#define DRIVERS_BUTTON_H_

#include <stdbool.h>
#include "Timer.h"
#include "gpio.h"

#define BUTTON_IDLE 0
#define BUTTON_PRESSED 1
#define BUTTON_HELD 2
#define BUTTON_LONG_HELD 3

uint16_t NewButton(pin_t pin, bool activeHigh);
bool SetDebouncing(uint16_t buttonId, ticks dt);

bool readButtonStatus(uint16_t buttonId);
uint8_t readButtonData(uint16_t buttonId);

void DeleteButton();

#endif /* DRIVERS_BUTTON_H_ */

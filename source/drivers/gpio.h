/***************************************************************************//**
  @file     gpio.h
  @brief    Simple GPIO Pin services, similar to Arduino
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "Callback.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// Ports
enum { PA, PB, PC, PD, PE };

// Convert port and number into pin ID
// Ex: PTB5  -> PORTNUM2PIN(PB,5)  -> 0x25
//     PTC22 -> PORTNUM2PIN(PC,22) -> 0x56
#define PORTNUM2PIN(p,n)    (((p)<<5) + (n))
#define PIN2PORT(p)         (((p)>>5) & 0x07)
#define PIN2NUM(p)          ((p) & 0x1F)


// Modes
#ifndef INPUT
#define INPUT               0
#define OUTPUT              1
#define INPUT_PULLUP        2
#define INPUT_PULLDOWN      3
#endif // INPUT

// IRQC modes
#ifndef IRQCMODES
#define IRQCMODES
#define NO_INT				0b0000
#define FLAG_DMA_POSEDGE 	0b0001
#define FLAG_DMA_NEGEDGE 	0b0010
#define FLAG_DMA_EDGE		0b0011
#define FLAG_INT_0			0b1000
#define FLAG_INT_1			0b1100
#define FLAG_INT_POSEDGE	0b1001
#define FLAG_INT_NEGEDGE	0b1010
#define FLAG_INT_EDGE		0b1011
#endif

// MUX alternatives
#ifndef MUXALT
#define ALT0				0b000
#define ALT1				0b001
#define ALT2				0b010
#define ALT3				0b011
#define ALT4				0b100
#define ALT5				0b101
#define ALT6				0b110
#define ALT7				0b111
#endif

// Digital values
#ifndef LOW
#define LOW     0
#define HIGH    1
#endif // LOW


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef uint8_t pin_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
/**
 * @brief Enables the hardware interruption support
 */
void gpioInitInterrupts();

// Esta funcion recibe el valor de mux que debe colocar en el PCR del pin @param pin
void gpioMux(pin_t pin, uint8_t mux);

// Esta funcion recibe el pin, si hay que habilitar o deshabilitar las pull, y si es pullup o pulldown
void gpioPullRes(pin_t pin, bool enablePullRes, bool pullUp);

/**
 * @brief Configures the specified pin to behave either as an input or an output
 * @param pin the pin whose mode you wish to set (according PORTNUM2PIN)
 * @param mode INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN.
 */
void gpioMode (pin_t pin, uint8_t mode);

/**
 * @brief Write a HIGH or a LOW value to a digital pin
 * @param pin the pin to write (according PORTNUM2PIN)
 * @param val Desired value (HIGH or LOW)
 */
void gpioWrite (pin_t pin, bool value);

/**
 * @brief Toggle the value of a digital pin (HIGH<->LOW)
 * @param pin the pin to toggle (according PORTNUM2PIN)
 */
void gpioToggle (pin_t pin);

/**
 * @brief Reads the value from a specified digital pin, either HIGH or LOW.
 * @param pin the pin to read (according PORTNUM2PIN)
 * @return HIGH or LOW
 */
bool gpioRead (pin_t pin);

void gpioSetSlewRate(pin_t pin, bool slewRateLow);

/**
 * @brief Setup the interrupt mode of a gpio
 * @param Pointer to the ISR, otherwise defaults to empty
 */
void gpioSetupISR(pin_t pin, uint8_t interrupt_mode, callback* pCallback, void* user_data);
void gpioSetUserData(pin_t pin, void* user_data);

/*******************************************************************************
 ******************************************************************************/

#endif // _GPIO_H_

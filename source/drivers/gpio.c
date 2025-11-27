/***************************************************************************//**
  @file     gpio.c Rev2.0
  @brief    gpio perisferics functions
  @author   Group 2
 ******************************************************************************/

#include "gpio.h"
#include "hardware.h"

typedef struct {
	callback* pCallback;
	void* user_data;
} CallbackAndState;

static CallbackAndState callbackMatrix[160] = {0};

/*******************************************************************************
 *                                FUNCTIONS
 ******************************************************************************/

void gpioInitInterrupts()
{
	NVIC_EnableIRQ(PORTE_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_EnableIRQ(PORTB_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void gpioSetupISR(pin_t pin, uint8_t interrupt_mode, callback* pCallback, void* user_data)
{
	uint32_t portValue = (interrupt_mode<<PORT_PCR_IRQC_SHIFT);
	uint32_t portMask = PORT_PCR_IRQC_MASK;

	static PORT_Type * const portBase[] = PORT_BASE_PTRS;

	portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = (portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & ~portMask) | portValue;

	callbackMatrix[pin].pCallback = pCallback;
	callbackMatrix[pin].user_data = user_data;
}


void gpioSetUserData(pin_t pin, void* user_data)
{
	callbackMatrix[pin].user_data = user_data;
}

void gpioMux(pin_t pin, uint8_t mux)
{
	// ubico el valor de mux en los bits mux del pcr
	uint32_t portMuxValue = (mux << PORT_PCR_MUX_SHIFT);

	// obtengo un arreglo de punteros a pin
	static PORT_Type * const portBase[] = PORT_BASE_PTRS;

	// en el mux del pcr del pin completo con el valor del mux obtenido
	portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = (portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & ~PORT_PCR_MUX_MASK) | portMuxValue;
}

void gpioPullRes(pin_t pin, bool enablePullRes, bool pullUp)
{
	uint32_t portValue = (((uint32_t)enablePullRes) << PORT_PCR_PE_SHIFT) | (((uint32_t)pullUp) << PORT_PCR_PS_SHIFT);
	uint32_t portMask = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

	// obtengo un arreglo de punteros a pin
	static PORT_Type * const portBase[] = PORT_BASE_PTRS;

	portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = (portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & ~portMask) | (portValue & portMask);
}

void gpioMode (pin_t pin, uint8_t mode)
{
	uint32_t portValue = mode | (1<<PORT_PCR_MUX_SHIFT); // el valor que debo ingresar al port
	uint32_t portMask = PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_MUX_MASK; // mascara para modificar el pull enable, el pull set y el mux

	static PORT_Type * const portBase[] = PORT_BASE_PTRS;

	portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = ((portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & ~portMask) | portValue);

	uint32_t gpioValue = 0;

	if(mode == OUTPUT)
	{
		gpioValue = (1<<PIN2NUM(pin));
	}
	uint32_t gpioMask = (1<<PIN2NUM(pin)); // mascara para modificar el bit numero pin del PDDR. Este sirve para setear si es input u output

	static GPIO_Type * const gpioBase[] = GPIO_BASE_PTRS;

	gpioBase[PIN2PORT(pin)]->PDDR = ((gpioBase[PIN2PORT(pin)]->PDDR & ~gpioMask) | gpioValue); // Si es entrada o salida en el GPIO

}


void gpioWrite (pin_t pin, bool value)
{

	static PORT_Type * const portBase[] = PORT_BASE_PTRS;

	uint32_t periferic = portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & PORT_PCR_MUX_MASK;
	periferic = periferic >> PORT_PCR_MUX_SHIFT;
	if(!(periferic == 0b001))
	{
		return; // si el puerto no esta configurado como gpio saltea el resto
	}

	static GPIO_Type * const gpioBase[] = GPIO_BASE_PTRS; //arreglo de punteros a struct tipo GPIO_Type

	uint32_t gpioMask = (1<<PIN2NUM(pin)); // mascara para modificar el bit numero pin del PDOR
	uint32_t gpioValue = (((uint32_t)value)<<PIN2NUM(pin)); //ubico el valor en la pos pin

	gpioBase[PIN2PORT(pin)]->PDOR = ((gpioBase[PIN2PORT(pin)]->PDOR & ~gpioMask) | gpioValue);
}


void gpioToggle (pin_t pin)
{
	static PORT_Type * const portBase[] = PORT_BASE_PTRS;

	uint32_t periferic = portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & PORT_PCR_MUX_MASK;
	periferic = periferic >> PORT_PCR_MUX_SHIFT;
	if(!(periferic == 0b001))
	{
		return; // si el puerto no esta configurado como gpio saltea el resto
	}

	static GPIO_Type * const gpioBase[] = GPIO_BASE_PTRS;

	uint32_t gpioMask = (1<<PIN2NUM(pin));

	gpioBase[PIN2PORT(pin)]->PDOR = gpioBase[PIN2PORT(pin)]->PDOR ^ gpioMask;

}


bool gpioRead (pin_t pin)
{
	static GPIO_Type * const gpioBase[] = GPIO_BASE_PTRS;

	uint32_t gpioMask = (1<<PIN2NUM(pin));

	bool result = (gpioBase[PIN2PORT(pin)]->PDIR & gpioMask)>>PIN2NUM(pin);

	return result;
}

void gpioSetSlewRate(pin_t pin, bool slewRateLow)
{
	static PORT_Type * const portBase[] = PORT_BASE_PTRS;

	uint32_t periferic = portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & PORT_PCR_MUX_MASK;
	periferic = periferic >> PORT_PCR_MUX_SHIFT;
	if(!(periferic == 0b001))
	{
		return; // si el puerto no esta configurado como gpio saltea el resto
	}

	uint32_t portValue = (slewRateLow<<PORT_PCR_SRE_SHIFT); // el valor que debo ingresar al port
	uint32_t portMask = PORT_PCR_SRE_MASK; // mascara para modificar el pull enable, el pull set y el mux

	portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = ((portBase[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & ~portMask) | portValue);
}

void PortX_IRQImpl(uint8_t portNumber)
{
	static PORT_Type * const portBase[] = PORT_BASE_PTRS;
	uint32_t ISFR = portBase[portNumber]->ISFR;
	uint8_t contador = 0;
	while(!(ISFR & 0x01) && ISFR != 0)
	{
		ISFR = ISFR>>1;
		contador++;
	}

	if(contador == 0)
	{
		return;
	}

	portBase[portNumber]->ISFR = 1<<contador;
	CallbackAndState* p = &callbackMatrix[portNumber*32 + contador];
	p->pCallback(p->user_data);
}

__ISR__ PORTA_IRQHandler(void)
{
	PortX_IRQImpl(PA);
}

__ISR__ PORTB_IRQHandler(void)
{
	PortX_IRQImpl(PB);
}

__ISR__ PORTC_IRQHandler(void)
{
	PortX_IRQImpl(PC);
}

__ISR__ PORTD_IRQHandler(void)
{
	PortX_IRQImpl(PD);
}

__ISR__ PORTE_IRQHandler(void)
{
	PortX_IRQImpl(PE);
}

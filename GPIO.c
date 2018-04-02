/**
	\file
		GPIO.c
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014

 */
#include "MK64F12.h"
#include "GPIO.h"

/** Constant to clear interrupt*/
#define CLEAR_INTERRUPT 0xFFFFFFFF

static GPIO_interruptFlags_t GPIO_intrStatusFlag;

/**Actions taken when an interruption involving GPIOA occurs*/
void PORTA_IRQHandler()
{
	/**Sets interruption flag for PORTA*/
	GPIO_intrStatusFlag.flagPortA  = TRUE;

	/**Clears interruption of PORTA*/
	GPIO_clearInterrupt(GPIO_A);
}

/**Actions taken when an interruption involving GPIOB occurs*/
void PORTB_IRQHandler()
{
	/**Sets interruption flag for PORTB*/
	GPIO_intrStatusFlag.flagPortB = TRUE;

	/**Clears interruption of PORTB*/
	GPIO_clearInterrupt(GPIO_B);
}

/**Actions taken when an interruption involving GPIOC occurs*/
void PORTC_IRQHandler()
{
	/**Sets interruption flag for PORTC*/
	GPIO_intrStatusFlag.flagPortC = TRUE;

	/**Clears interruption of PORTC*/
	GPIO_clearInterrupt(GPIO_C);

}

/**Actions taken when an interruption involving GPIOD occurs*/
void PORTD_IRQHandler()
{
	/**Sets interruption flag for PORTD*/
	GPIO_intrStatusFlag.flagPortD = TRUE;

	/**Clears interruption of PORTD*/
	GPIO_clearInterrupt(GPIO_D);

}

/**Actions taken when an interruption involving GPIOE occurs*/
void PORTE_IRQHandler()
{
	/**Sets interruption flag for PORTE*/
	GPIO_intrStatusFlag.flagPortE = TRUE;

	/**Clears interruption of PORTE*/
	GPIO_clearInterrupt(GPIO_E);

}

/**Gets Interrupt Status from a GPIO*/
uint8 GPIO_getIRQStatus(GPIO_portNameType gpio)
{
	uint8 return_value;
	switch (gpio) {
		case GPIO_A:/**Interrupt Status in GPIOA*/
			return_value = GPIO_intrStatusFlag.flagPortA;
		break;
		case GPIO_B:/**Interrupt Status in GPIOB*/
			return_value = GPIO_intrStatusFlag.flagPortB;
		break;
		case GPIO_C:/**Interrupt Status in GPIOC*/
			return_value = GPIO_intrStatusFlag.flagPortC;
		break;
		case GPIO_D:/**Interrupt Status in GPIOD*/
			return_value = GPIO_intrStatusFlag.flagPortD;
		break;
		case GPIO_E:/**Interrupt Status in GPIOE*/
			return_value = GPIO_intrStatusFlag.flagPortE;
		break;
		default:/**If doesn't exist the option*/
			return_value = ERROR;
		break;
	}

	return (return_value);

}

/**Clears Interrupt Status from GPIO*/
uint8 GPIO_clearIRQStatus(GPIO_portNameType gpio)
{
	uint8 return_value = TRUE;
	switch (gpio)
	{
		case GPIO_A:/**Clear Interrupt Status in GPIOA*/
			GPIO_intrStatusFlag.flagPortA = FALSE;
		break;
		case GPIO_B:/**Clear Interrupt Status in GPIOB*/
			GPIO_intrStatusFlag.flagPortB = FALSE;
		break;
		case GPIO_C:/**Clear Interrupt Status in GPIOC*/
			GPIO_intrStatusFlag.flagPortC = FALSE;
		break;
		case GPIO_D:/**Clear Interrupt Status in GPIOD*/
			GPIO_intrStatusFlag.flagPortD = FALSE;
		break;
		case GPIO_E:/**Clear Interrupt Status in GPIOE*/
			GPIO_intrStatusFlag.flagPortE = FALSE;
		break;
		default:/**If doesn't exist the option*/
			return_value = ERROR;
		break;
	}

	return(return_value);

}


void GPIO_clearInterrupt(GPIO_portNameType portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR = CLEAR_INTERRUPT;
		break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR = CLEAR_INTERRUPT;
		break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = CLEAR_INTERRUPT;
		break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR = CLEAR_INTERRUPT;
		break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->ISFR = CLEAR_INTERRUPT;
		break;
		default: 	/** Leaves values as they are*/

		break;

	}// end switch
}

/*********************************************/

uint8 GPIO_clockGating(GPIO_portNameType portName)
{
	uint8 return_value = TRUE;
	switch(portName)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
		break;
		case GPIO_B: /** GPIO B is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
		break;
		case GPIO_C: /** GPIO C is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
		break;
		case GPIO_D: /** GPIO D is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
		break;
		case GPIO_E: /** GPIO E is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
		break;
		default: /**If doesn't exist the option*/
			return_value = FALSE;
		break;
	}// end switch

	/**Successful configuration*/
	return(return_value);

}// end function

uint8 GPIO_pinControlRegister(GPIO_portNameType portName,uint8 pin,const GPIO_pinControlRegisterType*  pinControlRegister)
{
	uint8 return_value = TRUE;
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pinControlRegister;
		break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pinControlRegister;
		break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pinControlRegister;
		break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pinControlRegister;
		break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin]= *pinControlRegister;
		default:/**If doesn't exist the option*/
			return_value = FALSE;
		break;
	}

	/**Successful configuration*/
	return(return_value);

}

/**
 * Adition of Functions by Leon Arpio & Ricardo Delsordo********
 */

/**Writes all the PINS from a specific GPIO.*/
void GPIO_writePORT(GPIO_portNameType portName, uint32 Data )
{
	switch(portName)
	{
		case GPIO_A:/**Write PINS from GPIO A*/
			GPIOA->PDOR = Data;
		break;
		case GPIO_B:/**Write PINS from GPIO B*/
			GPIOB->PDOR = Data;
		break;
		case GPIO_C:/**Write PINS from GPIO C*/
			GPIOC->PDOR = Data;
		break;
		case GPIO_D:/**Write PINS from GPIO D*/
			GPIOD->PDOR = Data;
		break;
		case GPIO_E: /**Write PINS from GPIO E*/
			GPIOE->PDOR = Data;
		break;
		default:	/** Leaves values as they are*/

		break;
	}
}

/**Reads all the PINS from a specific GPIO.*/
uint32 GPIO_readPORT(GPIO_portNameType portName)
{
	uint32 return_value;

	switch(portName)
	{
		case GPIO_A:/**Reads PINS from GPIO A*/
			return_value = GPIOA->PDOR;
		break;
		case GPIO_B:/**Reads PINS from GPIO B*/
			return_value = GPIOB->PDOR;
		break;
		case GPIO_C:/**Reads PINS from GPIO C*/
			return_value = GPIOC->PDOR;
		break;
		case GPIO_D:/**Reads PINS from GPIO D*/
			return_value = GPIOD->PDOR;
		break;
		case GPIO_E: /**Reads PINS from GPIO E*/
			return_value = GPIOE->PDOR;
		default:/**If doesn't exist the option*/
			return_value = ERROR;
		break;
	}

	return (return_value);
}

/**Returns if the desired PIN is 1 (TRUE) or 0 (FALSE).*/
uint32 GPIO_readPIN(GPIO_portNameType portName, uint8 pin)
{

	/**Variable takes selected position (PIN)*/
	uint32 set_value = 0x00000001 << pin;
	uint32 return_value;

	switch(portName)
	{
		case GPIO_A:/** Selected PIN for GPIOA*/
			return_value = GPIOA->PDIR & set_value;
		break;
		case GPIO_B:/** Selected PIN for GPIOB*/
			return_value = GPIOB->PDIR;
			return_value &= set_value;
		break;
		case GPIO_C:/** Selected PIN for GPIOC*/
			return_value = GPIOC->PDIR & set_value;
		break;
		case GPIO_D:/** Selected PIN for GPIOD*/
			return_value = GPIOD->PDIR & set_value;
		break;
		case GPIO_E: /** Selected PIN for GPIOE*/
			return_value = GPIOE->PDIR & set_value;
		break;
		default:/**If doesn't exist the option*/
			return_value = ERROR;
		break;
	}

	return (return_value);

}

/**Returns if the desired PIN is 1 (TRUE) or 0 (FALSE).*/
uint32 GPIO_readPortInput(GPIO_portNameType portName)
{
	uint32 return_value;

	switch(portName)
	{
		case GPIO_A:/** Selected PIN for GPIOA*/
			return_value = GPIOA->PDIR;
		break;
		case GPIO_B:/** Selected PIN for GPIOB*/
			return_value = GPIOB->PDIR;
		break;
		case GPIO_C:/** Selected PIN for GPIOC*/
			return_value = GPIOC->PDIR;
		break;
		case GPIO_D:/** Selected PIN for GPIOD*/
			return_value = GPIOD->PDIR;
		break;
		case GPIO_E: /** Selected PIN for GPIOE*/
			return_value = GPIOE->PDIR;
		break;
		default:/**If doesn't exist the option*/
			return_value = ERROR;
		break;
	}

	return (return_value);

}

/**Sets the value of a PIN to 1 */
void GPIO_setPIN(GPIO_portNameType portName, uint8 pin)
{
	/**Selects PIN*/
	uint32 set_value = 0x00000001 << pin;

	switch(portName)
	{
		case GPIO_A:/** Sets PIN in GPIOA*/
			GPIOA->PSOR = set_value;
		break;
		case GPIO_B:/** Sets PIN in GPIOB*/
			GPIOB->PSOR = set_value;
		break;
		case GPIO_C:/** Sets PIN in GPIOC*/
			GPIOC->PSOR = set_value;
		break;
		case GPIO_D:/** Sets PIN in GPIOD*/
			GPIOD->PSOR = set_value;
		break;
		case GPIO_E: /** Sets PIN in GPIOE*/
			GPIOE->PSOR = set_value;
		break;
		default: /** Leaves values as they are*/

		break;
	}

}

/** Sets the value of a PIN to 0*/
void GPIO_clearPIN(GPIO_portNameType portName, uint8 pin)
{
	uint32 set_value = 0x00000001 << pin;

	switch(portName)
	{
	case GPIO_A:/** Clears PIN in GPIOA*/
			GPIOA->PCOR = set_value;
		break;
		case GPIO_B:/** Clears PIN in GPIOB*/
			GPIOB->PCOR = set_value;
		break;
		case GPIO_C:/** Clears PIN in GPIOC*/
			GPIOC->PCOR = set_value;
		break;
		case GPIO_D:/** Clears PIN in GPIOD*/
			GPIOD->PCOR = set_value;
		break;
		case GPIO_E: /** Clears PIN in GPIOE*/
			GPIOE->PCOR = set_value;
		break;
		default:	/** Leaves values as they are*/

		break;
	}
}

/** Changes the value of PIN between 1 and 0*/
void GPIO_togglePIN(GPIO_portNameType portName, uint8 pin)
{
	uint32 set_value = 0x00000001 << pin;

	switch(portName)
	{
		case GPIO_A:/** Toggles PIN in GPIOA*/
			GPIOA->PTOR = set_value;
		break;
		case GPIO_B:/** Toggles PIN in GPIOB*/
			GPIOB->PTOR = set_value;
		break;
		case GPIO_C:/** Toggles PIN in GPIOC*/
			GPIOC->PTOR = set_value;
		break;
		case GPIO_D:/** Toggles PIN in GPIOD*/
			GPIOD->PTOR = set_value;
		break;
		case GPIO_E: /** Toggles PIN in GPIOE*/
			GPIOE->PTOR = set_value;
		break;
		default:	/** Leaves values as they are*/

		break;
	}

}

/** Configures PORT as input or output*/
void GPIO_dataDirectionPORT(GPIO_portNameType portName ,uint32 direction)
{
	switch(portName)
	{
		case GPIO_A:/** Configures GPIOA*/
			GPIOA->PDDR = direction;
		break;
		case GPIO_B:/** Configures GPIOB*/
			GPIOB->PDDR = direction;
		break;
		case GPIO_C:/** Configures GPIOC*/
			GPIOC->PDDR = direction;
		break;
		case GPIO_D:/** Configures GPIOD*/
			GPIOD->PDDR = direction;
		break;
		case GPIO_E: /** Configures GPIOE*/
			GPIOE->PDDR = direction;
		break;
		default: /** Leaves values as they are*/

		break;
	}
}

/** Configures PIN as input or output*/
void GPIO_dataDirectionPIN(GPIO_portNameType portName, uint8 state, uint8 pin)
{

	uint32 set_value = 0x00000001 << pin;
	/** Sets PIN*/
	if(state)
	{
		switch(portName)
		{
			case GPIO_A:/** Sets PIN in GPIOA*/
				GPIOA->PDDR |= set_value;
			break;
			case GPIO_B:/** Sets PIN in GPIOB*/
				GPIOB->PDDR |= set_value;
			break;
			case GPIO_C:/** Sets PIN in GPIOC*/
				GPIOC->PDDR |= set_value;
			break;
			case GPIO_D:/** Sets PIN in GPIOD*/
				GPIOD->PDDR |= set_value;
			break;
			case GPIO_E: /** Sets PIN in GPIOE*/
				GPIOE->PDDR |= set_value;
			break;
			default:	/** Leaves values as they are*/

			break;
		}
	}

	/** Clears PIN*/
	else
	{
		switch(portName)
		{
		case GPIO_A:	/** Clears PIN in GPIOA*/
			GPIOA->PDDR &= ~set_value;
		break;
		case GPIO_B:	/** Clears PIN in GPIOB*/
			GPIOB->PDDR &= ~set_value;
		break;
		case GPIO_C:	/** Clears PIN in GPIOC*/
			GPIOC->PDDR &= ~set_value;
		break;
		case GPIO_D:	/** Clears PIN in GPIOD*/
			GPIOD->PDDR &= ~set_value;
		break;
		case GPIO_E: 	/** Clears PIN in GPIOE*/
			GPIOE->PDDR &= ~set_value;
		break;
		default:		/** Leaves values as they are*/

		break;
		}
	}
}

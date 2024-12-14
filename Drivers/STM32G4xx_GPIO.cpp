/*
 * STM32F4xx_GPIO.cpp
 *
 *  Created on: Aug 26, 2023
 *      Author: efakb
 */

#include "STM32G4xx_GPIO.hpp"

/* 
* @brief Enable Clock for GPIO port
* @return None
*/
void GPIO::ClockEnable()
{
	switch(Port)
	{
	case 0:

		RCC->AHB1ENR.GPIOAEN = 1;

		break;

	case 1:

		RCC->AHB1ENR.GPIOBEN = 1;

		break;

	case 2:

		RCC->AHB1ENR.GPIOCEN = 1;

		break;

	case 3:

		RCC->AHB1ENR.GPIODEN = 1;

		break;

	case 4:

		RCC->AHB1ENR.GPIOEEN = 1;

		break;

	case 5:

		RCC->AHB1ENR.GPIOFEN = 1;

		break;

	case 6:

		RCC->AHB1ENR.GPIOGEN = 1;

		break;

	case 7:

		RCC->AHB1ENR.GPIOHEN = 1;

		break;

	case 8:

		RCC->AHB1ENR.GPIOIEN = 1;

		break;

	case 9:

		RCC->AHB1ENR.GPIOJEN = 1;

		break;

	case 10:

		RCC->AHB1ENR.GPIOKEN = 1;

		break;
	}
}

/*
* @brief Configures the GPIO peripheral
* @return None
*/
void GPIO::Init(){

	ClockEnable();												//Enabling the corresponding GPIO Clock

	GPIO_Port->MODER &= ~(0x3<<(Pin*2));						//Clearing the mode register
	GPIO_Port->MODER |= Mode<<(Pin*2);							//Setting the mode register

	GPIO_Port->OTYPER &= ~(0x1<<Pin);							//Clearing the output type register
	GPIO_Port->OTYPER |= Output_Type<<Pin;						//Setting the output type register

	GPIO_Port->OSPEEDR &= ~(0x3<<(Pin*2));						//Clearing the output speed register
	GPIO_Port->OSPEEDR |=  Output_Speed<<(Pin*2);				//Setting the output speed register

	GPIO_Port->PUPDR &= ~(0x3<<(Pin*2));						//Clearing the pull up pull down register
	GPIO_Port->PUPDR |=  PullUp_PullDown<<(Pin*2);  			//Setting the pull up pull down register

	GPIO_Port->AFR &= ~(0xF<<(Pin*4));							//Clearing the alternate function down register
	GPIO_Port->AFR |= (uint64_t)Alternative_Function<<(Pin*4); 	//Setting the pull up pull down register
}

/*
* @brief Initialization for the GPIO
* @param Pin_Name: GPIO pin name 
* @param Pin_Mode: GPIO pin mode [INPUT, OUTPUT, ALTFN, ANALOG]
* @param Pin_Output_Type: GPIO pin output type [PUSHPULL, OPENDRAIN]
* @param Pin_Output_Speed: GPIO pin output speed [SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH, SPEED_VERYHIGH]
* @param Pin_PUPD: GPIO pin out
*/
void GPIO::Init(uint8_t Pin_Name, Pin_Mode Pin_Mode, Pin_Output_Type Pin_Output_Type, Pin_Output_Speed Pin_Output_Speed, Pin_PUPD Pin_PUPD)
{
	Port = Pin_Name >> 4;

	Pin = Pin_Name & 0x0F;

	GPIO_Port = reinterpret_cast<GPIOx_RegDef_t*>(GPIO_BASEADDR + Port*GPIO_OFFSET);

	Mode = Pin_Mode;

	Output_Type = Pin_Output_Type;

	Output_Speed = Pin_Output_Speed;

	PullUp_PullDown = Pin_PUPD;

	Init();
}

/*
* @brief Constructor for the GPIO (uninitialized)
* @return None
*/
GPIO::GPIO()
{
}


/*
* @brief Constructor for the GPIO
* @param Pin_Name: GPIO pin name
*/
GPIO::GPIO(uint8_t Pin_Name)
{
	Port = Pin_Name >> 4;

	Pin = Pin_Name & 0x0F;

	GPIO_Port = reinterpret_cast<GPIOx_RegDef_t*>(GPIO_BASEADDR + Port*GPIO_OFFSET);

	Init();
}

/*
* @brief Constructor for the GPIO
* @param Pin_Name: GPIO pin name
* @param Pin_Mode: GPIO pin mode [INPUT, OUTPUT, ALTFN, ANALOG]
*/
GPIO::GPIO(uint8_t Pin_Name, Pin_Mode Pin_Mode)
:Mode(Pin_Mode)
{
	Port = Pin_Name >> 4;

	Pin = Pin_Name & 0x0F;

	GPIO_Port = reinterpret_cast<GPIOx_RegDef_t*>(GPIO_BASEADDR + Port*GPIO_OFFSET);

	Init();
}

/*
* @brief Constructor for the GPIO
* @param Pin_Name: GPIO pin name
* @param Pin_Mode: GPIO pin mode [INPUT, OUTPUT, ALTFN, ANALOG]
* @param Pin_Output_Type: GPIO pin output type [PUSHPULL, OPENDRAIN]
* @param Pin_Output_Speed: GPIO pin output speed [SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH, SPEED_VERYHIGH]
* @param Pin_PUPD: GPIO pin out
*/
GPIO::GPIO(uint8_t Pin_Name, Pin_Mode Pin_Mode, Pin_Output_Type Pin_Output_Type, Pin_Output_Speed Pin_Output_Speed, Pin_PUPD Pin_PUPD)
:Mode(Pin_Mode), Output_Type(Pin_Output_Type), Output_Speed(Pin_Output_Speed), PullUp_PullDown(Pin_PUPD)
{
	Port = Pin_Name >> 4;

	Pin = Pin_Name & 0x0F;

	GPIO_Port = reinterpret_cast<GPIOx_RegDef_t*>(GPIO_BASEADDR + Port*GPIO_OFFSET);

	Output_Speed = Pin_Output_Speed;

	PullUp_PullDown = Pin_PUPD;

	Init();
}

/*
* @brief Set the GPIO pin alternate function
* @param AltFn: Alternate function number [0~15]
* @return None
*/
void GPIO::SetAltFn(uint8_t AltFN)
{
	Mode = ALTFN;

	Alternative_Function = AltFN;

	Init();
}

/*
* @brief Read the GPIO pin state
* @return GPIO pin state [0 = Pin Status Low, 1 = Pin Status High]
*/
uint8_t GPIO::Read()
{
	return (uint8_t)((GPIO_Port->IDR >> Pin) & 0x1);
}

/*
* @brief Write the GPIO pin state
* @param value: GPIO pin state [0 = Pin Status Low, 1 = Pin Status High]
* @return None
*/
void GPIO::Write(uint8_t value)
{
	GPIO_Port->ODR = (GPIO_Port->ODR & ~(0x1 << Pin)) | (value << Pin);
}

/*
* @brief Switch the GPIO pin state. If the pin state low, this function change the state to High
* @return None
*/
void GPIO::Toggle()
{
	GPIO_Port->ODR ^= ( 1 << Pin);
}

/*
* @brief Attach interrupt to GPIO
* @param *callback: callback function pointer
* @param EXTI_Type: interrupt occour type [EXTI_Trigger_Rising, EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling]
*/
void GPIO::AttachExtInterrupt(void (*callback)(void), EXTI_Trigger EXTI_Type)
{

	//Configure the event type.
	switch (EXTI_Type)
	{
	case 0: //Case: Rising Trigger

		EXTI_Reg->RTSR |= (0x1<<Pin);	//Set Rising edge trigger

		EXTI_Reg->FTSR &= ~(0x1<<Pin);	//Clear Falling edge trigger

		break;

	case 1:	//Case: Falling Trigger

		EXTI_Reg->RTSR &= ~(0x1<<Pin);	//Clear Rising edge trigger

		EXTI_Reg->FTSR |= (0x1<<Pin);	//Clear Falling edge trigger

		break;

	case 2:	//Case: Both Rising and Falling Trigger

		EXTI_Reg->RTSR |= (0x1<<Pin);	//Set Rising edge trigger

		EXTI_Reg->FTSR |= (0x1<<Pin);	//Set Falling edge trigger

		break;
	}

	RCC->APB2ENR.SYSCFGEN = 1;	// Enable the sysconfig clock in case it was not already enabled.
	
	switch (Pin)
	{
	case 0:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI0 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI0>>5] |= 1<<IRQ_NO_EXTI0;

		break;

	case 1:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI1 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI1>>5] |= 1<<IRQ_NO_EXTI1;

		break;

	case 2:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI2 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI2>>5] |= 1<<IRQ_NO_EXTI2;

		break;

	case 3:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI3 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI3>>5] |= 1<<IRQ_NO_EXTI3;

		break;

	case 4:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI4 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI4>>5] |= 1<<IRQ_NO_EXTI4;

		break;

	case 5:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI5 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI9_5>>5] |= 1<<IRQ_NO_EXTI9_5;

		break;

	case 6:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI6 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI9_5>>5] |= 1<<IRQ_NO_EXTI9_5;

		break;

	case 7:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI7 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI9_5>>5] |= 1<<IRQ_NO_EXTI9_5;

		break;

	case 8:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI8 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI9_5>>5] |= 1<<IRQ_NO_EXTI9_5;

		break;

	case 9:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI9 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI9_5>>5] |= 1<<IRQ_NO_EXTI9_5;

		break;

	case 10:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI10 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI15_10>>5] |= 1<<(IRQ_NO_EXTI15_10%32);

		break;

	case 11:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI11 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI15_10>>5] |= 1<<(IRQ_NO_EXTI15_10%32);

		break;

	case 12:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI12 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI15_10>>5] |= 1<<(IRQ_NO_EXTI15_10%32);

		break;

	case 13:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI13 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI15_10>>5] |= 1<<(IRQ_NO_EXTI15_10%32);

		break;

	case 14:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI14 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI15_10>>5] |= 1<<(IRQ_NO_EXTI15_10%32);

		break;

	case 15:
		//Assign EXTI to corresponding PORT (A,B,C,..K)
		SYSCFG_Reg->EXTI15 = Port;
		//Attach callback function
		EXTI_callback[Pin] = callback;
		//Enable interrupt
		NVIC_Reg->ISER[IRQ_NO_EXTI15_10>>5] |= 1<<(IRQ_NO_EXTI15_10%32);

		break;

	}

	//Enable the EXTI interrupt using interrupt mask registers IMR

	EXTI_Reg->IMR |= (1<<Pin);

}

/*
* @brief Returns the Port who owns the specific EXTI line
* @return Port ID
*/
uint8_t GPIO::GetExtInterruptOwner()
{
	//Returns the Port who owns the specific EXTI line
	uint8_t ret;

		switch (Pin)
		{
		case 0:

			ret = (uint8_t) (SYSCFG_Reg->EXTI0);

			break;

		case 1:

			ret = (uint8_t) (SYSCFG_Reg->EXTI1);

			break;

		case 2:

			ret = (uint8_t) (SYSCFG_Reg->EXTI2);

			break;

		case 3:

			ret = (uint8_t) (SYSCFG_Reg->EXTI3);

			break;

		case 4:

			ret = (uint8_t) (SYSCFG_Reg->EXTI4);

			break;

		case 5:

			ret = (uint8_t) (SYSCFG_Reg->EXTI5);

			break;

		case 6:

			ret = (uint8_t) (SYSCFG_Reg->EXTI6);

			break;

		case 7:

			ret = (uint8_t) (SYSCFG_Reg->EXTI7);

			break;

		case 8:

			ret = (uint8_t) (SYSCFG_Reg->EXTI8);

			break;

		case 9:

			ret = (uint8_t) (SYSCFG_Reg->EXTI9);

			break;

		case 10:

			ret = (uint8_t) (SYSCFG_Reg->EXTI10);

			break;

		case 11:

			ret = (uint8_t) (SYSCFG_Reg->EXTI11);

			break;

		case 12:

			ret = (uint8_t) (SYSCFG_Reg->EXTI12);

			break;

		case 13:

			ret = (uint8_t) (SYSCFG_Reg->EXTI13);

			break;

		case 14:

			ret = (uint8_t) (SYSCFG_Reg->EXTI14);

			break;

		case 15:

			ret = (uint8_t) (SYSCFG_Reg->EXTI15);

			break;

		}

		return ret;

}

/*
* @brief Enables the interrupt
* @return None
*/
void GPIO::EnableExtInterrupt(void)
{
	if (EXTI_callback[Pin] == nullptr)
	{
		//callback function is not defined. Raise CONFIG_ERROR.
		CONFIG_ERROR
	}

	//Check if the Port owns the interrupt
	if (GetExtInterruptOwner()==Port)
	{
		EXTI_Reg->IMR |= (1<<Pin);
	}

}

/*
* @brief Disables the interrupt
* @return None
*/
void GPIO::DisableExtInterrupt(void)
{
	//Check if the Port owns the interrupt
	if (GetExtInterruptOwner()==Port)
	{
		EXTI_Reg->IMR &= ~(1<<Pin);
	}
}

/*
* @brief Sets the interrupt priority
* @return None
*/
void GPIO::SetExtInterruptPriority(uint8_t Priority)
{
	if (Pin>9)
	{
		NVIC_Reg->IPR[IRQ_NO_EXTI15_10] = Priority <<4;
	}

	else if (Pin>4)
	{
		NVIC_Reg->IPR[IRQ_NO_EXTI9_5] = Priority <<4;
	}

	else
	{
		switch (Pin)
		{
		case 0:

			NVIC_Reg->IPR[IRQ_NO_EXTI0] = Priority <<4;

			break;

		case 1:

			NVIC_Reg->IPR[IRQ_NO_EXTI1] = Priority <<4;

			break;

		case 2:

			NVIC_Reg->IPR[IRQ_NO_EXTI2] = Priority <<4;

			break;

		case 3:

			NVIC_Reg->IPR[IRQ_NO_EXTI3] = Priority <<4;

			break;

		case 4:

			NVIC_Reg->IPR[IRQ_NO_EXTI4] = Priority <<4;

			break;
		}
	}
}

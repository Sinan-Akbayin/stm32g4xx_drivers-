/*
 * STM32F4xx_IRQ.cpp
 *
 *  Created on: Aug 27, 2023
 *      Author: efakb
 */

#include <STM32G4xx_IT.hpp>

void SysTick_Handler()
{
	SystemTick++;
}


/*
 * External Interrupt Handlers
 */
void (*EXTI_callback[16])(void);

void EXTI0_IRQHandler()
{
	//Call Interrupt Service Functions if they exist
	if (EXTI_callback[0] != nullptr)
	{
		EXTI_callback[0]();
	}

	//Reset EXTI Pending Register by setting corresponding bit
	EXTI_Reg->PR |= (1<<0);
}

void EXTI1_IRQHandler()
{
	//Call Interrupt Service Functions if they exist
	if (EXTI_callback[1] != nullptr)
	{
		EXTI_callback[1]();
	}

	//Reset EXTI Pending Register by setting corresponding bit
	EXTI_Reg->PR |= (1<<1);
}

void EXTI2_IRQHandler()
{
	//Call Interrupt Service Functions if they exist
	if (EXTI_callback[2] != nullptr)
	{
		EXTI_callback[2]();
	}

	//Reset EXTI Pending Register by setting corresponding bit
	EXTI_Reg->PR |= (1<<2);
}

void EXTI3_IRQHandler()
{
	//Call Interrupt Service Functions if they exist
	if (EXTI_callback[3] != nullptr)
	{
		EXTI_callback[3]();
	}

	//Reset EXTI Pending Register by setting corresponding bit
	EXTI_Reg->PR |= (1<<3);
}

void EXTI4_IRQHandler()
{
	//Call Interrupt Service Functions if they exist
	if (EXTI_callback[4] != nullptr)
	{
		EXTI_callback[4]();
	}
	//Reset EXTI Pending Register by setting corresponding bits
	EXTI_Reg->PR |= (1<<4);
}

void EXTI9_5_IRQHandler()
{
	//Call Interrupt Service Functions if they exist
	if (EXTI_callback[5] != nullptr)
	{
		EXTI_callback[5]();
	}

	if (EXTI_callback[6] != nullptr)
	{
		EXTI_callback[6]();
	}

	if (EXTI_callback[7] != nullptr)
	{
		EXTI_callback[7]();
	}

	if (EXTI_callback[8] != nullptr)
	{
		EXTI_callback[8]();
	}
	if (EXTI_callback[9] != nullptr)
	{
		EXTI_callback[9]();
	}

	//Reset EXTI Pending Register by setting corresponding bits
	EXTI_Reg->PR |= 0b11111<<5;
}

void EXTI15_10_IRQHandler()
{
	//Call Interrupt Service Functions if they exist
	if (EXTI_callback[10] != nullptr)
	{
		EXTI_callback[10]();
	}

	if (EXTI_callback[11] != nullptr)
	{
		EXTI_callback[11]();
	}

	if (EXTI_callback[12] != nullptr)
	{
		EXTI_callback[12]();
	}

	if (EXTI_callback[13] != nullptr)
	{
		EXTI_callback[13]();
	}

	if (EXTI_callback[14] != nullptr)
	{
		EXTI_callback[14]();
	}
	if (EXTI_callback[15] != nullptr)
	{
		EXTI_callback[15]();
	}

	//Reset EXTI Pending Register by setting corresponding bits
	EXTI_Reg->PR |= 0b111111<<10;
}


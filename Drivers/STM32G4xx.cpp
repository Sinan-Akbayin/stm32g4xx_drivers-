/*
 * sysinit.cpp
 *
 *  Created on: Aug 30, 2023
 *      Author: efakb
 */
#include <STM32G4xx_IT.hpp>
#include "STM32G4xx.hpp"

uint32_t System_Clock_Freq; //System Clock Frequency in Hz

uint32_t SystemTick = 0;		//System Tick holds the time

void SysClockConfig(void)
{
	RCC->CR.HSEON = 1;				//Enabling the high speed external oscillator (HSE)

	while(!RCC->CR.HSERDY);			//Waiting for the HSE to stabilize

	RCC->APB1ENR.PWREN = 1;			//Enabling the power interface clock

	PWR->CR.VOS = 1;				//Set to voltage scale 1 (for high power)

	FLASH->ACR.PRFTEN = 1;			//Prefetch is enabled

	FLASH->ACR.ICEN = 1; 			//Instruction cache is enabled is enabled

	FLASH->ACR.DCEN = 1;			//Data cache is enabled

	FLASH->ACR.LATENCY = 5;			//Five wait states (see Table 10 on page 80 in RM0090)

	RCC->CFGR.HPRE = 0;				//Setting the AHB1 clock divider to 1

	RCC->CFGR.PPRE1 = 0b101;		//Setting the APB1 clock divider to 4

	RCC->CFGR.PPRE2 = 0b100;		//Setting the APB2 clock divider to 2

	RCC->PLLCFGR.PLLM = 8;			//Setting the M value of the PLL

	RCC->PLLCFGR.PLLN = 336;		//Setting the N value of the PLL

	RCC->PLLCFGR.PLLP = 0;			//Setting the P value of the PLL

	RCC->PLLCFGR.PLLQ = 7;			//Setting the Q value of the PLL

	RCC->PLLCFGR.PLLSRC = 1;		//Setting HSE as the PLL source

	RCC->CR.PLLON = 1;				//Enabling the PLL

	while(!RCC->CR.PLLRDY);			//Waiting for the PLL to stabilize

	RCC->CFGR.SW = 2;				//PLL selected as system clock

	while (!(RCC->CFGR.SWS==2)); 	//Waiting for source switch to be made

	System_Clock_Freq = SYSTEM_CORE_CLOCK;	//Setting the system clock to new value
}

void SysTickConfig(void)
{
	//Set system tick freq.
	SYSTICK->STRVR = System_Clock_Freq/System_Tick_Freq;

	//Setting SyTick source to the processor clock

	SYSTICK->STCSR |= (1<<2);

	//Enable SysTick Interrupt

	SYSTICK->STCSR |= (1<<1);

	//Enable Systick Counter

	SYSTICK->STCSR |= 1;
}


void SysConfig(void)
{
	SysClockConfig();			//Setting System Clock

	SysTickConfig();			//Enabling the System Tick

	*CPACR |= (0xF<<20); 		// Enabling the access for FPU
}

void SysSleep(uint32_t ms){
	volatile uint32_t current_tick = SystemTick;
	while (SystemTick < current_tick + ms);
}

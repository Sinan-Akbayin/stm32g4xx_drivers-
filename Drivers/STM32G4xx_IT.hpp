/*
 * STM32F4xx_IRQ.hpp
 *
 *  Created on: Aug 27, 2023
 *      Author: efakb
 */

#pragma once
#include "STM32G4xx.hpp"

//Define Error Types

#define CONFIG_ERROR while(1){}

//IRQ Numbers for STM32F4xx

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_EXTI15_10	40
#define IRQ_NO_SPI3			51

/*
 * NVIC Registers for CORTEX M4 Processors
 */
//

typedef struct
{
	volatile uint32_t ISER[8]; //Set enable registers
	uint32_t reserved0 [24];
	volatile uint32_t ICER[8]; //Clear enable registers
	uint32_t reserved1 [24];
	volatile uint32_t ISPR[8]; //Set pending registers
	uint32_t reserved2 [24];
	volatile uint32_t ICPR[8]; //Clear pending registers
	uint32_t reserved3 [24];
	volatile uint32_t IABR[8]; //Active bit register
	uint32_t reserved4 [56];
	volatile uint8_t IPR[240]; //Priority register
}NVIC_Typedef_t;


/*
 * NVIC Pointer Definitions
 */

#define NVIC_Reg 		((NVIC_Typedef_t*)NVIC_BASEADDR)

//Function Pointers for callbacks

extern void (*EXTI_callback[16])(void);

void IRQ_Config(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);

#ifdef __cplusplus
	extern "C" {
#endif

//void NMI_Handler();
//void HardFault_Handler();
//void MemManage_Handler();
//void BusFault_Handler();
//void UsageFault_Handler();
//void SVC_Handler();
//void DebugMon_Handler();
//void PendSV_Handler();
void SysTick_Handler();
//void WWDG_IRQHandler();
//void PVD_IRQHandler();
//void TAMP_STAMP_IRQHandler();
//void RTC_WKUP_IRQHandler();
//void RCC_IRQHandler();
void EXTI0_IRQHandler();
void EXTI1_IRQHandler();
void EXTI2_IRQHandler();
void EXTI3_IRQHandler();
void EXTI4_IRQHandler();
//void DMA1_Stream0_IRQHandler();
//void DMA1_Stream1_IRQHandler();
//void DMA1_Stream2_IRQHandler();
//void DMA1_Stream3_IRQHandler();
//void DMA1_Stream4_IRQHandler();
//void DMA1_Stream5_IRQHandler();
//void DMA1_Stream6_IRQHandler();
//void ADC_IRQHandler();
//void CAN1_TX_IRQHandler();
//void CAN1_RX0_IRQHandler();
//void CAN1_RX1_IRQHandler();
//void CAN1_SCE_IRQHandler();
void EXTI9_5_IRQHandler();
//void TIM1_BRK_TIM9_IRQHandler();
//void TIM1_UP_TIM10_IRQHandler();
//void TIM1_TRG_COM_TIM11_IRQHandler();
//void TIM1_CC_IRQHandler();
//void TIM2_IRQHandler();
//void TIM3_IRQHandler();
//void TIM4_IRQHandler();
//void I2C1_EV_IRQHandler();
//void I2C1_ER_IRQHandler();
//void I2C2_EV_IRQHandler();
//void I2C2_ER_IRQHandler();
//void USART1_IRQHandler();
//void USART2_IRQHandler();
//void USART3_IRQHandler();
void EXTI15_10_IRQHandler();
//void RTC_Alarm_IRQHandler();
//void OTG_FS_WKUP_IRQHandler();
//void TIM8_BRK_TIM12_IRQHandler();
//void TIM8_UP_TIM13_IRQHandler();
//void TIM8_TRG_COM_TIM14_IRQHandler();
//void TIM8_CC_IRQHandler();
//void DMA1_Stream7_IRQHandler();
//void FSMC_IRQHandler();
//void SDIO_IRQHandler();
//void TIM5_IRQHandler();
//void UART4_IRQHandler();
//void UART5_IRQHandler();
//void TIM6_DAC_IRQHandler();
//void TIM7_IRQHandler();
//void DMA2_Stream0_IRQHandler();
//void DMA2_Stream1_IRQHandler();
//void DMA2_Stream2_IRQHandler();
//void DMA2_Stream3_IRQHandler();
//void DMA2_Stream4_IRQHandler();
//void ETH_IRQHandler();
//void ETH_WKUP_IRQHandler();
//void CAN2_TX_IRQHandler();
//void CAN2_RX0_IRQHandler();
//void CAN2_RX1_IRQHandler();
//void CAN2_SCE_IRQHandler();
//void OTG_FS_IRQHandler();
//void DMA2_Stream5_IRQHandler();
//void DMA2_Stream6_IRQHandler();
//void DMA2_Stream7_IRQHandler();
//void USART6_IRQHandler();
//void I2C3_EV_IRQHandler();
//void I2C3_ER_IRQHandler();
//void OTG_HS_EP1_OUT_IRQHandler();
//void OTG_HS_EP1_IN_IRQHandler();
//void OTG_HS_WKUP_IRQHandler();
//void OTG_HS_IRQHandler();
//void DCMI_IRQHandler();
//void CRYP_IRQHandler();
//void HASH_RNG_IRQHandler();
//void FPU_IRQHandler();
//void LCD_TFT_IRQHandler();
//void LCD_TFT_1_IRQHandler();

#ifdef __cplusplus
}
#endif

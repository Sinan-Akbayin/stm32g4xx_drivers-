/*
 * STM32F4xx.h
 *
 *  Created on: Jun 30, 2022
 *      Author: efakb
 */

#pragma once

#include <stdint.h>

#define System_Tick_Freq 1000 //Hz

#define SYSTEM_CORE_CLOCK	170000000 //Hz

extern uint32_t System_Clock_Freq; //Hz

extern uint32_t SystemTick; //System Tick holds time value since syatem initiated.

//System Initialization Functions Prototypes
void SysClockConfig(void);
void SysTickConfig(void);
void SysConfig(void);
void SysSleep(uint32_t ms);

/******************************************BASE ADRESSES OF REGISTERS***********************************************/
//Memory map of the Cortex M4 Processors. This is from memory map section of the "Generic User Guide" (Cortex?-M4 Devices)

//SYSTEM TICK REGISTERS
#define SYSTICK_BASEADDR				0X40021000U

//NESTED VECTOR INTERRUPT CONTROLLER (SOFTWARE TRIGGER) REGISTERS
#define NVICSTIR_BASEADDR				0XE000EF00U

//NESTED VECTOR INTERRUPT CONTROLLER REGISTERS
#define NVIC_BASEADDR					0XE000E100U

//MEMORY PROTECTION UNIT RREGISTERS

#define MPU_BASEADDR					0XE000ED90U

//ID REGISTERS

#define ID_BASEADDR						0XE000ED00U

//SYSTEM CONTROL REGISTERS FOR THE FP EXTENSION

#define FPE_BASEADDR					0XE000EF34U

//SYSTEM CONTROL REGISTERS

#define CONTROL_BASEADDR				0XE000E008U
#define CPACR_BASEADDR					0XE000ED88U

//Memory map of the STM32F4 uC. This is from memory map section of the reference manual.

//APB1 PERIPHERALS BASE ADDRESS
#define APB1PERIPH_BASEADDR				0x40000000U

//TIM2-7,12-14 REGISTER
#define TIM2_BASEADDR					0x40000000U
#define TIM3_BASEADDR					0x40000400U
#define TIM4_BASEADDR					0x40000800U
#define TIM5_BASEADDR					0x40000C00U
#define TIM6_BASEADDR					0x40001000U
#define TIM7_BASEADDR					0x40001400U
#define TIM12_BASEADDR					0x40001800U
#define TIM13_BASEADDR					0x40001C00U
#define TIM14_BASEADDR					0x40002000U

//RTC BACKUP REGISTER
#define RTCBKP_BASEADDR					0x40002800U

//WATCHDOG REGISTER
#define WWDG_BASEADDR					0x40002C00U
#define IWDG_BASEADDR					0x40003000U

//SPI and I2S REGISTER
#define I2S2ext_BASEADDR				0x40003400U
#define SPI2_BASEADDR					0x40003800U
#define SPI3_BASEADDR					0x40003C00U
#define I2S3ext_BASEADDR				0x40004000U

//USART2,3 and UART4,5 REGISTER
#define USART2_BASEADDR					0x40004400U
#define USART3_BASEADDR					0x40004800U
#define UART4_BASEADDR					0x40004C00U
#define UART5_BASEADDR					0x40005000U

//I2C1-3 REGISTER
#define I2C1_BASEADDR					0x40005400U
#define I2C2_BASEADDR					0x40005800U
#define I2C3_BASEADDR					0x40005C00U

//CAN1-2 REGISTER
#define CAN1_BASEADDR					0x40006400U
#define CAN2_BASEADDR					0x40006800U

//PWR REGISTER
#define PWR_BASEADDR					0x40007000U

//DAC REGISTER
#define DAC_BASEADDR					0x40007400U

//UART7,8 REGISTER
#define UART7_BASEADDR					0x40007800U
#define UART8_BASEADDR					0x40007C00U


/*
 *  Base addresses of peripherals that is connected to APB2 Bus
 */

//BASE ADDRESS
#define APB2PERIPH_BASEADDR				0x40010000U

//TIM1,8 REGISTER
#define TIM1_BASEADDR					0x40010000U
#define TIM8_BASEADDR					0x40010400U

//USART1,6 REGISTER
#define USART1_BASEADDR					0x40011000U
#define USART6_BASEADDR					0x40011400U

//ADC1,2,3 REGISTER
#define ADC1_BASEADDR					0x40012000U
#define ADC2_BASEADDR					0x40012100U
#define ADC3_BASEADDR					0x40012200U

//SDIO REGISTER
#define SDIO_BASEADDR					0x40012C00U

//SPI1,4 REGISTER
#define SPI1_BASEADDR					0x40013000U
#define SPI2_BASEADDR					0x40003800U
#define SPI3_BASEADDR					0x40003C00U
#define SPI4_BASEADDR					0x40013400U

//SYSCONFIG REGISTER
#define SYSCONFIG_BASEADDR				0x40013800U

//EXTI REGISTER
#define EXTI_BASEADDR					0x40013C00U

//TIM9,10,11 REGISTER
#define TIM9_BASEADDR					0x40014000U
#define TIM10_BASEADDR					0x40014400U
#define TIM11_BASEADDR					0x40014800U

//SPI5,6 REGISTER
#define SPI5_BASEADDR					0x40015000U
#define SPI6_BASEADDR					0x40015400U

//SAI REGISTER
#define SAI_BASEADDR					0x40015800U

//LCD-TFT REGISTER
#define LCDTFT_BASEADDR					0x40016800U

/*
 *  Base addresses of peripherals that is connected to AHB1 Bus
 */

//BASE ADDRESS
#define AHB1PERIPH_BASEADDR				0x40020000U

//GPIOA-K REGISTER
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR					(GPIOA_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR					(GPIOB_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR					(GPIOC_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR					(GPIOD_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR					(GPIOE_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR					(GPIOF_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR					(GPIOG_BASEADDR + 0x1C00U)
#define GPIOI_BASEADDR					(GPIOH_BASEADDR + 0x2000U)
#define GPIOJ_BASEADDR					(GPIOI_BASEADDR + 0x2400U)
#define GPIOK_BASEADDR					(GPIOJ_BASEADDR + 0x2800U)

//CRC REGISTER
#define CRC_BASEADDR					0x40023000U

//RCC REGISTER
#define RCC_BASEADDR 					0x40023800U

//FLASH INTERFACE REGISTER
#define FLASH_BASEADDR					0x40023C00U

//BKPSRAM REGISTER
#define BKPSRAM_BASEADDR				0x40024000U

//DMA1,2 REGISTER
#define DMA1_BASEADDR					0x40026000U
#define DMA2_BASEADDR					0x40026400U

//ETHERNET MAC REGISTER
#define ETHERNETMAC_BASEADDR			0x40028000U

//DMA2D REGISTER
#define DMA2D_BASEADDR					0x4002B000U

//USB OTG HS REGISTER
#define USBOTGHS_BASEADDR				0x40040000U

/*
 *  Base addresses of peripherals that is connected to AHB2 Bus
 */

//BASE ADDRESS
#define AHB2PERIPH_BASEADDR				0x50000000U

//USB OTG FS REGISTER
#define USBOTGFS_BASEADDR				0x50000000U

//DCMI REGISTER
#define DCMI_BASEADDR					0x50050000U

//CRYP REGISTER
#define CRYP_BASEADDR					0x50060000U

//HASH REGISTER
#define HASH_BASEADDR					0x50060400U

//RNG REGISTER
#define RNG_BASEADDR					0x50060800U

/*
 *  Base addresses of peripherals that is connected to AHB3 Bus
 */

//BASE ADDRESS
#define AHB3PERIPH_BASEADDR				0xA0000000U

/******************************************TYPDEFs FOR REGISTERS***********************************************/

/*
 * SysTick Registers (CM4)
 */

typedef struct
{

	volatile uint32_t STCSR;
	volatile uint32_t STRVR;
	volatile uint32_t STCVR;
	volatile uint32_t STCR;

}SysTick_t;

/*
 * System Core Clock Definition
 */

//volatile uint32_t System_Clock; //Default system core clock speed at startup in Hz (8MHz)

/**********************************************************************************************************************/
/*/
 * Reset and Clock Control (RCC) register typedefs.
 */

//RCC clock control register (RCC_CR)
typedef struct
{

	volatile uint32_t HSION:1;
	volatile uint32_t HSIRDY:1;
	uint32_t reserved0:1;
	volatile uint32_t HSITRM:5;
	volatile uint32_t HSICAL:8;
	volatile uint32_t HSEON:1;
	volatile uint32_t HSERDY:1;
	volatile uint32_t HSEBYP:1;
	volatile uint32_t CSSON:1;
	uint32_t reserved1:4;
	volatile uint32_t PLLON:1;
	volatile uint32_t PLLRDY:1;
	volatile uint32_t PLLI2SON:1;
	volatile uint32_t PLLI2SRDY:1;
	volatile uint32_t PLLSAION:1;
	volatile uint32_t PLLSAIRDY:1;
	uint32_t reserved2:2;

}RCC_CR_t;

//RCC PLL configuration register (RCC_PLLCFGR)
typedef struct
{

	volatile uint32_t PLLM:6;
	volatile uint32_t PLLN:9;
	uint32_t reserved0:1;
	volatile uint32_t PLLP:2;
	uint32_t reserved1:4;
	volatile uint32_t PLLSRC:1;
	uint32_t reserved2:1;
	volatile uint32_t PLLQ:4;
	uint32_t reserved3:4;

}RCC_PLLCFGR_t;

//RCC clock configuration register (RCC_CFGR)
typedef struct
{

	volatile uint32_t SW:2;
	volatile uint32_t SWS:2;
	volatile uint32_t HPRE:4;
	uint32_t reserved0:2;
	volatile uint32_t PPRE1:3;
	volatile uint32_t PPRE2:3;
	volatile uint32_t RTCPRE:5;
	volatile uint32_t MCO1:2;
	volatile uint32_t I2SSCR:1;
	volatile uint32_t MCO1PRE:3;
	volatile uint32_t MCO2PRE:3;
	volatile uint32_t MCO2:2;

}RCC_CFGR_t;


//RCC clock interrupt register (RCC_CIR)
typedef struct
{

	volatile uint32_t LSIRDYF:1;
	volatile uint32_t LSERDYF:1;
	volatile uint32_t HSIRDYF:1;
	volatile uint32_t HSERDYF:1;
	volatile uint32_t PLLRDYF:1;
	volatile uint32_t PLLI2SRDYF:1;
	volatile uint32_t PLLSAIRDYF:1;
	volatile uint32_t CSSF:1;
	volatile uint32_t LSIRDYIE:1;
	volatile uint32_t LSERDYIE:1;
	volatile uint32_t HSIRDYIE:1;
	volatile uint32_t HSERDYIE:1;
	volatile uint32_t PLLRDYIE:1;
	volatile uint32_t PLLI2SRDYIE:1;
	volatile uint32_t PLLSAIRDYIE:1;
	uint32_t reserved0:1;
	volatile uint32_t LSIRDYC:1;
	volatile uint32_t LSERDYC:1;
	volatile uint32_t HSIRDYC:1;
	volatile uint32_t HSERDYC:1;
	volatile uint32_t PLLRDYC:1;
	volatile uint32_t PLLI2SRDYC:1;
	volatile uint32_t PLLSAIRDYC:1;
	volatile uint32_t CSSC:1;
	uint32_t reserved1:8;

}RCC_CIR_t;

//RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
typedef struct
{

	volatile uint32_t GPIOARST:1;
	volatile uint32_t GPIOBRST:1;
	volatile uint32_t GPIOCRST:1;
	volatile uint32_t GPIODRST:1;
	volatile uint32_t GPIOERST:1;
	volatile uint32_t GPIOFRST:1;
	volatile uint32_t GPIOGRST:1;
	volatile uint32_t GPIOHRST:1;
	volatile uint32_t GPIOIRST:1;
	volatile uint32_t GPIOJRST:1;
	volatile uint32_t GPIOKRST:1;
	uint32_t reserved0:1;
	volatile uint32_t CRCRST:1;
	uint32_t reserved1:8;
	volatile uint32_t DMA1RST:1;
	volatile uint32_t DMA2RST:1;
	volatile uint32_t DMA2DRST:1;
	uint32_t reserved5:1;
	volatile uint32_t ETHMACRST:1;
	uint32_t reserved6:3;
	volatile uint32_t OTGHSRST:1;
	uint32_t reserved7:2;

}RCC_AHB1RSTR_t;

//RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
typedef struct
{

	volatile uint32_t DCMIRST:1;
	uint32_t reserved0:3;
	volatile uint32_t CRYPRST:1;
	volatile uint32_t HASHRST:1;
	volatile uint32_t RNGRST:1;
	volatile uint32_t OTGFSRST:1;
	uint32_t reserved1:24;


}RCC_AHB2RSTR_t;

//RCC AHB3 peripheral reset register (RCC_AHB3RSTR)
typedef struct
{

	volatile uint32_t FMCRST:1;
	uint32_t reserved0:31;

}RCC_AHB3RSTR_t;

//RCC APB1 peripheral reset register (RCC_APB1RSTR)
typedef struct
{

	volatile uint32_t TIM2RST:1;
	volatile uint32_t TIM3RST:1;
	volatile uint32_t TIM4RST:1;
	volatile uint32_t TIM5RST:1;
	volatile uint32_t TIM6RST:1;
	volatile uint32_t TIM7RST:1;
	volatile uint32_t TIM12RST:1;
	volatile uint32_t TIM13RST:1;
	volatile uint32_t TIM14RST:1;
	uint32_t reserved0:2;
	volatile uint32_t WWDGRST:1;
	uint32_t reserved1:2;
	volatile uint32_t SPI2RST:1;
	volatile uint32_t SPI3RST:1;
	uint32_t reserved2:1;
	volatile uint32_t USART2RST:1;
	volatile uint32_t USART3RST:1;
	volatile uint32_t UART4RST:1;
	volatile uint32_t UART5RST:1;
	volatile uint32_t I2C1RST:1;
	volatile uint32_t I2C2RST:1;
	volatile uint32_t I2C3RST:1;
	uint32_t reserved3:1;
	volatile uint32_t CAN1RST:1;
	volatile uint32_t CAN2RST:1;
	uint32_t reserved4:1;
	volatile uint32_t PWRRST:1;
	volatile uint32_t DACRST:1;
	volatile uint32_t UART7RST:1;
	volatile uint32_t UART8RST:1;

}RCC_APB1RSTR_t;

//RCC APB2 peripheral reset register (RCC_APB2RSTR)
typedef struct
{

	volatile uint32_t TIM1RST:1;
	volatile uint32_t TIM8RST:1;
	uint32_t reserved0:2;
	volatile uint32_t USART1RST:1;
	volatile uint32_t USART6RST:1;
	uint32_t reserved1:2;
	volatile uint32_t ADCRST:1;
	uint32_t reserved2:2;
	volatile uint32_t SDIORST:1;
	volatile uint32_t SPI1RST:1;
	volatile uint32_t SPI4RST:1;
	volatile uint32_t SYSCFGRST:1;
	uint32_t reserved3:1;
	volatile uint32_t TIM9RST:1;
	volatile uint32_t TIM10RST:1;
	volatile uint32_t TIM11RST:1;
	uint32_t reserved4:1;
	volatile uint32_t SPI5RST:1;
	volatile uint32_t SPI6RST:1;
	volatile uint32_t SAI1RST:1;
	uint32_t reserved5:3;
	volatile uint32_t LTDCRST:1;
	uint32_t reserved6:5;

}RCC_APB2RSTR_t;

//RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
typedef struct
{

	volatile uint32_t GPIOAEN:1;
	volatile uint32_t GPIOBEN:1;
	volatile uint32_t GPIOCEN:1;
	volatile uint32_t GPIODEN:1;
	volatile uint32_t GPIOEEN:1;
	volatile uint32_t GPIOFEN:1;
	volatile uint32_t GPIOGEN:1;
	volatile uint32_t GPIOHEN:1;
	volatile uint32_t GPIOIEN:1;
	volatile uint32_t GPIOJEN:1;
	volatile uint32_t GPIOKEN:1;
	uint32_t reserved0:1;
	volatile uint32_t CRCEN:1;
	uint32_t reserved1:5;
	volatile uint32_t BKPSRAMEN:1;
	uint32_t reserved2:1;
	volatile uint32_t CCMDATARAMEN:1;
	volatile uint32_t DMA1EN:1;
	volatile uint32_t DMA2EN:1;
	volatile uint32_t DMA2DEN:1;
	uint32_t reserved3:1;
	volatile uint32_t ETHMACEN:1;
	volatile uint32_t ETHMACTXEN:1;
	volatile uint32_t ETHMACRXEN:1;
	volatile uint32_t ETHMACPTPEN:1;
	volatile uint32_t OTGHSEN:1;
	volatile uint32_t OTGHSULPIEN:1;
	uint32_t reserved4:1;

}RCC_AHB1ENR_t;

//RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
typedef struct
{

	volatile uint32_t DCMIEN:1;
	uint32_t reserved0:3;
	volatile uint32_t CRYPEN:1;
	volatile uint32_t HASHEN:1;
	volatile uint32_t RNGEN:1;
	volatile uint32_t OTGFSEN:1;
	uint32_t reserved1:24;

}RCC_AHB2ENR_t;

//RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)
typedef struct
{

	volatile uint32_t FMCEN:1;
	uint32_t reserved0:31;

}RCC_AHB3ENR_t;

//RCC APB1 peripheral clock enable register (RCC_APB1ENR)
typedef struct
{

	volatile uint32_t TIM2EN:1;
	volatile uint32_t TIM3EN:1;
	volatile uint32_t TIM4EN:1;
	volatile uint32_t TIM5EN:1;
	volatile uint32_t TIM6EN:1;
	volatile uint32_t TIM7EN:1;
	volatile uint32_t TIM12EN:1;
	volatile uint32_t TIM13EN:1;
	volatile uint32_t TIM14EN:1;
	uint32_t reserved0:2;
	volatile uint32_t WWDGEN:1;
	uint32_t reserved1:2;
	volatile uint32_t SPI2EN:1;
	volatile uint32_t SPI3EN:1;
	uint32_t reserved2:1;
	volatile uint32_t USART2EN:1;
	volatile uint32_t USART3EN:1;
	volatile uint32_t UART4EN:1;
	volatile uint32_t UART5EN:1;
	volatile uint32_t I2C1EN:1;
	volatile uint32_t I2C2EN:1;
	volatile uint32_t I2C3EN:1;
	uint32_t reserved3:1;
	volatile uint32_t CAN1EN:1;
	volatile uint32_t CAN2EN:1;
	uint32_t reserved4:1;
	volatile uint32_t PWREN:1;
	volatile uint32_t DACEN:1;
	volatile uint32_t UART7EN:1;
	volatile uint32_t UART8EN:1;

}RCC_APB1ENR_t;

//RCC APB2 peripheral clock enable register (RCC_APB2ENR)
typedef struct
{

	volatile uint32_t TIM1EN:1;
	volatile uint32_t TIM8EN:1;
	uint32_t reserved0:2;
	volatile uint32_t USART1EN:1;
	volatile uint32_t USART6EN:1;
	uint32_t reserved1:2;
	volatile uint32_t ADC1EN:1;
	volatile uint32_t ADC2EN:1;
	volatile uint32_t ADC3EN:1;
	volatile uint32_t SDIOEN:1;
	volatile uint32_t SPI1EN:1;
	volatile uint32_t SPI4EN:1;
	volatile uint32_t SYSCFGEN:1;
	uint32_t reserved3:1;
	volatile uint32_t TIM9EN:1;
	volatile uint32_t TIM10EN:1;
	volatile uint32_t TIM11EN:1;
	uint32_t reserved4:1;
	volatile uint32_t SPI5EN:1;
	volatile uint32_t SPI6EN:1;
	volatile uint32_t SAI1EN:1;
	uint32_t reserved5:3;
	volatile uint32_t LTDCEN:1;
	uint32_t reserved6:5;

}RCC_APB2ENR_t;

//RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR)
typedef struct
{

	volatile uint32_t GPIOALPEN:1;
	volatile uint32_t GPIOBLPEN:1;
	volatile uint32_t GPIOCLPEN:1;
	volatile uint32_t GPIODLPEN:1;
	volatile uint32_t GPIOELPEN:1;
	volatile uint32_t GPIOFLPEN:1;
	volatile uint32_t GPIOGLPEN:1;
	volatile uint32_t GPIOHLPEN:1;
	volatile uint32_t GPIOILPEN:1;
	volatile uint32_t GPIOJLPEN:1;
	volatile uint32_t GPIOKLPEN:1;
	uint32_t reserved0:1;
	volatile uint32_t CRCLPEN:1;
	uint32_t reserved1:2;
	volatile uint32_t FLITFLPEN:1;
	volatile uint32_t SRAM1LPEN:1;
	volatile uint32_t SRAM2LPEN:1;
	volatile uint32_t BKPSRAMLPEN:1;
	volatile uint32_t SRAM3LPEN:1;
	uint32_t reserved2:1;
	volatile uint32_t DMA1LPEN:1;
	volatile uint32_t DMA2LPEN:1;
	volatile uint32_t DMA2DLPEN:1;
	uint32_t reserved3:1;
	volatile uint32_t ETHMACLPEN:1;
	volatile uint32_t ETHTXLPEN:1;
	volatile uint32_t ETHRXLPEN:1;
	volatile uint32_t ETHPTPLPEN:1;
	volatile uint32_t OTGHSLPEN:1;
	volatile uint32_t OTGHSULPILPEN:1;
	uint32_t reserved4:1;

}RCC_AHB1LPENR_t;

//RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR)
typedef struct
{

	volatile uint32_t DCMILPEN:1;
	uint32_t reserved0:3;
	volatile uint32_t CRYPLPEN:1;
	volatile uint32_t HASHLPEN:1;
	volatile uint32_t RNGLPEN:1;
	volatile uint32_t OTGFSLPEN:1;
	uint32_t reserved1:24;

}RCC_AHB2LPENR_t;

//RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR)
typedef struct
{

	volatile uint32_t FMCLPEN:1;
	uint32_t reserved0:31;

}RCC_AHB3LPENR_t;

//RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR)
typedef struct
{

	volatile uint32_t TIM2LPEN:1;
	volatile uint32_t TIM3LPEN:1;
	volatile uint32_t TIM4LPEN:1;
	volatile uint32_t TIM5LPEN:1;
	volatile uint32_t TIM6LPEN:1;
	volatile uint32_t TIM7LPEN:1;
	volatile uint32_t TIM12LPEN:1;
	volatile uint32_t TIM13LPEN:1;
	volatile uint32_t TIM14LPEN:1;
	uint32_t reserved0:2;
	volatile uint32_t WWDGLPEN:1;
	uint32_t reserved1:2;
	volatile uint32_t SPI2LPEN:1;
	volatile uint32_t SPI3LPEN:1;
	uint32_t reserved2:1;
	volatile uint32_t USART2LPEN:1;
	volatile uint32_t USART3LPEN:1;
	volatile uint32_t UART4LPEN:1;
	volatile uint32_t UART5LPEN:1;
	volatile uint32_t I2C1LPEN:1;
	volatile uint32_t I2C2LPEN:1;
	volatile uint32_t I2C3LPEN:1;
	uint32_t reserved3:1;
	volatile uint32_t CAN1LPEN:1;
	volatile uint32_t CAN2LPEN:1;
	uint32_t reserved4:1;
	volatile uint32_t PWRLPEN:1;
	volatile uint32_t DACLPEN:1;
	volatile uint32_t UART7LPEN:1;
	volatile uint32_t UART8LPEN:1;

}RCC_APB1LPENR_t;

//RCC APB2 peripheral clock enable in low power mode register (RCC_APB2LPENR)
typedef struct
{

	volatile uint32_t TIM1LPEN:1;
	volatile uint32_t TIM8LPEN:1;
	uint32_t reserved0:2;
	volatile uint32_t USART1LPEN:1;
	volatile uint32_t USART6LPEN:1;
	uint32_t reserved1:2;
	volatile uint32_t ADC1LPEN:1;
	volatile uint32_t ADC2LPEN:1;
	volatile uint32_t ADC3LPEN:1;
	volatile uint32_t SDIOLPEN:1;
	volatile uint32_t SPI1LPEN:1;
	volatile uint32_t SPI4LPEN:1;
	volatile uint32_t SYSCFGLPEN:1;
	uint32_t reserved3:1;
	volatile uint32_t TIM9LPEN:1;
	volatile uint32_t TIM10LPEN:1;
	volatile uint32_t TIM11LPEN:1;
	uint32_t reserved4:1;
	volatile uint32_t SPI5LPEN:1;
	volatile uint32_t SPI6LPEN:1;
	volatile uint32_t SAI1LPEN:1;
	uint32_t reserved5:3;
	volatile uint32_t LTDCLPEN:1;
	uint32_t reserved6:5;

}RCC_APB2LPENR_t;

//RCC Backup domain control register (RCC_BDCR)
typedef struct
{

	volatile uint32_t LSEON:1;
	volatile uint32_t LSERDY:1;
	volatile uint32_t LSEBYP:1;
	uint32_t reserved0:5;
	volatile uint32_t RTCSEL:2;
	uint32_t reserved1:5;
	volatile uint32_t RTCEN:1;
	volatile uint32_t BDRST:1;
	uint32_t reserved3:15;

}RCC_BDCR_t;

//RCC clock control & status register (RCC_CSR)
typedef struct
{

	volatile uint32_t LSION:1;
	volatile uint32_t LSIRDY:1;
	uint32_t reserved0:22;
	volatile uint32_t RMVF:1;
	volatile uint32_t BORRSTF:1;
	volatile uint32_t PINRSTF:1;
	volatile uint32_t PORRSTF:1;
	volatile uint32_t SFTRSTF:1;
	volatile uint32_t IWDGRSTF:1;
	volatile uint32_t WWDGRSTF:1;
	volatile uint32_t LPWRRSTF:1;

}RCC_CSR_t;

//RCC spread spectrum clock generation register (RCC_SSCGR)

typedef struct
{

	volatile uint32_t MODPER:13;
	volatile uint32_t INCSTEP:15;
	uint32_t reserved0:2;
	volatile uint32_t SPREADSEL:1;
	volatile uint32_t SSCGEN:1;

}RCC_SSCGR_t;

//RCC clock control & status register (RCC_PLLI2SCFGR)
typedef struct
{

	uint32_t reserved0:6;
	volatile uint32_t PLLI2SN:9;
	uint32_t reserved1:9;
	volatile uint32_t PLLI2SQ:4;
	volatile uint32_t PLLI2SR:3;
	uint32_t reserved2:1;

}RCC_PLLI2SCFGR_t;

//RCC PLL configuration register (RCC_PLLSAICFGR)
typedef struct
{

	uint32_t reserved0:6;
	volatile uint32_t PLLSAIN:9;
	uint32_t reserved1:9;
	volatile uint32_t PLLSAIQ:4;
	volatile uint32_t PLLSAIR:3;
	uint32_t reserved2:1;

}RCC_PLLSAICFGR_t;

//RCC Dedicated Clock Configuration Register (RCC_DCKCFGR)
typedef struct
{

	volatile uint32_t PLLS2DIVQ:5;
	uint32_t reserved0:3;
	volatile uint32_t PLLSAIDIVQ:5;
	uint32_t reserved1:3;
	volatile uint32_t PLLSAIDIVR:2;
	uint32_t reserved2:2;
	volatile uint32_t SAI1ASRC:2;
	volatile uint32_t SAI1BSRC:2;
	volatile uint32_t TIMPRE:1;
	uint32_t reserved3:7;

}RCC_DCKCFGR_t;

//Whole RCC registers.
typedef struct
{

	volatile RCC_CR_t CR;
	volatile RCC_PLLCFGR_t PLLCFGR;
	volatile RCC_CFGR_t CFGR;
	volatile RCC_CIR_t CIR;
	volatile RCC_AHB1RSTR_t AHB1RSTR;
	volatile RCC_AHB2RSTR_t AHB2RSTR;
	volatile RCC_AHB3RSTR_t AHB3RSTR;
	uint32_t reserved0;
	volatile RCC_APB1RSTR_t APB1RSTR;
	volatile RCC_APB2RSTR_t APB2RSTR;
	uint32_t reserved1[2];
	volatile RCC_AHB1ENR_t AHB1ENR;
	volatile RCC_AHB2ENR_t AHB2ENR;
	volatile RCC_AHB3ENR_t AHB3ENR;
	uint32_t reserved2;
	volatile RCC_APB1ENR_t APB1ENR;
	volatile RCC_APB2ENR_t APB2ENR;
	uint32_t reserved3[2];
	volatile RCC_AHB1LPENR_t AHB1LPENR;
	volatile RCC_AHB2LPENR_t AHB2LPENR;
	volatile RCC_AHB3LPENR_t AHB3LPENR;
	uint32_t reserved4;
	volatile RCC_APB1LPENR_t APB1LPENR;
	volatile RCC_APB2LPENR_t APB2LPENR;
	uint32_t reserved5[2];
	volatile RCC_BDCR_t BDCR;
	volatile RCC_CSR_t CSR;
	uint32_t reserved6[2];
	volatile RCC_SSCGR_t SSCGR;
	volatile RCC_PLLI2SCFGR_t PLLI2SCFGR;
	volatile RCC_PLLSAICFGR_t PLLSAICFGR;
	volatile RCC_DCKCFGR_t DCKCFGR;

} RCC_RegDef_t;

/********************************************************************************************************/

/*
 * Power control registers (STM32F405xx/07xx and STM32F415xx/17xx)
 */

//TODO: Implement power control registers for (STM32F42xxx and STM32F43xxx)

//PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t LPDS:1;
	volatile uint32_t PDDS:1;
	volatile uint32_t CWUF:1;
	volatile uint32_t CSBF:1;
	volatile uint32_t PVDE:1;
	volatile uint32_t PLS:3;
	volatile uint32_t DBP:1;
	volatile uint32_t FPDS:1;
	uint32_t reserved0:4;
	volatile uint32_t VOS:1;
	uint32_t reserved1:17;

}PWR_CR_t;


//PWR power control/status register (PWR_CSR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t WUF:1;
	volatile uint32_t SBF:1;
	volatile uint32_t PVDO:1;
	volatile uint32_t BRR:1;
	uint32_t reserved0:4;
	volatile uint32_t EWUP:1;
	volatile uint32_t BRE:1;
	uint32_t reserved1:4;
	volatile uint32_t VOSRDY:1;
	uint32_t reserved2:17;

}PWR_CSR_t;

//Whole PWR registers.
typedef struct
{

	volatile PWR_CR_t CR;
	volatile PWR_CSR_t CSR;


} PWR_RegDef_t;


/********************************************************************************************************/

/*
 * Flash control registers (STM32F405xx/07xx and STM32F415xx/17xx)
 */

//TODO: Implement flash control registers for (STM32F42xxx and STM32F43xxx)

//Flash access control register (FLASH_ACR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t LATENCY:3;
	uint32_t reserved0:5;
	volatile uint32_t PRFTEN:1;
	volatile uint32_t ICEN:1;
	volatile uint32_t DCEN:1;
	volatile uint32_t ICRST:1;
	volatile uint32_t DCRST:1;
	uint32_t reserved1:19;

}FLASH_ACR_t;


//Flash status register (FLASH_SR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t EOP:1;
	volatile uint32_t OPERR:1;
	uint32_t reserved0:2;
	volatile uint32_t WRPERR:1;
	volatile uint32_t PGAERR:1;
	volatile uint32_t PGPERR:1;
	volatile uint32_t PGSERR:1;
	uint32_t reserved1:8;
	volatile uint32_t BUSY:1;
	uint32_t reserved2:15;

}FLASH_SR_t;

//Flash control register (FLASH_CR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t PG:1;
	volatile uint32_t SER:1;
	volatile uint32_t MER:1;
	uint32_t reserved0:1;
	volatile uint32_t PSIZE:2;
	uint32_t reserved1:6;
	volatile uint32_t STRT:1;
	uint32_t reserved2:7;
	volatile uint32_t EOPIE:1;
	volatile uint32_t ERRIE:1;
	uint32_t reserved3:5;
	volatile uint32_t LOCK:1;

}FLASH_CR_t;

//Flash option control register (FLASH_OPTCR) for STM32F405xx/07xx and STM32F415xx/17xx
typedef struct
{

	volatile uint32_t OPTLOCK:1;
	volatile uint32_t OPTRST:1;
	volatile uint32_t BOR_LEV:2;
	uint32_t reserved0:1;
	volatile uint32_t WDG_SW:1;
	volatile uint32_t nRST_STOP:1;
	volatile uint32_t nRST_STDBY:1;
	volatile uint32_t RDP:8;
	volatile uint32_t nWRP:12;
	uint32_t reserved1:4;

}FLASH_OPTCR_t;


//Whole Flash registers.
typedef struct
{

	volatile FLASH_ACR_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPT_KEYR;
	volatile FLASH_SR_t SR;
	volatile FLASH_CR_t CR;
	volatile FLASH_OPTCR_t OPTCR;

} FLASH_RegDef_t;

/********************************************************************************************************/

/*
 * Flash control registers (STM32F405xx/07xx and STM32F415xx/17xx)
 */
//Whole EXTI registers.
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_typedef;

/********************************************************************************************************/

/*
 * SYSCFG registers (STM32F405xx/07xx and STM32F415xx/17xx)
 */
//Whole Sysconfig registers.

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTI0:4;
	volatile uint32_t EXTI1:4;
	volatile uint32_t EXTI2:4;
	volatile uint32_t EXTI3:4;
	uint32_t reserved0:16;
	volatile uint32_t EXTI4:4;
	volatile uint32_t EXTI5:4;
	volatile uint32_t EXTI6:4;
	volatile uint32_t EXTI7:4;
	uint32_t reserved1:16;
	volatile uint32_t EXTI8:4;
	volatile uint32_t EXTI9:4;
	volatile uint32_t EXTI10:4;
	volatile uint32_t EXTI11:4;
	uint32_t reserved2:16;
	volatile uint32_t EXTI12:4;
	volatile uint32_t EXTI13:4;
	volatile uint32_t EXTI14:4;
	volatile uint32_t EXTI15:4;
	uint32_t reserved3:16;
	volatile uint32_t CMPCR;

}SYSCFG_typedef;

/********************************************************************************************************/

/*
 * Peripheral Pointer Definitions
 */

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)
#define PWR 		((PWR_RegDef_t*)PWR_BASEADDR)
#define FLASH 		((FLASH_RegDef_t*)FLASH_BASEADDR)
#define EXTI_Reg 	((EXTI_typedef*)EXTI_BASEADDR)
#define SYSCFG_Reg 	((SYSCFG_typedef*)SYSCONFIG_BASEADDR)
#define CPACR 		((uint32_t*)CPACR_BASEADDR)
#define SYSTICK 	((SysTick_t*)SYSTICK_BASEADDR)

/**
  * @brief Status structures definition
  */

typedef enum
{
	STATUS_OK       = 0x00U,
	STATUS_ERROR    = 0x01U,
	STATUS_BUSY     = 0x02U,
	STATUS_TIMEOUT  = 0x03U
} Status_TypeDef_e;

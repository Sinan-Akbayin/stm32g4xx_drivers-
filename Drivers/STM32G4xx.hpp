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

//TIM2-7 REGISTER
#define TIM2_BASEADDR					0x40000000U
#define TIM3_BASEADDR					0x40000400U
#define TIM4_BASEADDR					0x40000800U
#define TIM5_BASEADDR					0x40000C00U
#define TIM6_BASEADDR					0x40001000U
#define TIM7_BASEADDR					0x40001400U

//CRS REGISTER
#define CRS_BASEADDR					0x40002000U

//TAMP REGISTER
#define TAMP_BASEADDR					0x40002400U

//RTC BACKUP REGISTER
#define RTCBKP_BASEADDR					0x40002800U

//WATCHDOG REGISTER
#define WWDG_BASEADDR					0x40002C00U
#define IWDG_BASEADDR					0x40003000U

//SPI2,3 and I2S REGISTER
#define SPI2_BASEADDR					0x40003800U
#define SPI3_BASEADDR					0x40003C00U

//USART2,3 and UART4,5 REGISTER
#define USART2_BASEADDR					0x40004400U
#define USART3_BASEADDR					0x40004800U
#define UART4_BASEADDR					0x40004C00U
#define UART5_BASEADDR					0x40005000U

//I2C1-2 REGISTER
#define I2C1_BASEADDR					0x40005400U
#define I2C2_BASEADDR					0x40005800U

//USB Device FS REGISTER
#define USB_DEVICE_FS_BASEADDR			0x40005C00U
#define USB_SRAM_1KBYTE_BASEADDR		0x40006000U

//FDCAN1-2-3 REGISTER
#define FDCAN1_BASEADDR					0x40006400U
#define FDCAN2_BASEADDR					0x40006800U
#define FDCAN3_BASEADDR					0x40006C00U

//PWR REGISTER
#define PWR_BASEADDR					0x40007000U

//I2C3 REGISTER
#define I2C3_BASEADDR					0x40007800U

//LPTIM1 REGISTER
#define LPTIM1_BASEADDR					0x40007C00U

//LPUART1 REGISTER
#define LPUART1_BASEADDR				0x40008000U

//I2C4 REGISTER
#define I2C4_BASEADDR					0x40008400U

//UCPD1 REGISTER
#define UCPD1_BASEADDR					0x4000A000U

//FDCANs MESSAGE REGISTER
#define FDCAN_BASEADDR					0x4000A400U

/*
 *  Base addresses of peripherals that is connected to APB2 Bus
 */

//BASE ADDRESS
#define APB2PERIPH_BASEADDR				0x40010000U

//SYSCFG REGISTER
#define SYSCFG_BASEADDR					0x40010000U

//VREFBUF REGISTER
#define VREFBUF_BASEADDR				0x40010030U

//COMP REGISTER
#define COMP_BASEADDR					0x40010200U

//OPAMP REGISTER
#define	OPAMP_BASEADDR					0x40010300U

//EXTI REGISTER
#define EXTI_BASEADDR					0x40010400U

//TIM1 REGISTER
#define TIM1_BASEADDR					0x40012C00U

//SPI1,4 REGISTER
#define SPI1_BASEADDR					0x40013000U

//TIM8 REGISTER
#define TIM8_BASEADDR					0x40013400U

//USART1 REGISTER
#define USART1_BASEADDR					0x40013800U

//SPI4 REGISTER
#define SPI4_BASEADDR					0x40013C00U

//TIM15,17 REGISTER
#define TIM15_BASEADDR					0x40014000U
#define TIM16_BASEADDR					0x40014400U
#define TIM17_BASEADDR					0x40014800U

//TIM20 REGISTER
#define TIM20_BASEADDR					0x40015000U

//SAI1 REGISTER
#define SAI1_BASEADDR					0x40015400U

//HRTIM REGISTER
#define HRTIM_BASEADDR					0x40016800U

/*
 *  Base addresses of peripherals that is connected to AHB1 Bus
 */

//BASE ADDRESS
#define AHB1PERIPH_BASEADDR				0x40020000U

//DMA1,2 REGISTER
#define DMA1_BASEADDR					0x40020000U
#define DMA2_BASEADDR					0x40020400U

//DMAMUX REGISTER
#define DMAMUX_BASEADDR					0x40020800U

//CORDIC REGISTER
#define	CORDIC_BASEADDR					0x40020C00U

//RCC REGISTER
#define RCC_BASEADDR					0x40021000U

//FMAC REGISTER
#define FMAC_BASEADDR					0x40021400U

//FLASH INTERFACE REGISTER
#define FLASH_INTERFACE_BASEADDR		0x40022000U

//CRC REGISTER
#define CRC_BASEADDR					0x40023000U

/*
 *  Base addresses of peripherals that is connected to AHB2 Bus
 */
//BASE ADDRESS
#define AHB2PERIPH_BASEADDR				0x48000000U

//GPIOA-G REGISTER
#define GPIOA_BASEADDR					(AHB2PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR					(GPIOA_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR					(GPIOB_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR					(GPIOC_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR					(GPIOD_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR					(GPIOE_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR					(GPIOF_BASEADDR + 0x1800U)

//ADC1-ADC2 REGISTER
#define ADC1_ADC2_BASEADDR				0x50000000U

//ADC3-ADC4-ADC5 REGISTER
#define ADC3_ADC4_ADC5_BASEADDR			0x50000400U

//DAC1 REGISTER
#define DAC1_BASEADDR					0x50000800U

//DAC2 REGISTER
#define DAC2_BASEADDR					0x50000C00U

//DAC3 REGISTER
#define DAC3_BASEADDR					0x50001000U

//DAC4 REGISTER
#define DAC4_BASEADDR					0x50001400U

//AES REGISTER
#define AES_BASEADDR					0x50060000U

//RNG REGISTER
#define RNG_BASEADDR					0x50060800U


#define ADC2_BASEADDR					0x40012100U
#define ADC3_BASEADDR					0x40012200U

//BKPSRAM REGISTER
#define BKPSRAM_BASEADDR				0x40024000U


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
 *  Base addresses of peripherals that is connected to No Bus
 */

//BASE ADDRESS
#define NOBUS_BASEADDR				0xA0000000U

//FSMC REGISTER
#define FSMC_BASEADDR				0xA0000400U

//QUADSPI REGISTER
#define QUADSPI_BASEADDR			0xA0001000U

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

/*
*	RCC clock control register (RCC_CR)
*	Address offset: 0x00
*	Reset value: 0x0000 0500
*	HSEBYP is not affected by reset.
*	Access: no wait state, word, half-word and byte access
*/
typedef struct
{
	/*
	*	Reserved, must be kept at reset value.
	*/
	uint32_t reserved : 8;
	/*
	*	HSI16 clock enable
	*	Set and cleared by software.
	*	Cleared by hardware to stop the HSI16 oscillator when entering Stop, Standby or Shutdown
	*	mode.
	*	Set by hardware to force the HSI16 oscillator ON when STOPWUCK=1 or HSIASFS = 1
	*	when leaving Stop modes, or in case of failure of the HSE crystal oscillator.
	*	This bit is set by hardware if the HSI16 is used directly or indirectly as system clock.
	*	0: HSI16 oscillator OFF
	*	1: HSI16 oscillator ON
	*/
	volatile uint32_t HSION : 1;
	/*
	*	HSI16 always enable for peripheral kernels.
	*	Set and cleared by software to force HSI16 ON even in Stop modes. The HSI16 can only
	*	feed USARTs and I2Cs peripherals configured with HSI16 as kernel clock. Keeping the
	*	HSI16 ON in Stop mode allows to avoid slowing down the communication speed because of
	*	the HSI16 startup time. This bit has no effect on HSION value.
	*	0: No effect on HSI16 oscillator.
	*	1: HSI16 oscillator is forced ON even in Stop mode
	*/
	volatile uint32_t HSIKERON : 1;
	/*
	*	HSI16 clock ready flag
	*	Set by hardware to indicate that HSI16 oscillator is stable. This bit is set only when HSI16 is
	*	enabled by software by setting HSION.
	*	0: HSI16 oscillator not ready
	*	1: HSI16 oscillator ready
	*	Note: Once the HSION bit is cleared, HSIRDY goes low after 6 HSI16 clock cycles.
	*/
	volatile uint32_t HSIRDY : 1;
	/*
	*	Reserved, must be kept at reset value.
	*/
	uint32_t reserved0 : 5;
	/*
	*	HSE clock enable
	*	Set and cleared by software.
	*	Cleared by hardware to stop the HSE oscillator when entering Stop, Standby or Shutdown
	*	mode. This bit cannot be reset if the HSE oscillator is used directly or indirectly as the system
	*	clock.
	*	0: HSE oscillator OFF
	*	1: HSE oscillator ON
	*/
	volatile uint32_t HSEON : 1;
	/*
	*	HSE clock ready flag
	*	Set by hardware to indicate that the HSE oscillator is stable.
	*	0: HSE oscillator not ready
	*	1: HSE oscillator ready
	*	Note: Once the HSEON bit is cleared, HSERDY goes low after 6 HSE clock cycles.
	*/
	volatile uint32_t HSERDY : 1;
	/*
	*	HSE crystal oscillator bypass
	*	Set and cleared by software to bypass the oscillator with an external clock. The external
	*	clock must be enabled with the HSEON bit set, to be used by the device. The HSEBYP bit
	*	can be written only if the HSE oscillator is disabled.
	*	0: HSE crystal oscillator not bypassed
	*	1: HSE crystal oscillator bypassed with external clock
	*/
	volatile uint32_t HSEBYP : 1;
	/*
	*	Clock security system enable
	*	Set by software to enable the clock security system. When CSSON is set, the clock detector
	*	is enabled by hardware when the HSE oscillator is ready, and disabled by hardware if a HSE
	*	clock failure is detected. This bit is set only and is cleared by reset.
	*	0: Clock security system OFF (clock detector OFF)
	*	1: Clock security system ON (Clock detector ON if the HSE oscillator is stable, OFF if not).
	*/
	volatile uint32_t CSSON : 1;
	/*
	*	Reserved, must be kept at reset value.
	*/	
	uint32_t reserved1 : 4;
	/*
	*	Main PLL enable
	*	Set and cleared by software to enable the main PLL.
	*	Cleared by hardware when entering Stop, Standby or Shutdown mode. This bit cannot be
	*	reset if the PLL clock is used as the system clock.
	*	0: PLL OFF
	*	1: PLL ON
	*/
	volatile uint32_t PLLON:1;
	/*
	*	Main PLL clock ready flag
	*	Set by hardware to indicate that the main PLL is locked.
	*	0: PLL unlocked
	*	1: PLL locked
	*/
	volatile uint32_t PLLRDY:1;
	/*
	*	Reserved, must be kept at reset value.
	*/	
	uint32_t reserved2:6;
}RCC_CR_t;

/*
*	Internal clock sources calibration register (RCC_ICSCR)
*	Address offset: 0x04
*	Reset value: 0x40XX 00XX
*	where X is factory-programmed.
*	Access: no wait state, word, half-word and byte access
*/
typedef struct 
{
	/*
	*	Bits 15:0 Reserved, must be kept at reset value.
	*/
	volatile uint32_t reserved : 16;
	/*
	*	Bits 23:16 HSICAL[7:0]: HSI16 clock calibration
	*	These bits are initialized at startup with the factory-programmed HSI16 calibration trim value.
	*	When HSITRIM is written, HSICAL is updated with the sum of HSITRIM and the factory trim
	*	value
	*/
	volatile uint32_t HSICAL : 8;
	/*
	*	Bits 30:24 HSITRIM[6:0]: HSI16 clock trimming
	*	These bits provide an additional user-programmable trimming value that is added to the
	*	HSICAL[7:0] bits. It can be programmed to adjust to variations in voltage and temperature
	*	that influence the frequency of the HSI16.
	*	The default value is 64, which, when added to the HSICAL value, trims HSI16 to
	*	16 MHz ± 1 %. 
	*/
	volatile uint32_t HSITRIM : 7;
	/*
	*	Bit 31 Reserved, must be kept at reset value.
	*/
	volatile uint32_t reserved1 : 1;
}RCC_ICSCR_t;

/*
*	Clock configuration register (RCC_CFGR)
*	Address offset: 0x08
*	Reset value: 0x0000 0005
*	Access: 0 ≤ wait state ≤ 2, word, half-word and byte access
*	1 or 2 wait states inserted only if the access occurs during clock source switch.
*	From 0 to 15 wait states inserted if the access occurs when the APB or AHB prescalers
*	values update is on going.
*/
typedef struct
{
	/*
	*	Bits 1:0 SW[1:0]: System clock switch
	*	Set and cleared by software to select system clock source (SYSCLK).
	*	Configured by hardware to force HSI16 oscillator selection when exiting Stop and Standby
	*	modes or in case of failure of the HSE oscillator.
	*	00: Reserved, must be kept at reset value
	*	01: HSI16 selected as system clock
	*	10: HSE selected as system clock
	*	11: PLL selected as system clock
	*/
	volatile uint32_t SW : 2;
	/*
	*	Bits 3:2 SWS[1:0]: System clock switch status
	*	Set and cleared by hardware to indicate which clock source is used as system clock.
	*	00: Reserved, must be kept at reset value
	*	01: HSI16 oscillator used as system clock
	*	10: HSE used as system clock
	*	11: PLL used as system clock
	*/
	volatile uint32_t SWS : 2;
	/*
	*	Bits 7:4 HPRE[3:0]: AHB prescaler
	*	Set and cleared by software to control the division factor of the AHB clock.
	*	Caution: Depending on the device voltage range, the software must set correctly
	*	these bits to ensure that the system frequency does not exceed the
	*	maximum allowed frequency (for more details refer to Section 6.1.5:
	*	Dynamic voltage scaling management). After a write operation to these
	*	bits and before decreasing the voltage range, this register must be read
	*	to be sure that the new value has been taken into account.
	*	0xxx: SYSCLK not divided
	*	1000: SYSCLK divided by 2
	*	1001: SYSCLK divided by 4
	*	1010: SYSCLK divided by 8
	*	1011: SYSCLK divided by 16
	*	1100: SYSCLK divided by 64
	*	1101: SYSCLK divided by 128
	*	1110: SYSCLK divided by 256
	*	1111: SYSCLK divided by 512
	*/
	volatile uint32_t HPRE : 4;
	/*
	*	Bits 10:8 PPRE1[2:0]:APB1 prescaler
	*	Set and cleared by software to control the division factor of the APB1 clock (PCLK1).
	*	0xx: HCLK not divided
	*	100: HCLK divided by 2
	*	101: HCLK divided by 4
	*	110: HCLK divided by 8
	*	111: HCLK divided by 16
	*/
	volatile uint32_t PPRE1 : 3;
	/*
	*	Bits 13:11 PPRE2[2:0]: APB2 prescaler
	*	Set and cleared by software to control the division factor of the APB2 clock (PCLK2).
	*	0xx: HCLK not divided
	*	100: HCLK divided by 2
	*	101: HCLK divided by 4
	*	110: HCLK divided by 8
	*	111: HCLK divided by 16
	*/
	volatile uint32_t PPRE2 : 3;
	/*
	*	Bits 23:14 Reserved, must be kept at reset value
	*/
	uint32_t reserved : 10;
	/*
	*	Bits 27:24 MCOSEL[3:0]: Microcontroller clock output
	*	Set and cleared by software.
	*	0000: MCO output disabled, no clock on MCO
	*	0001: SYSCLK system clock selected
	*	0010: Reserved, must be kept at reset value
	*	0011: HSI16 clock selected
	*	0100: HSE clock selected
	*	0101: Main PLL clock selected
	*	0110: LSI clock selected
	*	0111: LSE clock selected
	*	1000: Internal HSI48 clock selected
	*	Others: Reserved
	*	Note: This clock output may have some truncated cycles at startup or during MCO clock
	*	source switching.
	*/
	volatile uint32_t MCOSEL : 4;
	/*
	*	Bits 30:28 MCOPRE[2:0]: Microcontroller clock output prescaler
	*	These bits are set and cleared by software.
	*	It is highly recommended to change this prescaler before MCO output is enabled.
	*	000: MCO is divided by 1
	*	001: MCO is divided by 2
	*	010: MCO is divided by 4
	*	011: MCO is divided by 8
	*	100: MCO is divided by 16
	*	Others: not allowed
	*/
	volatile uint32_t MCOPRE : 3;
	/*
	*	Bit 31 Reserved, must be kept at reset value.
	*/
	uint32_t reserved0 : 1;
}RCC_CFGR_t;

/*
*	PLL configuration register (RCC_PLLCFGR)
*	Address offset: 0x0C
*	Reset value: 0x0000 1000
*	Access: no wait state, word, half-word and byte access
*	This register is used to configure the PLL clock outputs according to the formulas:
*	• f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
*	• f(PLL_P) = f(VCO clock) / PLLP
*	• f(PLL_Q) = f(VCO clock) / PLLQ
*	• f(PLL_R) = f(VCO clock) / PLLR
*/
typedef struct
{
	/*
	*	Bits 1:0 PLLSRC[1:0]: Main PLL entry clock source
	*	Set and c.leared by software to select PLL clock source. These bits can be written only when
	*	PLL is disabled.
	*	In order to save power, when no PLL is used, the value of PLLSRC should be 00.
	*	00: No clock sent to PLL
	*	01: No clock sent to PLL
	*	10: HSI16 clock selected as PLL clock entry
	*	11: HSE clock selected as PLL clock entry
	*/
	volatile uint32_t PLLSRC : 2;
	/*
	*	Bits 3:2 Reserved, must be kept at reset value.
	*/
	volatile uint32_t reserved : 2;
	/*
	*	Bits 7:4 PLLM[3:0]: Division factor for the main PLL input clock
	*	Set and cleared by software to divide the PLL input clock before the VCO. These bits can be
	*	written only when all PLLs are disabled.
	*	VCO input frequency = PLL input clock frequency / PLLM with 1 ≤ PLLM ≤ 16
	*	0000: PLLM = 1
	*	0001: PLLM = 2
	*	0010: PLLM = 3
	*	0011: PLLM = 4
	*	0100: PLLM = 5
	*	0101: PLLM = 6
	*	0110: PLLM = 7
	*	0111: PLLM = 8
	*	1000: PLLSYSM = 9
	*	...
	*	1111: PLLSYSM= 16
	*	Caution: The software must set these bits correctly to ensure that the VCO input
	*	frequency is within the range defined in the datasheet.
	*/
	volatile uint32_t PLLM : 4;
	/*
	*	Bits 14:8 PLLN[6:0]: Main PLL multiplication factor for VCO
	*	Set and cleared by software to control the multiplication factor of the VCO. These bits can be
	*	written only when the PLL is disabled.
	*	VCO output frequency = VCO input frequency x PLLN with 8 =< PLLN =< 127
	*	0000000: PLLN = 0 wrong configuration
	*	0000001: PLLN = 1 wrong configuration
	*	...
	*	0000111: PLLN = 7 wrong configuration
	*	0001000: PLLN = 8
	*	0001001: PLLN = 9
	*	...
	*	1111111: PLLN = 127
	*	Caution: The software must set correctly these bits to assure that the VCO output
	*	frequency is within the range defined in the datasheet.
	*/
	volatile uint32_t PLLN : 7;
	/*
	*	Bit 15 Reserved, must be kept at reset value.
	*/
	uint32_t reserved0 : 1;
	/*
	*	Bit 16 PLLPEN: Main PLL PLL “P” clock output enable
	*	Set and reset by software to enable the PLL “P” clock output of the PLL.
	*	To save power, when the PLL “P” clock output of the PLL is not used, the value of PLLPEN
	*	should be 0.
	*	0: PLL “P” clock output disabled
	*	1: PLL “P” clock output enabled
	*/
	volatile uint32_t PLLPEN : 1;
	/*
	*	Bit 17 PLLP: Main PLL division factor for PLL “P” clock.
	*	Set and cleared by software to control the frequency of the main PLL output clock PLL “P”
	*	clock. These bits can be written only if PLL is disabled.
	*	When the PLLPDIV[4:0] is set to “00000”PLL “P” output clock frequency = VCO frequency /
	*	PLLP with PLLP =7, or 17
	*	0: PLLP = 7
	*	1: PLLP = 17
	*	Caution: These bits must be set so as not to exceed 170 MHz on this domain.
	*/
	volatile uint32_t PLLP : 1;
	/*
	*	Bits 19:18 Reserved, must be kept at reset value.
	*/
	uint32_t reserved1 : 2;
	/*
	*	Bit 20 PLLQEN: Main PLL “Q” clock output enable
	*	Set and reset by software to enable the PLL “Q” clock output of the PLL.
	*	In order to save power, when the PLL “Q” clock output of the PLL is not used, the value of
	*	PLLQEN should be 0.
	*	0: PLL “Q” clock output disabled
	*	1: PLL “Q” clock output enabled
	*/
	volatile uint32_t PLLQEN : 1;
	/*
	*	Bits 22:21 PLLQ[1:0]: Main PLL division factor for PLL “Q” clock.
	*	Set and cleared by software to control the frequency of the main PLL output clock PLL “Q”
	*	clock. This output can be selected for USB, RNG, SAI (48 MHz clock). These bits can be
	*	written only if PLL is disabled.
	*	PLL “Q” output clock frequency = VCO frequency / PLLQ with PLLQ = 2, 4, 6, or 8
	*	00: PLLQ = 2
	*	01: PLLQ = 4
	*	10: PLLQ = 6
	*	11: PLLQ = 8
	*	Caution: These bits must be set so as not to exceed 170 MHz on this domain
	*/
	volatile uint32_t PLLQ : 2;
	/*
	*	Bit 23 Reserved, must be kept at reset value.
	*/
	volatile uint32_t reserved2 : 1;
	/*
	*	Bit 24 PLLREN: PLL “R” clock output enable
	*	Set and reset by software to enable the PLL “R” clock output of the PLL (used as system
	*	clock).
	*	This bit cannot be written when PLL “R” clock output of the PLL is used as System Clock.
	*	In order to save power, when the PLL “R” clock output of the PLL is not used, the value of
	*	PLLREN should be 0.
	*	0: PLL “R” clock output disabled
	*	1: PLL “R” clock output enabled
	*/
	volatile uint32_t PLLREN : 1;
	/*
	*	Bits 26:25 PLLR[1:0]: Main PLL division factor for PLL “R” clock (system clock)
	*	Set and cleared by software to control the frequency of the main PLL output clock PLLCLK.
	*	This output can be selected as system clock. These bits can be written only if PLL is
	*	disabled.
	*	PLL “R” output clock frequency = VCO frequency / PLLR with PLLR = 2, 4, 6, or 8
	*	00: PLLR = 2
	*	01: PLLR = 4
	*	10: PLLR = 6
	*	11: PLLR = 8
	*	Caution: These bits must be set so as not to exceed 170 MHz on this domain.
	*/
	volatile uint32_t PLLREN : 2;
	/*
	*	Bits 31:27 PLLPDIV[4:0]: Main PLLP division factor
	*	Set and cleared by software to control the PLL “P” frequency. PLL “P” output clock frequency
	*	= VCO frequency / PLLPDIV.
	*	00000: PLL “P” clock is controlled by the bit PLLP
	*	00001: Reserved.
	*	00010: PLL “P” clock = VCO / 2
	*	....
	*	11111: PLL “P” clock = VCO / 31
	*/
	volatile uint32_t PLLPDIV : 5;
}RCC_PLLCFGR_t;

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

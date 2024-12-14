/*
 * STM32F4xx.h
 *
 *  Created on: 7 Jul 2022
 *      Author: efakb
 */
#pragma once
#include <STM32G4xx_IT.hpp>

//GPIOA-K REGISTER
#define GPIO_BASEADDR	0x40020000U
#define GPIO_OFFSET		0x0400U

/********************************************************************************************************/
/*
* Port & PIN Definitions
* First 4 bits defines the GPIO Ports
* Last 4 bits defines the GPIO Pins 
*/

/* PORT - A */
#define PA0 	0x00
#define PA1 	0x01
#define PA2 	0x02
#define PA3 	0x03
#define PA4 	0x04
#define PA5 	0x05
#define PA6 	0x06
#define PA7 	0x07
#define PA8 	0x08
#define PA9 	0x09
#define PA10 	0x0A
#define PA11 	0x0B
#define PA12 	0x0C
#define PA13 	0x0D
#define PA14 	0x0E
#define PA15 	0x0F

/* PORT - B */
#define PB0 	0x10
#define PB1 	0x11
#define PB2 	0x12
#define PB3 	0x13
#define PB4 	0x14
#define PB5 	0x15
#define PB6 	0x16
#define PB7 	0x17
#define PB8 	0x18
#define PB9 	0x19
#define PB10 	0x1A
#define PB11 	0x1B
#define PB12 	0x1C
#define PB13 	0x1D
#define PB14 	0x1E
#define PB15 	0x1F

/* PORT - C */
#define PC0 	0x20
#define PC1 	0x21
#define PC2 	0x22
#define PC3 	0x23
#define PC4 	0x24
#define PC5 	0x25
#define PC6 	0x26
#define PC7 	0x27
#define PC8 	0x28
#define PC9 	0x29
#define PC10 	0x2A
#define PC11 	0x2B
#define PC12 	0x2C
#define PC13 	0x2D
#define PC14 	0x2E
#define PC15 	0x2F

/* PORT - D */
#define PD0 	0x30
#define PD1 	0x31
#define PD2 	0x32
#define PD3 	0x33
#define PD4 	0x34
#define PD5 	0x35
#define PD6 	0x36
#define PD7 	0x37
#define PD8 	0x38
#define PD9 	0x39
#define PD10 	0x3A
#define PD11 	0x3B
#define PD12 	0x3C
#define PD13 	0x3D
#define PD14 	0x3E
#define PD15 	0x3F

/* PORT - E */
#define PE0 	0x40
#define PE1 	0x41
#define PE2 	0x42
#define PE3 	0x43
#define PE4 	0x44
#define PE5 	0x45
#define PE6 	0x46
#define PE7 	0x47
#define PE8 	0x48
#define PE9 	0x49
#define PE10 	0x4A
#define PE11 	0x4B
#define PE12 	0x4C
#define PE13 	0x4D
#define PE14 	0x4E
#define PE15 	0x4F

/* PORT - F */
#define PF0 	0x50
#define PF1 	0x51
#define PF2 	0x52
#define PF3 	0x53
#define PF4 	0x54
#define PF5 	0x55
#define PF6 	0x56
#define PF7 	0x57
#define PF8 	0x58
#define PF9 	0x59
#define PF10 	0x5A
#define PF11 	0x5B
#define PF12 	0x5C
#define PF13 	0x5D
#define PF14 	0x5E
#define PF15 	0x5F

/* PORT - G */
#define PG0 	0x60
#define PG1 	0x61
#define PG2 	0x62
#define PG3 	0x63
#define PG4 	0x64
#define PG5 	0x65
#define PG6 	0x66
#define PG7 	0x67
#define PG8 	0x68
#define PG9 	0x69
#define PG10 	0x6A
#define PG11 	0x6B
#define PG12 	0x6C
#define PG13 	0x6D
#define PG14 	0x6E
#define PG15 	0x6F

/* PORT - H */
#define PH0 	0x70
#define PH1 	0x71
#define PH2 	0x72
#define PH3 	0x73
#define PH4 	0x74
#define PH5 	0x75
#define PH6 	0x76
#define PH7 	0x77
#define PH8 	0x78
#define PH9 	0x79
#define PH10 	0x7A
#define PH11 	0x7B
#define PH12 	0x7C
#define PH13 	0x7D
#define PH14 	0x7E
#define PH15 	0x7F

/* PORT - J */
#define PJ0 	0x90
#define PJ1 	0x91
#define PJ2 	0x92
#define PJ3 	0x93
#define PJ4 	0x94
#define PJ5 	0x95
#define PJ6 	0x96
#define PJ7 	0x97
#define PJ8 	0x98
#define PJ9 	0x99
#define PJ10 	0x9A
#define PJ11 	0x9B
#define PJ12 	0x9C
#define PJ13 	0x9D
#define PJ14 	0x9E
#define PJ15 	0x9F

/* PORT - K */
#define PK0 	0xA0
#define PK1 	0xA1
#define PK2 	0xA2
#define PK3 	0xA3
#define PK4 	0xA4
#define PK5 	0xA5
#define PK6 	0xA6
#define PK7 	0xA7
#define PK8 	0xA8
#define PK9 	0xA9
#define PK10 	0xAA
#define PK11 	0xAB
#define PK12 	0xAC
#define PK13 	0xAD
#define PK14 	0xAE
#define PK15 	0xAF

/********************************************************************************************************/




/********************************************************************************************************/
/*
 * @GPIO_ALTFN
 * GPIO pin possible Alternate Functions
 */

#define GPIO_AF0 	0
#define GPIO_AF1 	1
#define GPIO_AF2 	2
#define GPIO_AF3 	3
#define GPIO_AF4 	4
#define GPIO_AF5 	5
#define GPIO_AF6 	6
#define GPIO_AF7 	7
#define GPIO_AF8 	8
#define GPIO_AF9 	9
#define GPIO_AF10 	10
#define GPIO_AF11 	11
#define GPIO_AF12 	12
#define GPIO_AF13 	13
#define GPIO_AF14 	14
#define GPIO_AF15 	15

#define SPI1_AltFn 	GPIO_AF5
#define SPI2_AltFn 	GPIO_AF5
#define SPI3_AltFn	GPIO_AF6

#define TIM1_PWM_AltFn 	GPIO_AF1
#define TIM2_PWM_AltFn 	GPIO_AF1
#define TIM3_PWM_AltFn 	GPIO_AF2
#define TIM4_PWM_AltFn 	GPIO_AF2
#define TIM5_PWM_AltFn 	GPIO_AF2
#define TIM8_PWM_AltFn 	GPIO_AF3
#define TIM9_PWM_AltFn 	GPIO_AF3
#define TIM10_PWM_AltFn GPIO_AF3
#define TIM11_PWM_AltFn GPIO_AF3
#define TIM12_PWM_AltFn GPIO_AF9
#define TIM13_PWM_AltFn GPIO_AF9
#define TIM14_PWM_AltFn GPIO_AF9

/********************************************************************************************************/
/*
 * General Purpose Input Output (GPIOx) register typedefs.
 */

typedef struct
{

	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint64_t AFR;

}GPIOx_RegDef_t;


/********************************************************************************************************/
class GPIO
{
public:
	/*
	 * @GPIO_PIN_MODES
	 * GPIO pin mode enumeration
	 */
	enum Pin_Mode{INPUT=0, OUTPUT=1, ALTFN=2, ANALOG=3};

	/*
	 * @GPIO_OTYPE
	 * GPIO pin output type enumeration
	 */
	enum Pin_Output_Type{PUSHPULL=0, OPENDRAIN=1};

	/*
	 * @GPIO_SPEED
	 * GPIO pin possible Speed Modes
	 */
	enum Pin_Output_Speed{SPEED_LOW=0, SPEED_MEDIUM=1, SPEED_HIGH=2, SPEED_VERYHIGH=3};

	/*
	 * @GPIO_PULLUPDOWN
	 * GPIO pin Pull Up Pull Down enumeration
	 */
	enum Pin_PUPD {FLOAT=0,PULLUP=1,PULLDOWN=2};
	/*
	 * @GPIO_EXTI_TRIG
	 * External trigger event type
	 */
	enum EXTI_Trigger {  EXTI_Trigger_Rising = 0, EXTI_Trigger_Falling = 1, EXTI_Trigger_Rising_Falling = 2};

private:

	GPIOx_RegDef_t *GPIO_Port;

	uint8_t Port;

	uint8_t Pin;

public:

	Pin_Mode Mode=INPUT;

	Pin_Output_Type Output_Type=PUSHPULL;

	Pin_Output_Speed Output_Speed=SPEED_LOW;

	Pin_PUPD PullUp_PullDown=FLOAT;

	uint8_t Alternative_Function=0;

	EXTI_Trigger EXTI_Trig;

private:

	void ClockEnable();

public:

	//Constructors

	/*
	* @brief Constructor for the GPIO (uninitialized)
	* @return None
	*/
	GPIO();

	/*
	* @brief Constructor for the GPIO
	* @param Pin_Name: GPIO pin name
	*/
	GPIO(uint8_t Pin_Name);

	/*
	* @brief Constructor for the GPIO
	* @param Pin_Name: GPIO pin name
	* @param Pin_Mode: GPIO pin mode [INPUT, OUTPUT, ALTFN, ANALOG]
	*/
	GPIO(uint8_t Pin_Name, Pin_Mode Pin_Mode);

	/*
	* @brief Constructor for the GPIO
	* @param Pin_Name: GPIO pin name
	* @param Pin_Mode: GPIO pin mode [INPUT, OUTPUT, ALTFN, ANALOG]
	* @param Pin_Output_Type: GPIO pin output type [PUSHPULL, OPENDRAIN]
	* @param Pin_Output_Speed: GPIO pin output speed [SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH, SPEED_VERYHIGH]
	* @param Pin_PUPD: GPIO pin out
	*/
	GPIO(uint8_t Pin_Name, Pin_Mode Pin_Mode, Pin_Output_Type Pin_Output_Type, Pin_Output_Speed Pin_Output_Speed, Pin_PUPD Pin_PUPD);

	/*
	* @brief Configures the GPIO peripheral
	* @return None
	*/
	void Init();

	/*
	* @brief Set the GPIO pin alternate function
	* @param AltFn: Alternate function number [0~15]
	* @return None
	*/
	void SetAltFn(uint8_t AltFN);
	
	/*
	* @brief Initialization for the GPIO
	* @param Pin_Name: GPIO pin name 
	* @param Pin_Mode: GPIO pin mode [INPUT, OUTPUT, ALTFN, ANALOG]
	* @param Pin_Output_Type: GPIO pin output type [PUSHPULL, OPENDRAIN]
	* @param Pin_Output_Speed: GPIO pin output speed [SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH, SPEED_VERYHIGH]
	* @param Pin_PUPD: GPIO pin out
	*/
	void Init(uint8_t Pin_Name, Pin_Mode Pin_Mode, Pin_Output_Type Pin_Output_Type, Pin_Output_Speed Pin_Output_Speed, Pin_PUPD Pin_PUPD);
	
	/*
	* @brief Read the GPIO pin state
	* @return GPIO pin state [0 = Pin Status Low, 1 = Pin Status High]
	*/
	uint8_t Read();

	/*
	* @brief Write the GPIO pin state
	* @param value: GPIO pin state [0 = Pin Status Low, 1 = Pin Status High]
	* @return None
	*/
	void Write(uint8_t value);

	/*
	* @brief Switch the GPIO pin state. If the pin state low, this function change the state to High
	* @return None
	*/
	void Toggle();

	/*
	* @brief Attach interrupt to GPIO
	* @param *callback: callback function pointer
	* @param EXTI_Type: interrupt occour type [EXTI_Trigger_Rising, EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling]
	*/
	void AttachExtInterrupt(void (*callback)(void), EXTI_Trigger EXTI_Type=EXTI_Trigger_Rising);

	/*
	* @brief Returns the Port who owns the specific EXTI line
	* @return Port ID
	*/
	uint8_t GetExtInterruptOwner();
	
	/*
	* @brief Enables the interrupt
	* @return None
	*/
	void EnableExtInterrupt();

	/*
	* @brief Disables the interrupt
	* @return None
	*/
	void DisableExtInterrupt();

	/*
	* @brief Sets the interrupt priority
	* @return None
	*/
	void SetExtInterruptPriority(uint8_t Priority);

};

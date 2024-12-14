/*
 * STM32F4xx.h
 *
 *  Created on: 9 Sep 2023
 *      Author: sezgin-akbayin
 */
#pragma once
#include "STM32G4xx_IT.hpp"
#include "STM32G4xx.hpp"
#include "STM32G4xx_GPIO.hpp"

/**********************************************************************************************************************/
/*
 * TIMER registers typedefs.
*/

/*
 * TIMER control register 1.
*/
typedef struct
{
    volatile uint32_t CEN : 1;          //Counter enable
    volatile uint32_t UDIS : 1;         //Update disable
    volatile uint32_t URS : 1;          //Update request source
    volatile uint32_t OPM : 1;          //One pulse mode
    volatile uint32_t DIR : 1;          //Direction
    volatile uint32_t CMS : 2;          //Center-aligned mode selection
    volatile uint32_t ARPE : 1;         //Auto-reload preload enable
    volatile uint32_t CKD : 2;          //Clock division
    volatile uint32_t reserved : 22;    //Reserved
}TIM_CR1_Reg_t;

/*
 * TIMER control register 2.
*/
typedef struct
{
    volatile uint32_t CCPC : 1;         //Capture/compare preloaded control
    volatile uint32_t reserved : 1;     //Reserved
    volatile uint32_t CCUS : 1;         //Capture/compare control update selection
    volatile uint32_t CCDS : 1;         //Capture/compare DMA selection
    volatile uint32_t MMS : 3;          //Master mode selection
    volatile uint32_t TI1S : 1;         //TI1 selection
    volatile uint32_t OIS1 : 1;         //Output Idle state 1 (OC1 output)
    volatile uint32_t OIS1N : 1;        //Output Idle state 1 (OC1N output)
    volatile uint32_t OIS2 : 1;         //Output Idle state 2 (OC2 output)
    volatile uint32_t OIS2N : 1;        //Output Idle state 2 (OC2N output)
    volatile uint32_t OIS3 : 1;         //Output Idle state 3 (OC3 output)
    volatile uint32_t OIS3N : 1;        //Output Idle state 3 (OC3N output)
    volatile uint32_t OIS4 : 1;         //Output Idle state 4 (OC4 output)
    volatile uint32_t reserved1 : 17;   //Reserved
}TIM_CR2_Reg_t;

/*
 * TIMER slave mode control register.
*/
typedef struct
{
    volatile uint32_t SMS : 3;          //Slave mode selection
    volatile uint32_t reserved : 1;     //Reserved
    volatile uint32_t TS : 3;           //Trigger selection
    volatile uint32_t MSM : 1;          //Master/slave mode
    volatile uint32_t ETF : 4;          //External trigger filter
    volatile uint32_t ETPS : 2;         //External trigger prescaler
    volatile uint32_t ECE : 1;          //External clock enable
    volatile uint32_t ETP : 1;          //External trigger polarity
    volatile uint32_t reserved1 : 16;    //Reserved
}TIM_SMCR_Reg_t;

/*
 * TIMER DMA/interrupt enable register.
*/
typedef struct
{
    volatile uint32_t UIE : 1;          //Update interrupt enable 
    volatile uint32_t CC1IE : 1;        //Capture/Compare 1 interrupt enable
    volatile uint32_t CC2IE : 1;        //Capture/Compare 2 interrupt enable
    volatile uint32_t CC3IE : 1;        //Capture/Compare 3 interrupt enable
    volatile uint32_t CC4IE : 1;        //Capture/Compare 4 interrupt enable
    volatile uint32_t COMIE : 1;        //COM interrupt enable
    volatile uint32_t TIE : 1;          //Trigger interrupt enable
    volatile uint32_t BIE : 1;          //Break interrupt enable
    volatile uint32_t UDE : 1;          //Update DMA request enable
    volatile uint32_t CC1DE : 1;        //Capture/Compare 1 DMA request enable
    volatile uint32_t CC2DE : 1;        //Capture/Compare 2 DMA request enable
    volatile uint32_t CC3DE : 1;        //Capture/Compare 3 DMA request enable
    volatile uint32_t CC4DE : 1;        //Capture/Compare 4 DMA request enable
    volatile uint32_t COMDE : 1;        //COM DMA request enable
    volatile uint32_t TDE : 1;          //Trigger DMA request enable
    volatile uint32_t reserved : 17;    //Reserved
}TIM_DIER_Reg_t;

/*
 * TIMER status register.
*/
typedef struct
{
    volatile uint32_t UIF : 1;          //Update interrupt flag
    volatile uint32_t CC1IF : 1;        //Capture/Compare 1 interrupt flag
    volatile uint32_t CC2IF : 1;        //Capture/Compare 2 interrupt flag
    volatile uint32_t CC3IF : 1;        //Capture/Compare 3 interrupt flag
    volatile uint32_t CC4IF : 1;        //Capture/Compare 4 interrupt flag
    volatile uint32_t COMIF : 1;        //COM interrupt flag
    volatile uint32_t TIF : 1;          //Trigger interrupt flag
    volatile uint32_t BIF : 1;          //Break interrupt flag
    volatile uint32_t reserved : 1;     //Reserved
    volatile uint32_t CC1OF : 1;        //Capture/Compare 1 overcapture flag
    volatile uint32_t CC2OF : 1;        //Capture/Compare 2 overcapture flag
    volatile uint32_t CC3OF : 1;        //Capture/Compare 3 overcapture flag
    volatile uint32_t CC4OF : 1;        //Capture/Compare 4 overcapture flag
    volatile uint32_t reserved1 : 19;   //Reserved
}TIM_SR_Reg_t;


/*
 * TIMER event generation register.
*/
typedef struct
{
    volatile uint32_t UG : 1;           //Update generation
    volatile uint32_t CC1G : 1;         //Capture/Compare 1 generation
    volatile uint32_t CC2G : 1;         //Capture/Compare 2 generation
    volatile uint32_t CC3G : 1;         //Capture/Compare 3 generation
    volatile uint32_t CC4G : 1;         //Capture/Compare 4 generation
    volatile uint32_t COMG : 1;         //Capture/Compare control update generation
    volatile uint32_t TG : 1;           //Trigger generation
    volatile uint32_t BG : 1;           //Break generation
    volatile uint32_t reserved : 24;    //Reserved
}TIM_EGR_Reg_t;


/*
 * TIMER capture/compare mode register 1 (Output compare Mode).
*/
typedef struct
{
    volatile uint32_t CC1S : 2;         //Capture/Compare 1 selection
    volatile uint32_t OC1FE : 1;        //Output compare 1 fast enable
    volatile uint32_t OC1PE : 1;        //Output compare 1 preload enable
    volatile uint32_t OC1M : 3;         //Output compare 1 mode
    volatile uint32_t OC1CE : 1;        //Output compare 1 clear enable
    volatile uint32_t CC2S : 2;         //Capture/Compare 2 selection
    volatile uint32_t OC2FE : 1;        //Output compare 2 fast enable
    volatile uint32_t OC2PE : 1;        //Output compare 2 preload enable
    volatile uint32_t OC2M : 3;         //Output compare 2 mode
    volatile uint32_t OC2CE : 1;        //Output compare 2 clear enable
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCMR1_O_Reg_t;

/*
 * TIMER capture/compare mode register 1 (Input capture Mode).
*/
typedef struct
{
    volatile uint32_t CC1S : 2;         //Capture/Compare 1 selection
    volatile uint32_t IC1PSC : 2;       //Input capture 1 prescaler
    volatile uint32_t IC1F : 4;         //Input capture 1 filter
    volatile uint32_t CC2S : 2;         //Capture/Compare 2 selection
    volatile uint32_t IC2PSC : 2;       //Input capture 2 prescaler
    volatile uint32_t IC2F : 4;         //Input capture 2 filter
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCMR1_I_Reg_t;

/*
 * TIMER capture/compare mode register 2 (Output compare Mode).
*/
typedef struct
{
    volatile uint32_t CC3S : 2;         //Capture/Compare 3 selection
    volatile uint32_t OC3FE : 1;        //Output compare 3 fast enable
    volatile uint32_t OC3PE : 1;        //Output compare 3 preload enable
    volatile uint32_t OC3M : 3;         //Output compare 3 mode
    volatile uint32_t OC3CE : 1;        //Output compare 3 clear enable
    volatile uint32_t CC4S : 2;         //Capture/Compare 4 selection
    volatile uint32_t OC4FE : 1;        //Output compare 4 fast enable
    volatile uint32_t OC4PE : 1;        //Output compare 4 preload enable
    volatile uint32_t OC4M : 3;         //Output compare 4 mode
    volatile uint32_t OC4CE : 1;        //Output compare 4 clear enable
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCMR2_O_Reg_t;

/*
 * TIMER capture/compare mode register 2 (Input capture Mode).
*/
typedef struct
{
    volatile uint32_t CC3S : 2;         //Capture/Compare 3 selection
    volatile uint32_t IC3PSC : 2;       //Input capture 3 prescaler
    volatile uint32_t IC3F : 4;         //Input capture 3 filter
    volatile uint32_t CC4S : 2;         //Capture/Compare 4 selection
    volatile uint32_t IC4PSC : 2;       //Input capture 4 prescaler
    volatile uint32_t IC4F : 4;         //Input capture 4 filter
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCMR2_I_Reg_t;

/*
 * TIMER capture/compare enable register.
*/
typedef struct
{
   volatile uint32_t CC1E : 1;          //Capture/Compare 1 output enable
   volatile uint32_t CC1P : 1;          //Capture/Compare 1 output polarity
   volatile uint32_t CC1NE : 1;         //Capture/Compare 1 complementary output enable
   volatile uint32_t CC1NP : 1;         //Capture/Compare 1 complementary output polarity
   volatile uint32_t CC2E : 1;          //Capture/Compare 2 output enable
   volatile uint32_t CC2P : 1;          //Capture/Compare 2 output polarity
   volatile uint32_t CC2NE : 1;         //Capture/Compare 2 complementary output enable
   volatile uint32_t CC2NP : 1;         //Capture/Compare 2 complementary output polarity
   volatile uint32_t CC3E : 1;          //Capture/Compare 3 output enable
   volatile uint32_t CC3P : 1;          //Capture/Compare 3 output polarity
   volatile uint32_t CC3NE : 1;         //Capture/Compare 3 complementary output enable
   volatile uint32_t CC3NP : 1;         //Capture/Compare 3 complementary output polarity
   volatile uint32_t CC4E : 1;          //Capture/Compare 4 output enable
   volatile uint32_t CC4P : 1;          //Capture/Compare 4 output polarity
   volatile uint32_t reserved : 1;      //Reserved
   volatile uint32_t CC4NP : 1;         //Capture/Compare 4 complementary output enable
   volatile uint32_t reserved1 : 16;    //Reserved
}TIM_CCER_Reg_t;

/*
 * TIMER counter register.
*/
typedef struct
{
    volatile uint32_t CNT : 16;         //Counter value
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CNT_Reg_t;

/*
 * TIMER prescaler register.
*/
typedef struct
{
    volatile uint32_t PSC : 16;         //Prescaler value
    volatile uint32_t reserved : 16;    //Reserved
}TIM_PSC_Reg_t;

/*
 * TIMER auto-reload register.
*/
typedef struct
{
    volatile uint32_t ARR : 16;         //Auto-reload value
    volatile uint32_t reserved : 16;    //Reserved
}TIM_ARR_Reg_t;

/*
 * TIMER repetition counter register.
*/
typedef struct
{
    volatile uint32_t REP : 8;          //Repetition counter value
    volatile uint32_t reserved : 24;    //Reserved
}TIM_RCR_Reg_t;

/*
 * TIMER capture/compare register 1.
*/
typedef struct
{
    volatile uint32_t CCR1 : 16;        //Capture/Compare 1 value
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCR1_Reg_t;

/*
 * TIMER capture/compare register 2.
*/
typedef struct
{
    volatile uint32_t CCR2 : 16;        //Capture/Compare 2 value
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCR2_Reg_t;

/*
 * TIMER capture/compare register 3.
*/
typedef struct
{
    volatile uint32_t CCR3 : 16;        //Capture/Compare 3 value
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCR3_Reg_t;

/*
 * TIMER capture/compare register 4.
*/
typedef struct
{
    volatile uint32_t CCR4 : 16;        //Capture/Compare 4 value
    volatile uint32_t reserved : 16;    //Reserved
}TIM_CCR4_Reg_t;

/*
 * TIMER break and dead-time register.
*/
typedef struct
{
    volatile uint32_t DTG : 8;          //Dead-time generator setup
    volatile uint32_t LOCK : 2;         //Lock configuration
    volatile uint32_t OSSI : 1;         //Off-state selection for idle mode
    volatile uint32_t OSSR : 1;         //Off-state selection for run mode
    volatile uint32_t BKE : 1;          //Break enable
    volatile uint32_t BKP : 1;          //Break polarity
    volatile uint32_t AOE : 1;          //Automatic output enable
    volatile uint32_t MOE : 1;          //Main output enable
    volatile uint32_t reserved : 16;    //Reserved
}TIM_BDTR_Reg_t;

/*
 * TIMER DMA control register.
*/
typedef struct
{
    volatile uint32_t DBA : 5;          //DMA base address
    volatile uint32_t reserved : 3;     //Reserved
    volatile uint32_t DBL : 5;          //DMA burst length
    volatile uint32_t reserved1 : 19;   //Reserved
}TIM_DCR_Reg_t;

/*
 * TIMER DMA address for full transfer register.
*/
typedef struct
{
    volatile uint32_t DMAB : 32;//DMA register for burst accesses
}TIM_DMAR_Reg_t;

/*
 * TIMER peripheral typedefs (Output compare mode).
*/
typedef struct
{
    volatile TIM_CR1_Reg_t CR1;         //TIMER control register 1.
    volatile TIM_CR2_Reg_t CR2;         //TIMER control register 2.
    volatile TIM_SMCR_Reg_t SMCR;       //TIMER slave mode control register.
    volatile TIM_DIER_Reg_t DIER;       //TIMER DMA/interrupt enable register.
    volatile TIM_SR_Reg_t SR;           //TIMER status register.
    volatile TIM_EGR_Reg_t EGR;         //TIMER event generation register.
    volatile TIM_CCMR1_O_Reg_t CCMR1_O; //TIMER capture/compare mode register 1 (Output compare Mode).
    volatile TIM_CCMR2_O_Reg_t CCMR2_O; //TIMER capture/compare mode register 2 (Output compare Mode).
    volatile TIM_CCER_Reg_t CCER;       //TIMER capture/compare enable register.
    volatile TIM_CNT_Reg_t CNT;         //TIMER counter register.
    volatile TIM_PSC_Reg_t PSC;         //TIMER prescaler register.
    volatile TIM_ARR_Reg_t ARR;         //TIMER auto-reload register.
    volatile TIM_RCR_Reg_t RCR;         //TIMER repetition counter register.
    volatile TIM_CCR1_Reg_t CCR1;       //TIMER capture/compare register 1.
    volatile TIM_CCR2_Reg_t CCR2;       //TIMER capture/compare register 2.
    volatile TIM_CCR3_Reg_t CCR3;       //TIMER capture/compare register 3.
    volatile TIM_CCR4_Reg_t CCR4;       //TIMER capture/compare register 4.
    volatile TIM_BDTR_Reg_t BDTR;       //TIMER break and dead-time register.
    volatile TIM_DCR_Reg_t DCR;         //TIMER DMA control register.
    volatile TIM_DMAR_Reg_t DMAR;       //TIMER DMA address for full transfer register.
} TIMER_OC_TypeDef;

/*
 * TIMER peripheral typedefs (Input Capture Mode).
*/
typedef struct
{
    volatile TIM_CR1_Reg_t CR1;         //TIMER control register 1.
    volatile TIM_CR2_Reg_t CR2;         //TIMER control register 2.
    volatile TIM_SMCR_Reg_t SMCR;       //TIMER slave mode control register.
    volatile TIM_DIER_Reg_t DIER;       //TIMER DMA/interrupt enable register.
    volatile TIM_SR_Reg_t SR;           //TIMER status register.
    volatile TIM_EGR_Reg_t EGR;         //TIMER event generation register.
    volatile TIM_CCMR1_I_Reg_t CCMR1_O; //TIMER capture/compare mode register 1 (Input Capture Mode).
    volatile TIM_CCMR2_I_Reg_t CCMR2_O; //TIMER capture/compare mode register 2 (Input Capture Mode).
    volatile TIM_CCER_Reg_t CCER;       //TIMER capture/compare enable register.
    volatile TIM_CNT_Reg_t CNT;         //TIMER counter register.
    volatile TIM_PSC_Reg_t PSC;         //TIMER prescaler register.
    volatile TIM_ARR_Reg_t ARR;         //TIMER auto-reload register.
    volatile TIM_RCR_Reg_t RCR;         //TIMER repetition counter register.
    volatile TIM_CCR1_Reg_t CCR1;       //TIMER capture/compare register 1.
    volatile TIM_CCR2_Reg_t CCR2;       //TIMER capture/compare register 2.
    volatile TIM_CCR3_Reg_t CCR3;       //TIMER capture/compare register 3.
    volatile TIM_CCR4_Reg_t CCR4;       //TIMER capture/compare register 4.
    volatile TIM_BDTR_Reg_t BDTR;       //TIMER break and dead-time register.
    volatile TIM_DCR_Reg_t DCR;         //TIMER DMA control register.
    volatile TIM_DMAR_Reg_t DMAR;       //TIMER DMA address for full transfer register.
} TIMER_IC_TypeDef;

enum TIMERBase
{
    TIM1 = TIM1_BASEADDR, TIM2 = TIM2_BASEADDR, TIM3 = TIM3_BASEADDR, TIM4 = TIM4_BASEADDR, TIM5 = TIM5_BASEADDR,
    TIM6 = TIM6_BASEADDR, TIM7 = TIM7_BASEADDR, TIM8 = TIM8_BASEADDR, TIM9 = TIM9_BASEADDR, TIM10 = TIM10_BASEADDR,
    TIM11 = TIM11_BASEADDR, TIM12 = TIM12_BASEADDR, TIM13 = TIM13_BASEADDR, TIM14 = TIM14_BASEADDR
};

/*
 * TIMER enums.
*/
typedef enum
{
  TIMER_RESET     = 0x00,    	//Peripheral not enabled
  TIMER_READY     = 0x01,    	//Peripheral Initialized and ready for use
  TIMER_ERROR     = 0x02,       //TIMER error state
  TIMER_BUSY      = 0x03    	//an internal process is ongoing
}TIMER_State_e;

/*
 * PWM Class.
*/
class PWM
{

public:

    enum PWMPolarity
    {
         POLARITY_HIGH = 0, POLARITY_LOW = 1
    };
    /*Constructor*/
    PWM(TIMERBase tim_X,
        uint32_t frequency,
        uint8_t GPIO_OC_X, 
        PWMPolarity OC_polarity = POLARITY_HIGH,
        uint8_t GPIO_OCN_X = 0xFF,
        PWMPolarity OCN_polarity = POLARITY_HIGH,
        uint8_t dead_time = 0x00
        );

    void setDuty(uint8_t duty_cycle);

    void PWMStart();

    void PWMNStart();

    void PWMStop();

    void PWMNStop();

private:
    TIMER_OC_TypeDef *TIMER_Base;
    TIMER_State_e state;
};

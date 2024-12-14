/**
  ******************************************************************************
  * @file    STM32F4xx_ADC.hpp
  * @author  Sezgin - Eren 
  * 			      Akbayin
  * @brief   ADC hardware abstraction layer implementation
  *			 Analog to Digital Converter (ADC) peripheral:
  *           - Initialization and reinitialization functions
  *           - IO operation functions
  *           - Peripheral Control functions
  *           - Peripheral State functions
  *
  ******************************************************************************
*/
#pragma once
#include "STM32G4xx_IT.hpp"
#include "STM32G4xx.hpp"
/* Includes ***********************************************************************************************************/

/**********************************************************************************************************************/
/**********************************************************************************************************************/


/*
 * ADC status register (ADC_SR).
*/
typedef struct
{
  uint32_t AWD:1;         //Analog watchdog flag
  uint32_t EOC:1;         //Regular channel end of conversion
  uint32_t JEOC:1;        //Injected channel end of conversion
  uint32_t JSTRT:1;       //Injected channel start flag
  uint32_t STRT:1;        //Regular channel start flag
  uint32_t OVR:1;         //Overrun
  uint32_t Reserved:26;   //Reserved
}ADC_SR_Reg_t;

/*
* ADC control register 1 (ADC_CR1)
*/
typedef struct 
{
  uint32_t AWDCH:5;       //Analog watchdog channel select bits
  uint32_t EOCIE:1;       //Interrupt enable for end of conversion
  uint32_t AWDIE:1;       //Analog watchdog interrupt enable
  uint32_t JEOCIE:1;      //Interrupt enable for injected channels
  uint32_t SCAN:1;        //Scan mode
  uint32_t AWDSGL:1;      //Enable the watchdog on a single channel in scan mode
  uint32_t JAUTO:1;       //Automatic injected group conversion
  uint32_t DISCEN:1;      //Discontinuous mode on regular channels
  uint32_t JDISCEN:1;     //Discontinuous mode on injected channels
  uint32_t DISCNUM:3;     //Discontinuous mode channel count
  uint32_t Reserved:6;    //Reserved
  uint32_t JAWDEN:1;      //Analog watchdog enable on injected channels
  uint32_t AWDEN:1;       //Analog watchdog enable on regular channels
  uint32_t RES:2;         //Resolution
  uint32_t OVRIE:1;       //Overrun interrupt enable
  uint32_t Reserved1:5;   //Reserved
}ADC_CR1_Reg_t;

/*
* ADC control register 2 (ADC_CR2)
*/
typedef struct
{
  uint32_t ADON:1;        //A/D converter ON/OFF
  uint32_t CONT:1;        //Continuous conversion
  uint32_t Reserved:6;    //Reserved
  uint32_t DMA:1;         //Direct memory access mode (for single ADC mode)
  uint32_t DDS:1;         //DMA disable selection (for single ADC mode)
  uint32_t EOCS:1;        //End of conversion selection
  uint32_t ALIGN:1;       //Data alignment
  uint32_t Reserved:4;    //Reserved
  uint32_t JEXTSEL:4;     //External event select for injected group
  uint32_t JEXTEN:2;      //External trigger enable for injected channels
  uint32_t JSWSTART:1;    //Start conversion of injected channels
  uint32_t Reserved1:1;   //Reserved
  uint32_t EXTSEL:4;      //External event select for regular group
  uint32_t EXTEN:2;       //External trigger enable for regular channels
  uint32_t SWSTART:1;     //Start conversion of regular channels
  uint32_t Reserved2:1;   //Reserved
}ADC_CR2_Reg_t;

/*
 *  ADC sample time register 1 (ADC_SMPR1)
*/
typedef struct
{
  uint32_t SMP10:3;       //Channel 10 sampling time selection
  uint32_t SMP11:3;       //Channel 11 sampling time selection
  uint32_t SMP12:3;       //Channel 12 sampling time selection
  uint32_t SMP13:3;       //Channel 13 sampling time selection
  uint32_t SMP14:3;       //Channel 14 sampling time selection
  uint32_t SMP15:3;       //Channel 15 sampling time selection
  uint32_t SMP16:3;       //Channel 16 sampling time selection
  uint32_t SMP17:3;       //Channel 17 sampling time selection
  uint32_t SMP18:3;       //Channel 18 sampling time selection
  uint32_t Reserved:5;    //Reserved
}ADC_SMPR1_Reg_t;

/*
 * ADC sample time register 2 (ADC_SMPR2) 
*/
typedef struct 
{
  uint32_t SMP0:3;        //Channel 0 sampling time selection
  uint32_t SMP1:3;        //Channel 1 sampling time selection
  uint32_t SMP2:3;        //Channel 2 sampling time selection
  uint32_t SMP3:3;        //Channel 3 sampling time selection
  uint32_t SMP4:3;        //Channel 4 sampling time selection
  uint32_t SMP5:3;        //Channel 5 sampling time selection
  uint32_t SMP6:3;        //Channel 6 sampling time selection
  uint32_t SMP7:3;        //Channel 7 sampling time selection
  uint32_t SMP8:3;        //Channel 8 sampling time selection
  uint32_t SMP9:3;        //Channel 9 sampling time selection
  uint32_t Reserved:2;    //Reserved
}ADC_SMPR2_Reg_t;

/*
 * ADC injected channel data offset register x (ADC_JOFRx) (x=1~4)
*/
typedef struct
{
  uint32_t JOFFSET:12;    //Data offset for injected channel
  uint32_t Reserved:20;   //Reserved
}ADC_JOFRx_Reg_t;

/*
 * ADC watchdog higher threshold register (ADC_HTR) 
*/
typedef struct
{
  uint32_t HT:12;         //Analog watchdog higher threshold
  uint32_t Reserved:20;   //Reserved
}ADC_HTR_Reg_t;

/*
 * ADC watchdog lower threshold register (ADC_LTR) 
*/
typedef struct
{
  uint32_t LT:12;         //Analog watchdog lower threshold
  uint32_t Reserved:20;   //Reserved
}ADC_LTR_Reg_t;

/*
 * ADC regular sequence register 1 (ADC_SQR1) 
*/
typedef struct
{
  uint32_t SQ13:5;        //13th conversion in regular sequence
  uint32_t SQ14:5;        //14th conversion in regular sequence
  uint32_t SQ15:5;        //15th conversion in regular sequence
  uint32_t SQ16:5;        //16th conversion in regular sequence
  uint32_t L:4;           //Regular channel sequence length
  uint32_t Reserved:8;    //Reserved
}ADC_SQR1_Reg_t;

/*
 * ADC regular sequence register 2 (ADC_SQR2) 
*/
typedef struct
{
  uint32_t SQ7:5;         //7th conversion in regular sequence
  uint32_t SQ8:5;         //8th conversion in regular sequence
  uint32_t SQ9:5;         //9th conversion in regular sequence
  uint32_t SQ10:5;        //10th conversion in regular sequence
  uint32_t SQ11:5;        //11th conversion in regular sequence
  uint32_t SQ12:5;        //12th conversion in regular sequence
  uint32_t Reserved:2;    //Reserved
}ADC_SQR2_Reg_t;

/*
 * ADC regular sequence register 3 (ADC_SQR3) 
*/
typedef struct
{
  uint32_t SQ1:5;         //1st conversion in regular sequence
  uint32_t SQ2:5;         //2nd conversion in regular sequence
  uint32_t SQ3:5;         //3rd conversion in regular sequence
  uint32_t SQ4:5;         //4th conversion in regular sequence
  uint32_t SQ5:5;         //5th conversion in regular sequence
  uint32_t SQ6:5;         //6th conversion in regular sequence
  uint32_t Reserved:2;    //Reserved
}ADC_SQR3_Reg_t;

/*
 * ADC injected sequence register (ADC_JSQR)
*/
typedef struct
{
  uint32_t JSQ1:5;        //1st conversion in injected sequence
  uint32_t JSQ2:5;        //2nd conversion in injected sequence
  uint32_t JSQ3:5;        //3rd conversion in injected sequence
  uint32_t JSQ4:5;        //4th conversion in injected sequence
  uint32_t JL:2;          //Injected sequence length
  uint32_t Reserved:10;   //Reserved
}ADC_JSQR_Reg_t;

/*
 * ADC injected data register x (ADC_JDRx) (x= 1~4)
*/
typedef struct
{
  uint32_t JDATA:16;      //Injected data
  uint32_t Reserved;      //Reserved
}ADC_JDRx_Reg_t;

/*
 * ADC regular data register (ADC_DR) 
*/
typedef struct
{
  uint32_t DATA:16;       //Regular data
  uint32_t Reserved:16;   //Reserved
}ADC_DR_Reg_t;

/*
 * ADC Common status register (ADC_CSR)
*/
typedef struct
{
  uint32_t AWD1:1;        //Analog watchdog flag of ADC1
  uint32_t EOC1:1;        //End of conversion of ADC1
  uint32_t JEOC1:1;       //Injected channel end of conversion of ADC1
  uint32_t JSTRT1:1;      //Injected channel start flag of ADC1
  uint32_t STRT1:1;       //Regular channel start flag of ADC1
  uint32_t OVR1:1;        //Overrun flag of ADC1
  uint32_t Reserved:2;    //Reserved
  uint32_t AWD2:1;        //Analog watchdog flag of ADC2
  uint32_t EOC2:1;        //End of conversion of ADC2
  uint32_t JEOC2:1;       //Injected channel end of conversion of ADC2
  uint32_t JSTRT2:1;      //Injected channel start flag of ADC2
  uint32_t STRT2:1;       //Regular channel start flag of ADC2
  uint32_t OVR2:1;        //Overrun flag of ADC2
  uint32_t Reserved1:2;   //Reserved
  uint32_t AWD3:1;        //Analog watchdog flag of ADC3
  uint32_t EOC3:1;        //End of conversion of ADC3
  uint32_t JEOC3:1;       //Injected channel end of conversion of ADC3
  uint32_t JSTRT3:1;      //Injected channel start flag of ADC3
  uint32_t STRT3:1;       //Regular channel start flag of ADC3
  uint32_t OVR3:1;        //Overrun flag of ADC3
  uint32_t Reserved2:10;  //Reserved
}ADC_CSR_Reg_t;

/*
 * ADC common control register (ADC_CCR)
*/
typedef struct
{
  uint32_t MULTI:5;       //Multi ADC mode selection
  uint32_t Reserved:3;    //Reserved
  uint32_t DELAY:4;       //Delay between 2 sampling phases
  uint32_t Reserved1:1;   //Reserved
  uint32_t DDS:1;         //DMA disable selection (for multi-ADC mode)
  uint32_t DMA:2;         //DMA memory access mode for multi-ADC mode
  uint32_t ADCPRE:2;      //ADC prescaler
  uint32_t Reserved2:4;   //Reserved
  uint32_t VBATE:1;       //V_BAT enable
  uint32_t TSVREFE:1;     //Temperature sensor and V_REFINT enable
  uint32_t Reserved3:8;   //Reserved
}ADC_CCR_Reg_t;

/*
 * ADC common regular data register for dual and triple modes (ADC_CDR) 
*/
typedef struct
{
  uint32_t DATA1:16;      //1st data item of a pair of regular conversions
  uint32_t DATA2:16;      //2nd data item of a pair of regular conversions
}ADC_CDR_Reg_t;

/**
 * ADC register map
 * 
    Offset - Register
    0x000 - 0x04C ADC1
    0x050 - 0x0FC Reserved
    0x100 - 0x14C ADC2
    0x118 - 0x1FC Reserved
    0x200 - 0x24C ADC3
    0x250 - 0x2FC Reserved
    0x300 - 0x308 Common registers
*/

/*
 * ADC registers typedefs.
*/
typedef struct
{

}ADC_Typedef_t;
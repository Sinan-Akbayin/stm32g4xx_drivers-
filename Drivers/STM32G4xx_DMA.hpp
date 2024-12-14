/**
  *********************************************************************************************************************
  * @file    STM32F4xx_DMA.hpp
  * @author  Sezgin - Eren 
  * 			Akbayin
  * @brief   DMA hardware abstraction layer implementation
  *			 Direct Memory Access(DMA) peripheral:
  *           - Initialization and reinitialization functions
  *           - Peripheral Control functions
  *           - Peripheral State functions
  *
  *********************************************************************************************************************
*/
#pragma once
#include "STM32G4xx_IT.hpp"
#include "STM32G4xx.hpp"

/* Includes ***********************************************************************************************************/

/**********************************************************************************************************************/
/**********************************************************************************************************************/


/*
 * DMA low interrupt status register (DMA_LISR).
*/
typedef struct
{
    uint32_t FEIF0:1;       //Stream 0 FIFO error interrupt flag 
    uint32_t Reserved:1;    //Reserved 
    uint32_t DMEIF0:1;      //Stream 0 direct mode error interrupt flag 
    uint32_t TEIF0:1;       //Stream 0 transfer error interrupt flag 
    uint32_t HTIF0:1;       //Stream 0 half transfer interrupt flag 
    uint32_t TCIF0:1;       //Stream 0 transfer complete interrupt flag
    uint32_t FEIF1:1;       //Stream 1 FIFO error interrupt flag 
    uint32_t Reserved1:1;   //Reserved
    uint32_t DMEIF1:1;      //Stream 1 direct mode error interrupt flag
    uint32_t TEIF1:1;       //Stream 1 transfer error interrupt flag
    uint32_t HTIF1:1;       //Stream 1 half transfer interrupt flag
    uint32_t TCIF1:1;       //Stream 1 transfer complete interrupt flag
    uint32_t Reserved2:4;   //Reserved
    uint32_t FEIF2:1;       //Stream 2 FIFO error interrupt flag 
    uint32_t Reserved3:1;   //Reserved
    uint32_t DMEIF2:1;      //Stream 2 direct mode error interrupt flag
    uint32_t TEIF2:1;       //Stream 2 transfer error interrupt flag
    uint32_t HTIF2:1;       //Stream 2 half transfer interrupt flag
    uint32_t TCIF2:1;       //Stream 2 transfer complete interrupt flag
    uint32_t FEIF3:1;       //Stream 3 FIFO error interrupt flag 
    uint32_t Reserved4:1;   //Reserved
    uint32_t DMEIF3:1;      //Stream 3 direct mode error interrupt flag
    uint32_t TEIF3:1;       //Stream 3 transfer error interrupt flag
    uint32_t HTIF3:1;       //Stream 3 half transfer interrupt flag
    uint32_t TCIF3:1;       //Stream 3 transfer complete interrupt flag
    uint32_t Reserved5:4;   //Reserved
}DMA_LISR_Reg_t;

/*
 * DMA high interrupt status register (DMA_HISR).
*/
typedef struct
{
    uint32_t FEIF4:1;       //Stream 4 FIFO error interrupt flag 
    uint32_t Reserved:1;    //Reserved 
    uint32_t DMEIF4:1;      //Stream 4 direct mode error interrupt flag 
    uint32_t TEIF4:1;       //Stream 4 transfer error interrupt flag 
    uint32_t HTIF4:1;       //Stream 4 half transfer interrupt flag 
    uint32_t TCIF4:1;       //Stream 4 transfer complete interrupt flag
    uint32_t FEIF5:1;       //Stream 5 FIFO error interrupt flag 
    uint32_t Reserved1:1;   //Reserved
    uint32_t DMEIF5:1;      //Stream 5 direct mode error interrupt flag
    uint32_t TEIF5:1;       //Stream 5 transfer error interrupt flag
    uint32_t HTIF5:1;       //Stream 5 half transfer interrupt flag
    uint32_t TCIF5:1;       //Stream 5 transfer complete interrupt flag
    uint32_t Reserved2:4;   //Reserved
    uint32_t FEIF6:1;       //Stream 6 FIFO error interrupt flag 
    uint32_t Reserved3:1;   //Reserved
    uint32_t DMEIF6:1;      //Stream 6 direct mode error interrupt flag
    uint32_t TEIF6:1;       //Stream 6 transfer error interrupt flag
    uint32_t HTIF6:1;       //Stream 6 half transfer interrupt flag
    uint32_t TCIF6:1;       //Stream 6 transfer complete interrupt flag
    uint32_t FEIF7:1;       //Stream 7 FIFO error interrupt flag 
    uint32_t Reserved4:1;   //Reserved
    uint32_t DMEIF7:1;      //Stream 7 direct mode error interrupt flag
    uint32_t TEIF7:1;       //Stream 7 transfer error interrupt flag
    uint32_t HTIF7:1;       //Stream 7 half transfer interrupt flag
    uint32_t TCIF7:1;       //Stream 7 transfer complete interrupt flag
    uint32_t Reserved5:4;   //Reserved
}DMA_HISR_Reg_t;

/*
 * DMA low interrupt flag clear register (DMA_LIFCR).
*/
typedef struct
{
    uint32_t CFEIF0:1;      //Stream 0 clear FIFO error interrupt flag
    uint32_t Reserved:1;    //Reserved
    uint32_t CDMEIF0:1;     //Stream 0 clear direct mode error interrupt flag
    uint32_t CTEIF0:1;      //Stream 0 clear transfer error interrupt flag
    uint32_t CHTIF0:1;      //Stream 0 clear half transfer interrupt flag
    uint32_t CTCIF0:1;      //Stream 0 clear transfer complete interrupt flag
    uint32_t CFEIF1:1;      //Stream 1 clear FIFO error interrupt flag
    uint32_t Reserved1:1;   //Reserved
    uint32_t CDMEIF1:1;     //Stream 1 clear direct mode error interrupt flag
    uint32_t CTEIF1:1;      //Stream 1 clear transfer error interrupt flag
    uint32_t CHTIF1:1;      //Stream 1 clear half transfer interrupt flag
    uint32_t CTCIF1:1;      //Stream 1 clear transfer complete interrupt flag
    uint32_t Reserved2:4;   //Reserved
    uint32_t CFEIF2:1;      //Stream 2 clear FIFO error interrupt flag
    uint32_t Reserved3:1;   //Reserved
    uint32_t CDMEIF2:1;     //Stream 2 clear direct mode error interrupt flag
    uint32_t CTEIF2:1;      //Stream 2 clear transfer error interrupt flag
    uint32_t CHTIF2:1;      //Stream 2 clear half transfer interrupt flag
    uint32_t CTCIF2:1;      //Stream 2 clear transfer complete interrupt flag
    uint32_t CFEIF3:1;      //Stream 3 clear FIFO error interrupt flag
    uint32_t Reserved4:1;   //Reserved
    uint32_t CDMEIF3:1;     //Stream 3 clear direct mode error interrupt flag
    uint32_t CTEIF3:1;      //Stream 3 clear transfer error interrupt flag
    uint32_t CHTIF3:1;      //Stream 3 clear half transfer interrupt flag
    uint32_t CTCIF3:1;      //Stream 3 clear transfer complete interrupt flag
    uint32_t Reserved5:4;   //Reserved
}DMA_LIFCR_Reg_t;

/*
 * DMA high interrupt flag clear register (DMA_HIFCR).
*/
typedef struct
{
    uint32_t CFEIF4:1;      //Stream 4 clear FIFO error interrupt flag
    uint32_t Reserved:1;    //Reserved
    uint32_t CDMEIF4:1;     //Stream 4 clear direct mode error interrupt flag
    uint32_t CTEIF4:1;      //Stream 4 clear transfer error interrupt flag
    uint32_t CHTIF4:1;      //Stream 4 clear half transfer interrupt flag
    uint32_t CTCIF4:1;      //Stream 4 clear transfer complete interrupt flag
    uint32_t CFEIF5:1;      //Stream 5 clear FIFO error interrupt flag
    uint32_t Reserved1:1;   //Reserved
    uint32_t CDMEIF5:1;     //Stream 5 clear direct mode error interrupt flag
    uint32_t CTEIF5:1;      //Stream 5 clear transfer error interrupt flag
    uint32_t CHTIF5:1;      //Stream 5 clear half transfer interrupt flag
    uint32_t CTCIF5:1;      //Stream 5 clear transfer complete interrupt flag
    uint32_t Reserved2:4;   //Reserved
    uint32_t CFEIF6:1;      //Stream 6 clear FIFO error interrupt flag
    uint32_t Reserved3:1;   //Reserved
    uint32_t CDMEIF6:1;     //Stream 6 clear direct mode error interrupt flag
    uint32_t CTEIF6:1;      //Stream 6 clear transfer error interrupt flag
    uint32_t CHTIF6:1;      //Stream 6 clear half transfer interrupt flag
    uint32_t CTCIF6:1;      //Stream 6 clear transfer complete interrupt flag
    uint32_t CFEIF7:1;      //Stream 7 clear FIFO error interrupt flag
    uint32_t Reserved4:1;   //Reserved
    uint32_t CDMEIF7:1;     //Stream 7 clear direct mode error interrupt flag
    uint32_t CTEIF7:1;      //Stream 7 clear transfer error interrupt flag
    uint32_t CHTIF7:1;      //Stream 7 clear half transfer interrupt flag
    uint32_t CTCIF7:1;      //Stream 7 clear transfer complete interrupt flag
    uint32_t Reserved5:4;   //Reserved
}DMA_HIFCR_Reg_t;

/*
* DMA stream x configuration register (DMA_SxCR) (x = 0~7)
*/
typedef struct 
{
    uint32_t EN:1;          //Stream enable / flag stream ready when read low
    uint32_t DMEIE:1;       //Direct mode error interrupt enable
    uint32_t TEIE:1;        //Transfer error interrupt enable
    uint32_t HTIE:1;        //Half transfer interrupt enable
    uint32_t TCIE:1;        //Transfer complete interrupt enable
    uint32_t PFCTRL:1;      //Peripheral flow controller
    uint32_t DIR:2;         //Data transfer direction
    uint32_t CIRC:1;        //Circular mode
    uint32_t PINC:1;        //Peripheral increment mode
    uint32_t MINC:1;        //Memory increment mode
    uint32_t PSIZE:2;       //Peripheral data size
    uint32_t MSIZE:2;       //Memory data size
    uint32_t PINCOS:1;      //Peripheral increment offset size
    uint32_t PL:2;          //Priority level
    uint32_t DBM:1;         //Double buffer mode
    uint32_t CT:1;          //Current targer
    uint32_t PBURST:2;      //Peripheral burst transfer configuration
    uint32_t MBURST:2;      //Memory burst transfer configuration
    uint32_t CHSEL:3;       //Channel selection
    uint32_t Reserved:4;    //Reserved
}DMA_SxCR_Reg_t;

/*
* DMA stream x number of data register (DMA_SxNDTR) (x = 0~7)
*/
typedef struct
{
    uint32_t NDT:16;        //Number of data items to transfer
    uint32_t Reserved:16;   //Reserved
}DMA_SxNDTR_Reg_t;

/*
* DMA stream x peripheral address (DMA_SxPAR) (x = 0~7)
*/
typedef struct 
{
    uint32_t PAR;           //Peripheral address
}DMA_SxPAR_Reg_t;

/*
* DMA stream x memory 0 address register (DMA_SxM0AR) (x = 0~7)
*/
typedef struct 
{
    uint32_t M0A;           //Memory 0 address
}DMA_SxM0AR_Reg_t;

/*
* DMA stream x memory 1 address register (DMA_SxM1AR) (x = 0~7)
*/
typedef struct 
{
    uint32_t M1A;           //Memory 1 address
}DMA_SxM1AR_Reg_t;

/*
* DMA stream x FIFO control register (DMA_SxFCR) (x = 0~7)
*/
typedef struct 
{
    uint32_t FTH:2;         //FIFO threshold selection
    uint32_t DMDIS:1;       //Direct mode disable
    uint32_t FS:3;          //FIFO status
    uint32_t Reserved:1;    //Reserved
    uint32_t FEIE:1;        //FIFO error interrupt enable
    uint32_t Reserved1:24;  //Reserved
}DMA_SxFCR_Reg_t;

/*
 * DMA registers typedefs.
*/
typedef struct
{
    DMA_LISR_Reg_t LISR;    //DMA low interrupt status register
    DMA_HISR_Reg_t HISR;    //DMA high interrupt status register
    DMA_LIFCR_Reg_t LIFCR;  //DMA low interrupt flag clear register
    DMA_HIFCR_Reg_t HIFCR;  //DMA high interrupt flag clear register
    DMA_SxCR_Reg_t S0CR;    //DMA stream 0 configuration register
    DMA_SxNDTR_Reg_t S0NDTR;//DMA stream 0 number of data register
    DMA_SxPAR_Reg_t S0PAR;  //DMA stream 0 peripheral address
    DMA_SxM0AR_Reg_t S0M0AR;//DMA stream 0 memory 0 address register
    DMA_SxM1AR_Reg_t S0M1AR;//DMA stream 0 memory 1 address register
    DMA_SxFCR_Reg_t S0FCR;  //DMA stream 0 FIFO control register
    DMA_SxCR_Reg_t S1CR;    //DMA stream 1 configuration register
    DMA_SxNDTR_Reg_t S1NDTR;//DMA stream 1 number of data register
    DMA_SxPAR_Reg_t S1PAR;  //DMA stream 1 peripheral address
    DMA_SxM0AR_Reg_t S1M0AR;//DMA stream 1 memory 0 address register
    DMA_SxM1AR_Reg_t S1M1AR;//DMA stream 1 memory 1 address register
    DMA_SxFCR_Reg_t S1FCR;  //DMA stream 1 FIFO control register
    DMA_SxCR_Reg_t S2CR;    //DMA stream 2 configuration register
    DMA_SxNDTR_Reg_t S2NDTR;//DMA stream 2 number of data register
    DMA_SxPAR_Reg_t S2PAR;  //DMA stream 2 peripheral address
    DMA_SxM0AR_Reg_t S2M0AR;//DMA stream 2 memory 0 address register
    DMA_SxM1AR_Reg_t S2M1AR;//DMA stream 2 memory 1 address register
    DMA_SxFCR_Reg_t S2FCR;  //DMA stream 2 FIFO control register
    DMA_SxCR_Reg_t S3CR;    //DMA stream 3 configuration register
    DMA_SxNDTR_Reg_t S3NDTR;//DMA stream 3 number of data register
    DMA_SxPAR_Reg_t S3PAR;  //DMA stream 3 peripheral address
    DMA_SxM0AR_Reg_t S3M0AR;//DMA stream 3 memory 0 address register
    DMA_SxM1AR_Reg_t S3M1AR;//DMA stream 3 memory 1 address register
    DMA_SxFCR_Reg_t S3FCR;  //DMA stream 3 FIFO control register
    DMA_SxCR_Reg_t S4CR;    //DMA stream 4 configuration register
    DMA_SxNDTR_Reg_t S4NDTR;//DMA stream 4 number of data register
    DMA_SxPAR_Reg_t S4PAR;  //DMA stream 4 peripheral address
    DMA_SxM0AR_Reg_t S4M0AR;//DMA stream 4 memory 0 address register
    DMA_SxM1AR_Reg_t S4M1AR;//DMA stream 4 memory 1 address register
    DMA_SxFCR_Reg_t S4FCR;  //DMA stream 4 FIFO control register
    DMA_SxCR_Reg_t S5CR;    //DMA stream 5 configuration register
    DMA_SxNDTR_Reg_t S5NDTR;//DMA stream 5 number of data register
    DMA_SxPAR_Reg_t S5PAR;  //DMA stream 5 peripheral address
    DMA_SxM0AR_Reg_t S5M0AR;//DMA stream 5 memory 0 address register
    DMA_SxM1AR_Reg_t S5M1AR;//DMA stream 5 memory 1 address register
    DMA_SxFCR_Reg_t S5FCR;  //DMA stream 5 FIFO control register
    DMA_SxCR_Reg_t S6CR;    //DMA stream 6 configuration register
    DMA_SxNDTR_Reg_t S6NDTR;//DMA stream 6 number of data register
    DMA_SxPAR_Reg_t S6PAR;  //DMA stream 6 peripheral address
    DMA_SxM0AR_Reg_t S6M0AR;//DMA stream 6 memory 0 address register
    DMA_SxM1AR_Reg_t S6M1AR;//DMA stream 6 memory 1 address register
    DMA_SxFCR_Reg_t S6FCR;  //DMA stream 6 FIFO control register
    DMA_SxCR_Reg_t S7CR;    //DMA stream 7 configuration register
    DMA_SxNDTR_Reg_t S7NDTR;//DMA stream 7 number of data register
    DMA_SxPAR_Reg_t S7PAR;  //DMA stream 7 peripheral address
    DMA_SxM0AR_Reg_t S7M0AR;//DMA stream 7 memory 0 address register
    DMA_SxM1AR_Reg_t S7M1AR;//DMA stream 7 memory 1 address register
    DMA_SxFCR_Reg_t S7FCR;  //DMA stream 7 FIFO control register
}DMA_Typedef_t;

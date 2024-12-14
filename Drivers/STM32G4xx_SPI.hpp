/**
  *********************************************************************************************************************
  * @file    STM32F4xx_SPI.hpp
  * @author  Sezgin - Eren 
  * 			Akbayin
  * @brief   SPI hardware abstraction layer implementation
  *			 Serial Peripheral Interface (SPI) peripheral:
  *           - Initialization and reinitialization functions
  *           - IO operation functions
  *           - Peripheral Control functions
  *           - Peripheral State functions
  *
  *********************************************************************************************************************
*/
#pragma once

/* Includes ***********************************************************************************************************/
#include "STM32G4xx_GPIO.hpp"
/**********************************************************************************************************************/
/**********************************************************************************************************************/
/*
 * SPI and I2S registers typedefs.
*/

/*
 * SPI control register 1.
*/
typedef struct
{
    volatile uint32_t CPHA:1;       //Clock phase      
    volatile uint32_t CPOL:1;       //Clock polarity
    volatile uint32_t MSTR:1;       //Master selection
    volatile uint32_t BR:3;         //Baud rate control
    volatile uint32_t SPE:1;        //SPI enable
    volatile uint32_t LSBFIRST:1;   //Frame format
    volatile uint32_t SSI:1;        //Internal slave select
    volatile uint32_t SSM:1;        //Software slave management
    volatile uint32_t RXONLY:1;     //Receive only       
    volatile uint32_t DFF:1;        //Data frame format
    volatile uint32_t CRCNEXT:1;    //CRC transfer next
    volatile uint32_t CRCEN:1;      //Hardware CRC calculation enable
    volatile uint32_t BIDIOE:1;     //Output enable in bidirectional mode
    volatile uint32_t BIDMODE:1;    //Bidirectional data mode enable
    volatile uint32_t reserved:16;  //Reserved
} SPI_CR1_Reg_t;

/*
 * SPI control register 2.
*/
typedef struct
{
    volatile uint32_t RXDMAEN:1;    //Rx buffer DMA enable
    volatile uint32_t TXDMAEN:1;    //Tx buffer DMA enable
    volatile uint32_t SSOE:1;       //SS output enable
    volatile uint32_t reserved:1;   //Reserved
    volatile uint32_t FRF:1;        //Frame format
    volatile uint32_t ERRIE:1;      //Error interrupt enable
    volatile uint32_t RXNEIE:1;     //RX buffer not empty interrupt enable
    volatile uint32_t TXEIE:1;      //Tx buffer empty interrupt enable
    volatile uint32_t reserved1:24; //Reserved
} SPI_CR2_Reg_t;

/*
 * SPI status register.
*/
typedef struct
{
    volatile uint32_t RXNE:1;       //Receive buffer not empty
    volatile uint32_t TXE:1;        //Transmit buffer empty
    volatile uint32_t CHSIDE:1;     //Channel side
    volatile uint32_t UDR:1;        //Underrun flag
    volatile uint32_t CRCERR:1;     //CRC error flag
    volatile uint32_t MODF:1;       //Mode fault
    volatile uint32_t OVR:1;        //Overrun flag      
    volatile uint32_t BSY:1;        //Busy flag
    volatile uint32_t FRE:1;        //Frame format error
    volatile uint32_t reserved:23;  //Reserved
} SPI_SR_Reg_t;

/*
 * SPI data register.
*/
typedef struct
{
    volatile uint32_t DR:16;        //Data register
    volatile uint32_t reserved:16;  //Reserved                        
} SPI_DR_Reg_t;

/*
 * SPI CRC polynomial register.
*/
typedef struct
{
    volatile uint32_t CRCPOLY:16;   //CRC polynomial register
    volatile uint32_t reserved:16;  //Reserved             
} SPI_CRCPR_Reg_t;

/*
 * SPI RX CRC register.
*/
typedef struct
{
    volatile uint32_t RXCRC:16;     //Rx CRC register
    volatile uint32_t reserved:16;  //Reserved                     
} SPI_RXCRCR_Reg_t;

/*
 * SPI TX CRC register.
*/
typedef struct
{
    volatile uint32_t TXCRC:16;     //Tx CRC register
    volatile uint32_t reserved:16;  //Reserved                      
} SPI_TXCRCR_Reg_t;

/*
 * SPI_I2S configuration register.
*/
typedef struct
{
    volatile uint32_t CHLEN:1;      //Channel length (number of bits per audio channel)
    volatile uint32_t DATLEN:2;     //Data length to be transferred
    volatile uint32_t CKPOL:1;      //Steady state clock polarity
    volatile uint32_t I2SSTD:2;     //I2S standard selection
    volatile uint32_t reserved:1;   //Reserved
    volatile uint32_t PCMSYNC:1;    //PCM frame synchronization
    volatile uint32_t I2SCFG:2;     //I2S configuration mode
    volatile uint32_t I2SE:1;       //I2S Enable
    volatile uint32_t I2SMOD:1;     //I2S mode selection
    volatile uint32_t reserved1:20; //Reserved                
} SPI_I2SCFGR_Reg_t;

/*
 * SPI_I2S prescaler register.
*/
typedef struct
{
    volatile uint32_t I2SDIV:8;     //I2S Linear prescaler
    volatile uint32_t ODD:1;        //Odd factor for the prescaler
    volatile uint32_t MCKOE:1;      //Master clock output enable
    volatile uint32_t reserved:22;  //Reserved                    
} SPI_I2SPR_Reg_t;

/*
 * SPI and I2S registers typedefs.
*/
typedef struct
{
    volatile SPI_CR1_Reg_t        CR1;        // SPI control register 1  
    volatile SPI_CR2_Reg_t        CR2;        // SPI control register 2 
    volatile SPI_SR_Reg_t         SR;         // SPI status register 
    volatile SPI_DR_Reg_t         DR;         // SPI data register 
    volatile SPI_CRCPR_Reg_t      CRCPR;      // SPI CRC polynomial register 
    volatile SPI_RXCRCR_Reg_t     RXCRCR;     // SPI RX CRC register 
    volatile SPI_TXCRCR_Reg_t     TXCRCR;     // SPI TX CRC register 
    volatile SPI_I2SCFGR_Reg_t    I2SCFGR;    // SPI_I2S configuration register 
    volatile SPI_I2SPR_Reg_t      I2SPR;      // SPI_I2S prescaler register 
} SPI_TypeDef;
/**********************************************************************************************************************/

/**********************************************************************************************************************/
/*
 * SPI and I2S enums.
*/
typedef enum
{
  RESET     = 0x00,    	//Peripheral not enabled
  READY     = 0x01,    	//Peripheral Initialized and ready for use
  ERROR     = 0x02,     //SPI error state
  BUSY      = 0x03    	//an internal process is ongoing
}SPI_State_e;

typedef enum
{
  OK		= 0x00,    //Peripheral not enabled
  TIMEOUT	= 0x01     //Peripheral Initialized and ready for use
}SPI_ERROR_e;
/**********************************************************************************************************************/

/**********************************************************************************************************************/
/*
 * SPI Class.
*/
class SPI
{
public:

    enum SPIBase
    {SPI1 = SPI1_BASEADDR, SPI2 = SPI2_BASEADDR, SPI3 = SPI3_BASEADDR};

    enum Mode
    {SLAVE = 0, MASTER = 1};

    enum Direction
    {UNIDIRECTION = 0, BIDIRECTION_RX_ONLY = 1, BIDIRECTION_TX_ONLY = 2};
    
    enum DataSize
    {EIGHT_BIT = 0, SIXTEEN_BIT =1};

    enum ClockPolarity
    {LOW = 0, HIGH = 1};

    enum ClockPhase
    {FIRST_CLOCK = 0, SECOND_CLOCK = 1};

    enum BaudRate
    {DIV_2 = 0 , DIV_4 = 1, DIV_8 = 2, DIV_16 = 3, DIV_32 = 4, DIV_64 = 5, DIV_128 = 6, DIV_256 = 7};

    enum FirstBit
    {MSB = 0, LSB = 1};

    enum FrameFormat
    {MOTOROLA = 0, TI =1};

    enum EnableDisable
    {DISABLE = 0, ENABLE = 1};

private:

    SPIBase SPIx;

    SPI_TypeDef *SPI_Base;

    SPI_State_e state;

    /*SPI Pins*/

    uint8_t MOSI;		//Master Out Slave In

    uint8_t MISO;		//Master In Slave Out

    uint8_t SCK;		//Serial Clock

    uint8_t CS; 		//Chip Select (Active low, optional)

    /* RX and TX buffer pointers and counters*/

    uint16_t BufferSize;

    uint16_t RXCount;

    uint16_t TXCount;

    uint8_t *pTX_8bit_Buffer;

    uint8_t *pRX_8bit_Buffer;

    uint16_t *pTX_16bit_Buffer;

	uint16_t *pRX_16bit_Buffer;

    /* Callback function pointers */

	void (*TXCompletedCallback)(void) = nullptr;

    void (*RXCompletedCallback)(void) = nullptr;

    void (*TXRXCompletedCallback)(void) = nullptr;

    void (*TXHalfCompletedCallback)(void) = nullptr;

    void (*RXHalfCompletedCallback)(void) = nullptr;

    void (*TXRXHalfCompletedCallback)(void) = nullptr;

    void (*User_callback)(void) = nullptr;

public:

    /**
    * @brief This function enables the SPI peripheral
    * @return None
    */
    void Enable();
    
    /**
    * @brief This function disables the SPI peripheral
    * @return None
    */
	void Disable();
    
    /**
    * @brief This function initializes the SPI peripheral
    * @param [mode_x]    : Specifies the SPI peripheral [SPI1, SPI2, SPI3]
    * @param [direction] : Specifies the SPI direction [UNIDIRECTION, BIDIRECTION_RX_ONLY, BIDIRECTION_TX_ONLY]
    * @param [clk_pol]   : Specifies the clock polarity of the SPI peripheral [LOW, HIGH]
    * @param [clk_phase] : Specifies the clock phase of the SPI peripheral [FIRST_CLOCK, SECOND_CLOCK]
    * @param [baud_cntrl]: Specifies the SPI Baud rate (Peripheral Clock Speed / divide_X ) [DIV_2, DIV_4, DIV_8, DIV_16, DIV_32, DIV_64, DIV_128, DIV_256]
    * @param [fb_sel]    : Specifies the SPI frame format [MSB, LSB]
    * @param [format]    : Specifies the SPI frame format [MOTOROLA, TI]
    * @param [CRC]       : Enable or Disable the SPI CRC calculation
    * @return None
    */
	void Init(Mode mode_x = MASTER,
              Direction direction = UNIDIRECTION,
              ClockPolarity clk_pol = LOW, 
              ClockPhase clk_phase = FIRST_CLOCK,
              BaudRate baud_cntrl = DIV_256, 
              FirstBit fb_sel = MSB, 
              FrameFormat format = MOTOROLA, 
              EnableDisable CRC = DISABLE);
    
    /**
    * @brief Consturctor of SPI class
    * @param [SPIx]      : [SPI1, SPI2, SPI3]
    * @param [MOSI]      : Master Out Slave In Pin
    * @param [MISO]      : Master In Slave Out Pin
    * @param [SCK]       : Serial Clock Pin
    * @param [CS]        : Chip Select Pin
    * @param [mode_x]    : Specifies the SPI peripheral [SPI1, SPI2, SPI3]
    * @param [direction] : Specifies the SPI direction [UNIDIRECTION, BIDIRECTION_RX_ONLY, BIDIRECTION_TX_ONLY]
    * @param [clk_pol]   : Specifies the clock polarity of the SPI peripheral [LOW, HIGH]
    * @param [clk_phase] : Specifies the clock phase of the SPI peripheral [FIRST_CLOCK, SECOND_CLOCK]
    * @param [baud_cntrl]: Specifies the SPI Baud rate (Peripheral Clock Speed / divide_X ) [DIV_2, DIV_4, DIV_8, DIV_16, DIV_32, DIV_64, DIV_128, DIV_256]
    * @param [fb_sel]    : Specifies the SPI frame format [MSB, LSB]
    * @param [format]    : Specifies the SPI frame format [MOTOROLA, TI]
    * @param [CRC]       : Enable or Disable the SPI CRC calculation
    */
    SPI(SPIBase SPIx,
        uint8_t MOSI,
        uint8_t MISO,
        uint8_t SCK,
        uint8_t CS = 0xFF,
        Mode mode_x = MASTER,
        Direction direction = UNIDIRECTION,
        ClockPolarity clk_pol = LOW,
        ClockPhase clk_phase = FIRST_CLOCK,
        BaudRate baud_cntrl = DIV_256,
        FirstBit fb_sel = MSB,
        FrameFormat format = MOTOROLA,
        EnableDisable CRC = DISABLE
        );
    
    /**
    * @brief Deconstructor of SPI class
    */
    ~SPI();

    /**
    * @brief This function handles the SPI peripheral Reset event.
    * @return None
    */
    void Reset();

    /**
    * @brief Get the SPI status
    * @return Status
    */
    SPI_State_e GetState(void);

    /**
    * @brief Transmits and receives 8-bit data from SPI
    * @param [*TxData]    : Pointer of data to be transmitted
    * @param [*RxData]    : Pointer of data to be received
    * @param [Size]       : Size of the data to be transmitted
    * @param [Timeout]    : Timeout in milliseconds
    * @return SPI Status
    */
    Status_TypeDef_e TransmitReceive(uint8_t* TxData, uint8_t* RxData, uint16_t size, uint32_t Timeout);

    /**
    * @brief Transmits and receives 16-bit data from SPI
    * @param [*TxData]    : Pointer of data to be transmitted
    * @param [*RxData]    : Pointer of data to be received
    * @param [Size]       : Size of the data to be transmitted
    * @param [Timeout]    : Timeout in milliseconds
    * @return SPI Status
    */
    Status_TypeDef_e TransmitReceive(uint16_t* TxData, uint16_t* RxData, uint16_t size, uint32_t Timeout);
    
    /**
    * @brief This function set the transmits and receives ineterrupt
    * @param [*TxData]    : Data to be transmitted
    * @param [*RxData]    : Data to be received
    * @param [Size]       : Size of the data to be transmitted
    * @param [*callback]  : Callback function to be called when interrupt occurred
    * @return Status
    */
    Status_TypeDef_e TransmitReceiveIT(uint8_t* TxData, uint8_t* RxData, uint16_t size, void (*callback)(void));
    
    /**
    * @brief This function set the transmits and receives ineterrupt
    * @param [*TxData]    : Data to be transmitted
    * @param [*RxData]    : Data to be received
    * @param [Size]       : Size of the data to be transmitted
    * @param [*callback]  : Callback function to be called when interrupt occurred
    * @return Status
    */
    Status_TypeDef_e TransmitReceiveIT(uint16_t* TxData, uint16_t* RxData, uint16_t size, void (*callback)(void));
    
    /**
     * @brief This function attach the SPI Tx Completed callback function 
     * @param [*callback]  : Void pointer function (This function is called when the TX interrupt is occurred)
     * @return None
    */
    void attachTxCompletedCallback(void (*callback)(void));
    
    /**
     * @brief This function attach the SPI Rx Completed callback function 
     * @param [*callback] : Void pointer function (This function is called when the RXinterrupt is occurred)
     * @return None
     */
    void attachRxCompletedCallback(void (*callback)(void));

    /**
     * @brief This function attach the SPI TxRx Completed callback function
     * @param [*callback] : Void pointer function (This function is called when the interrupt is occurred)
     * @return None
    */
    void attachTxRxCompletedCallback(void (*callback)(void));

    /**
     * @brief This function attach the SPI Tx Half-Completed callback function
     * @param [*callback]  : Void pointer function (This function is called when the TXinterrupt is occurred)
     * @return None
    */
    void attachTxHalfCompletedCallback(void (*callback)(void));

    /**
     * @brief This function attach the SPI Rx Half-Completed callback function
     * @param [*callback] : Void pointer function (This function is called when the RXinterrupt is occurred)
     * @return None
     */
    void attachRxHalfCompletedCallback(void (*callback)(void));

    /**
     * @brief This function attach the SPI TxRx Half-Completed callback function
     * @param [*callback] : Void pointer function (This function is called when the interrupt is occurred)
     * @return None
    */
    void attachTxRxHalfCompletedCallback(void (*callback)(void));

    /**
    * @brief Enable the SPI interrupt
    * @return None
    */
    void EnableInterrupt();
    
    /**
    * @brief Enable the SPI interrupt
    * @return None
    */
    void DisableInterrupt();
    
    /**
    * @brief This function sets the interrupt priority
    * @param [Priority]   : specifies the interrupt priority
    * @return None
    */
    void SetInterruptPriority(uint8_t Priority);

private:

    /**
    * @brief This function ends the Transmit or Receive operations at given time
    * @param [Timeout]   : Timeout in milliseconds
    * @return Status of the operation
    */
    Status_TypeDef_e EndTransmitReceive(uint32_t Timeout);

    /**
    * @brief Interrupt service routine for 8-bit
    * @return None
    */
    void ISR();

};

/* This section required for IRQ Handling procces */
#ifdef __cplusplus
	extern "C" {
#endif

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

void SPI1_IRQHandler();

void SPI2_IRQHandler();

void SPI3_IRQHandler();

#ifdef __cplusplus
}
#endif

/**********************************************************************************************************************/
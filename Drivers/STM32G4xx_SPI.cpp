/**
  ******************************************************************************
  * @file    STM32F4xx_SPI.cpp
  * @author  Sezgin - Eren 
  * 			Akbayin
  * @brief   SPI hardware abstraction layer implementation
  *			 Serial Peripheral Interface (SPI) peripheral:
  *           - Initialization and reinitialization functions
  *           - IO operation functions
  *           - Peripheral Control functions
  *           - Peripheral State functions
  *
  ******************************************************************************
*/

/* Includes *******************************************************************/
#include "STM32G4xx_SPI.hpp"
/******************************************************************************/

/* Private variables **********************************************************/

/* This pointers created for control to interrupt routines */
static SPI* SPI_Class_Pointers[3];
/******************************************************************************/

/* Public functions ***********************************************************/

/**
* @brief This function enables the SPI peripheral
* @return None
*/
void SPI::Enable()
{
	SPI_Base->CR1.SPE = ENABLE;
}

/**
* @brief This function disables the SPI peripheral
* @return None
*/
void SPI::Disable()
{
	SPI_Base->CR1.SPE = DISABLE;
}

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
void SPI::Init(Mode mode_x,
			   Direction direction,
			   ClockPolarity clk_pol,
			   ClockPhase clk_phase,
			   BaudRate baud_cntrl, 
			   FirstBit fb_sel, 
			   FrameFormat format, 
			   EnableDisable CRC)
{
	/* Enable the SPI peripheral clock */
	switch(SPIx)
	{
	case SPI1:
		/* Enable SPI1 peripheral clock */
		RCC->APB2ENR.SPI1EN = 1;

		break;

	case SPI2:
		/* Enable SPI2 peripheral clock */
		RCC->APB1ENR.SPI2EN = 1;

		break;

	case SPI3:
		/* Enable SPI3 peripheral clock */
		RCC->APB1ENR.SPI3EN = 1;

		break;
	}

	/* Disable the SPI peripheral for configuration */
    SPI_Base->CR1.SPE = DISABLE;
	/* Set the SPI bidirection data mode */
    SPI_Base->CR1.BIDMODE = (direction == UNIDIRECTION) ?  UNIDIRECTION : 1;
	/* Set the output enable in bidirection mode */
    SPI_Base->CR1.BIDIOE = (direction > UNIDIRECTION) ? (direction-1) : UNIDIRECTION;
	/* Set the Master/Slave selection */
    SPI_Base->CR1.MSTR = mode_x;
	/* Set the clock polarity */
    SPI_Base->CR1.CPOL = clk_pol;
	/* Set the clock phase */
    SPI_Base->CR1.CPHA = clk_phase;
	/* Set the baudrate */
    SPI_Base->CR1.BR = baud_cntrl;
	/* Set the frame format */
    SPI_Base->CR1.LSBFIRST = fb_sel;
	/* Set the CRC calculation */
    SPI_Base->CR1.CRCEN = CRC;
	/*If Chip Select pin is defined */
    if (CS!=0xFF)
	{
		/* Chip Select output enable */
    	SPI_Base->CR2.SSOE = 1;
		/* Software Chip select disabled. Chip select pin driven by hardware*/
        SPI_Base->CR1.SSM = 0;
	}
    else
    {
		/* Chip Select output disable */
    	SPI_Base->CR2.SSOE = 0;
		/* Software Chip select enable. Chip select pin driven by software*/
    	SPI_Base->CR1.SSM = 1;
		/* Internal chip select disable */
    	SPI_Base->CR1.SSI = 0;
    }
	/* Set the frame format */
    SPI_Base->CR2.FRF = format;
}

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
SPI::SPI(SPIBase SPIx,
		uint8_t MOSI,
		uint8_t MISO,
		uint8_t SCK,
		uint8_t CS,
		Mode mode_x,
		Direction direction,
		ClockPolarity clk_pol,
		ClockPhase clk_phase,
		BaudRate baud_cntrl,
		FirstBit fb_sel,
		FrameFormat format,
		EnableDisable CRC)
		:SPIx(SPIx), MOSI(MOSI), MISO(MISO), SCK(SCK), CS(CS)
{
	/* Get the SPI Base Address */
	SPI_Base = reinterpret_cast<SPI_TypeDef*>(SPIx);
	/* Set the MOSI pin */
	GPIO _MOSI(MOSI, GPIO::ALTFN, GPIO::PUSHPULL, GPIO::SPEED_VERYHIGH, GPIO::FLOAT);
	/* Set the MISO pin */
	GPIO _MISO(MISO, GPIO::ALTFN, GPIO::PUSHPULL, GPIO::SPEED_VERYHIGH, GPIO::FLOAT);
	/* Set the SCK pin */
	GPIO _SCK(SCK, GPIO::ALTFN, GPIO::PUSHPULL, GPIO::SPEED_VERYHIGH, GPIO::FLOAT);
	/* Create the Chip Select Pin */
	GPIO _CS;
	/* If Chip select pin decelerated than let's set the pin */
	if (CS!=0xFF)
	{
		_CS.Init(CS, GPIO::ALTFN, GPIO::PUSHPULL, GPIO::SPEED_VERYHIGH, GPIO::FLOAT);
	}
	/* Set the pins alternate functions for SPI */
	switch(SPIx)
	{
	case SPI1:
		/* Set mosi pin alternate function */
		_MOSI.SetAltFn(SPI1_AltFn);
		/* Set miso pin alternate function */
		_MISO.SetAltFn(SPI1_AltFn);
		/* Set sck pin alternate function */
		_SCK.SetAltFn(SPI1_AltFn);
		/* If Chip select pin decelerated than let's set the alternate function */
		if (CS != 0xFF)
		{
			/* Than let's set the alternate function */
			_CS.SetAltFn(SPI1_AltFn);
		}

		break;

	case SPI2:
		/* Set mosi pin alternate function */
		_MOSI.SetAltFn(SPI2_AltFn);
		/* Set miso pin alternate function */
		_MISO.SetAltFn(SPI2_AltFn);
		/* Set sck pin alternate function */
		_SCK.SetAltFn(SPI2_AltFn);
		/* If Chip select pin decelerated  */
		if (CS != 0xFF)
		{
			/* Than let's set the alternate function */
			_CS.SetAltFn(SPI2_AltFn);
		}

		break;

	case SPI3:
		/* Set mosi pin alternate function */
		_MOSI.SetAltFn(SPI3_AltFn);
		/* Set miso pin alternate function */
		_MISO.SetAltFn(SPI3_AltFn);
		/* Set sck pin alternate function */
		_SCK.SetAltFn(SPI3_AltFn);
		/* If Chip select pin decelerated  */
		if (CS != 0xFF)
		{
			/* Than let's set the alternate function */
			_CS.SetAltFn(SPI3_AltFn);
		}

		break;
	}
	/* Set Peripheral configurations and initialize it */
	Init(mode_x,direction, clk_pol, clk_phase, baud_cntrl, fb_sel, format, CRC);
}

/**
* @brief Deconstructor of SPI class
*/
SPI::~SPI()
{
	switch(SPIx)
	{
	case SPI1:
		/* SPI1 Peripheral Reset */
		RCC->APB2RSTR.SPI1RST = 1;

		RCC->APB2RSTR.SPI1RST = 0;
		/* SPI1 Clock disable */
		RCC->APB2ENR.SPI1EN = 0;

		break;

	case SPI2:
		/* SPI2 Peripheral Reset */
		RCC->APB1RSTR.SPI2RST = 1;

		RCC->APB1RSTR.SPI2RST = 0;
		/* SPI2 Clock disable */
		RCC->APB1ENR.SPI2EN = 0;

		break;

	case SPI3:
		/* SPI3 Peripheral Reset */
		RCC->APB1RSTR.SPI3RST = 1;

		RCC->APB1RSTR.SPI3RST = 0;
		/* SPI3 Clock disable */
		RCC->APB1ENR.SPI3EN = 0;

		break;
	}
}

/**
* @brief This function handles the SPI peripheral reset event.
* @return None
*/
void SPI::Reset()
{
	/* Get the SPI mode */
	Mode mode_x = (Mode) SPI_Base->CR1.MSTR;
	/* Get the SPI direction */
	Direction direction;

	if (SPI_Base->CR1.BIDMODE == 0)
	{

		direction = UNIDIRECTION;

	}
	else
	{
		if (SPI_Base->CR1.BIDIOE == 0)
		{
			direction = BIDIRECTION_RX_ONLY;
		}
		else
		{
			direction = BIDIRECTION_TX_ONLY;
		}

	}
	/* Get the SPI clock polarity */
	ClockPolarity clk_pol = (ClockPolarity) SPI_Base->CR1.CPOL;
	/* Get the SPI clock phase */
	ClockPhase clk_phase = (ClockPhase) SPI_Base->CR1.CPHA;
	/* Get the SPI baud rate control  */
	BaudRate baud_cntrl = (BaudRate) SPI_Base->CR1.BR;
	/* Get the SPI frame format  */
	FirstBit fb_sel = (FirstBit) SPI_Base->CR1.LSBFIRST;
	/* Get the SPI CRC Calculation  */
	EnableDisable CRC = (EnableDisable) SPI_Base->CR1.CRCEN;
	/* Get the SPI frame format  */
	FrameFormat format = (FrameFormat) SPI_Base->CR2.FRF;
	/*Reset the peripheral*/
	switch(SPIx)
	{
	case SPI1:
		/* SPI1 Peripheral Reset */
		RCC->APB2RSTR.SPI1RST = 1;

		RCC->APB2RSTR.SPI1RST = 0;

		break;

	case SPI2:
		/* SPI2 Peripheral Reset */
		RCC->APB1RSTR.SPI2RST = 1;

		RCC->APB1RSTR.SPI2RST = 0;

		break;

	case SPI3:
		/* SPI3 Peripheral Reset */
		RCC->APB1RSTR.SPI3RST = 1;

		RCC->APB1RSTR.SPI3RST = 0;

		break;
	}
	/*Then reinitialize the peripheral*/
	Init(mode_x, direction, clk_pol, clk_phase, baud_cntrl, fb_sel, format, CRC);
}

/**
* @brief Get the SPI status
* @return Status
*/
SPI_State_e SPI::GetState(void)
{
	SPI_State_e ret;
	/* If there is any error flags */
	if (SPI_Base->SR.FRE || SPI_Base->SR.OVR || SPI_Base->SR.MODF || SPI_Base->SR.CRCERR || SPI_Base->SR.UDR)
	{
		ret = ERROR;
	}
	/* If SPI is not enabled */
	else if (SPI_Base->CR1.SPE == 0)
	{
		ret = RESET;
	}
	/* If SPI is busy */
	else if (SPI_Base->SR.BSY)
	{
		ret = BUSY;
	}
	else
	{
		ret = READY;
	}
	/* Update SPI state */
	state = ret;

	return state;
}

/**
* @brief Transmits and receives 8-bit data from SPI
* @param [*TxData]    : Pointer of data to be transmitted
* @param [*RxData]    : Pointer of data to be received
* @param [Size]       : Size of the data to be transmitted
* @param [Timeout]    : Timeout in milliseconds
* @return SPI Status
*/
Status_TypeDef_e SPI::TransmitReceive(uint8_t* TxData, uint8_t* RxData, uint16_t Size, uint32_t Timeout)
{
	/* Get the system tick for timeout */
	uint32_t CurrentTick = SystemTick;
	/* Get the parameters for transmission */
	uint16_t TxCount = Size;

	uint16_t RxCount = Size;

	uint8_t TxAllowed = 1;
	/* If the SPI is not ready */
	while( GetState() == ERROR || GetState() == BUSY )
	{
		/* Wait for timeout */
		if( SystemTick - CurrentTick > Timeout )
		{
			return STATUS_TIMEOUT;
		}
	}
	/* Set data size to 8 bit */
    SPI_Base->CR1.DFF = EIGHT_BIT;
    /* Enable SPI to start transmit */
	SPI_Base->CR1.SPE = ENABLE;
	/* If the SPI is slave or size is 8 bit data */
	if(SPI_Base->CR1.MSTR == SLAVE || Size == 0x01U)
	{
		/* Write data to the data register */
		SPI_Base->DR.DR = *TxData;
		/* Increment the transfer buffer address */
		TxData ++;
		/* Decrement the transfer data size */
		TxCount --;
	}
	/* Until the transmission is complete */
	while(TxCount > 0 || RxCount > 0 )
	{
		/* If TX empty and there is data to be transferred and transfer process is unlocked */
		if(SPI_Base->SR.TXE == 1 && TxCount > 0 && TxAllowed == 1)
		{
			/* Write data to the data register */
			SPI_Base->DR.DR = *TxData;
			/* Increment the transfer buffer address */
			TxData ++;
			/* Decrement the transfer data size */
			TxCount --;
			/* Lock the transfer process */
			TxAllowed = 0;
		}
		/* If RX not empty and there is data to be received */
		if(SPI_Base->SR.RXNE == 1 && RxCount > 0)
		{
			/* Read data to the data register */
			*RxData = SPI_Base->DR.DR;
			/* Increment the receive buffer address */
			RxData++;
			/* Decrement the receive data size */
			RxCount --;
			/* Unlock the transfer process */
			TxAllowed = 1;
		}
		/* Check the timeout */
		if (SystemTick > CurrentTick + Timeout)
		{
			/* Disable SPI */
			SPI_Base->CR1.SPE = DISABLE;

			return STATUS_TIMEOUT;
		}
	}

	return EndTransmitReceive(Timeout);
}

/**
* @brief Transmits and receives 16-bit data from SPI
* @param [*TxData]    : Pointer of data to be transmitted
* @param [*RxData]    : Pointer of data to be received
* @param [Size]       : Size of the data to be transmitted
* @param [Timeout]    : Timeout in milliseconds
* @return SPI Status
*/
Status_TypeDef_e SPI::TransmitReceive(uint16_t* TxData, uint16_t* RxData, uint16_t Size, uint32_t Timeout)
{
	/* Get the system tick for timeout */
	uint32_t CurrentTick = SystemTick;
	/* Get the parameters for transmission */
	uint16_t TxCount = Size;

	uint16_t RxCount = Size;

	uint8_t TxAllowed = 1;
	/* If the SPI is not ready */
	while( GetState() == ERROR || GetState() == BUSY )
	{
		/* Wait for timeout */
		if( SystemTick - CurrentTick > Timeout )
		{
			return STATUS_TIMEOUT;
		}
	}
	/* Set data size to 16 bit */
    SPI_Base->CR1.DFF = SIXTEEN_BIT;
    /* Enable SPI to start transmit */
	SPI_Base->CR1.SPE = ENABLE;
	/* If the SPI is slave or size is 16 bit data */
	if(SPI_Base->CR1.MSTR == SLAVE || Size == 0x01U)
	{
		/* Write data to the data register */
		SPI_Base->DR.DR = *TxData;
		/* Increment the transfer buffer address */
		TxData ++;
		/* Decrement the transfer data size */
		TxCount --;
	}
	/* Until the transmission is complete */
	while(TxCount > 0 || RxCount > 0 )
	{
		/* If TX empty and there is data to be transferred and transfer process is unlocked */
		if(SPI_Base->SR.TXE == 1 && TxCount > 0 && TxAllowed == 1)
		{
			/* Write data to the data register */
			SPI_Base->DR.DR = *TxData;
			/* Increment the transfer buffer address */
			TxData ++;
			/* Decrement the transfer data size */
			TxCount --;
			/* Lock the transfer process */
			TxAllowed = 0;
		}
		/* If RX not empty and there is data to be received */
		if(SPI_Base->SR.RXNE == 1 && RxCount > 0)
		{
			/* Read data to the data register */
			*RxData = SPI_Base->DR.DR;
			/* Increment the receive buffer address */
			RxData++;
			/* Decrement the receive data size */
			RxCount --;
			/* Unlock the transfer process */
			TxAllowed = 1;
		}
		/* Check the timeout */
		if (SystemTick > CurrentTick + Timeout)
		{
			SPI_Base->CR1.SPE = DISABLE;

			return STATUS_TIMEOUT;
		}
	}

	return EndTransmitReceive(Timeout);
}

/**
* @brief This function set the transmits and receives ineterrupt
* @param [*TxData]    : Data to be transmitted
* @param [*RxData]    : Data to be received
* @param [Size]       : Size of the data to be transmitted
* @param [*callback]  : Callback function to be called when interrupt occurred
* @return Status
*/
Status_TypeDef_e SPI::TransmitReceiveIT(uint8_t* TxData, uint8_t* RxData, uint16_t size, void (*callback)(void))
{
	/* If the SPI is not ready */
	if( GetState() == ERROR )
	{	
		return STATUS_ERROR;
	}
	else if( GetState() == BUSY )
	{
		return STATUS_BUSY;
	}
	/* Get the Tx Data pointer */
	this->pTX_8bit_Buffer = TxData;
	/* Get the Rx Data pointer */
	this->pRX_8bit_Buffer = RxData;
	/* Attach the user callback function */
	User_callback = callback;
	/* Get the parameters */
	BufferSize = size;

	RXCount = size;

	TXCount = size;
	/* Set the Class pointer for interrupt service routine */
	switch (SPIx)
	{
	case SPI1:
		/* Set the class pointers */
		SPI_Class_Pointers[0] = this;

		break;

	case SPI2:
		/* Set the class pointers */
		SPI_Class_Pointers[1] = this;

		break;

	case SPI3:
		/* Set the class pointers */
		SPI_Class_Pointers[2] = this;

		break;
	}
	/* Set data size to 8 bit */
    SPI_Base->CR1.DFF = EIGHT_BIT;
	/* Peripheral side SPI Enable */
	SPI_Base->CR1.SPE = 1;
	/* Peripheral side SPI RX buffer not empty interrupt disable */
	SPI_Base->CR2.RXNEIE = 1;
	/* Peripheral side SPI TX buffer empty interrupt enable */
	SPI_Base->CR2.TXEIE = 1;
	/* Cortex side SPI IT enable */
	EnableInterrupt();

	return STATUS_OK;
}

/**
* @brief This function set the transmits and receives ineterrupt
* @param [*TxData]    : Data to be transmitted
* @param [*RxData]    : Data to be received
* @param [Size]       : Size of the data to be transmitted
* @param [*callback]  : Callback function to be called when interrupt occurred
* @return Status
*/
Status_TypeDef_e SPI::TransmitReceiveIT(uint16_t* TxData, uint16_t* RxData, uint16_t size, void (*callback)(void))
{
	/* If the SPI is not ready */
	if( GetState() == ERROR )
	{	
		return STATUS_ERROR;
	}
	else if( GetState() == BUSY )
	{
		return STATUS_BUSY;
	}
	/* Get the Tx Data pointer */
	this->pTX_16bit_Buffer = TxData;
	/* Get the Rx Data pointer */
	this->pRX_16bit_Buffer = RxData;
	/* Attach the user callback function */
	User_callback = callback;
	/* Get the parameters */
	BufferSize = size;

	RXCount = size;

	TXCount = size;
	/* Set the Class pointer for interrupt service routine */
	switch (SPIx)
	{
	case SPI1:
		/* Set the class pointers */
		SPI_Class_Pointers[0] = this;

		break;

	case SPI2:
		/* Set the class pointers */
		SPI_Class_Pointers[1] = this;

		break;

	case SPI3:
		/* Set the class pointers */
		SPI_Class_Pointers[2] = this;

		break;
	}
	/* Set data size to 8 bit */
    SPI_Base->CR1.DFF = SIXTEEN_BIT;
	/* Peripheral side SPI Enable */
	SPI_Base->CR1.SPE = 1;
	/* Peripheral side SPI RX buffer not empty interrupt disable */
	SPI_Base->CR2.RXNEIE = 1;
	/* Peripheral side SPI TX buffer empty interrupt enable */
	SPI_Base->CR2.TXEIE = 1;
	/* Cortex side SPI IT enable */
	EnableInterrupt();

	return STATUS_OK;
}
/**
* @brief This function attach the SPI Tx Completed callback function 
* @param [*callback]  : Void pointer function (This function is called when the TX interrupt is occurred)
* @return None
*/
void SPI::attachTxCompletedCallback(void (*callback)(void))
{
	TXCompletedCallback = callback;
}
    
/**
* @brief This function attach the SPI Rx Completed callback function 
* @param [*callback] : Void pointer function (This function is called when the RXinterrupt is occurred)
* @return None
*/
void SPI::attachRxCompletedCallback(void (*callback)(void))
{
	RXCompletedCallback = callback;
}

/**
* @brief This function attach the SPI TxRx Completed callback function
* @param [*callback] : Void pointer function (This function is called when the interrupt is occurred)
* @return None
*/
void SPI::attachTxRxCompletedCallback(void (*callback)(void))
{
	TXRXCompletedCallback = callback;
}

/**
* @brief This function attach the SPI Tx Half-Completed callback function
* @param [*callback]  : Void pointer function (This function is called when the TXinterrupt is occurred)
* @return None
*/
void SPI::attachTxHalfCompletedCallback(void (*callback)(void))
{
	TXHalfCompletedCallback = callback;
}

/**
* @brief This function attach the SPI Rx Half-Completed callback function
* @param [*callback] : Void pointer function (This function is called when the RXinterrupt is occurred)
* @return None
*/
void SPI::attachRxHalfCompletedCallback(void (*callback)(void))
{
	RXHalfCompletedCallback = callback;
}

/**
* @brief This function attach the SPI TxRx Half-Completed callback function
* @param [*callback] : Void pointer function (This function is called when the interrupt is occurred)
* @return None
*/
void SPI::attachTxRxHalfCompletedCallback(void (*callback)(void))
{
	TXRXHalfCompletedCallback = callback;
}

/**
* @brief Enable the SPI interrupt
* @return None
*/
void SPI::EnableInterrupt()
{
	switch(SPIx)
	{
	case SPI1:
		/* Cortex side SPI1 interrupt enable */
		NVIC_Reg->ISER[IRQ_NO_SPI1>>5] |= 1<<(IRQ_NO_SPI1%32);

		break;

	case SPI2:
		/* Cortex side SPI2 interrupt enable */
		NVIC_Reg->ISER[IRQ_NO_SPI2>>5] |= 1<<(IRQ_NO_SPI2%32);

		break;

	case SPI3:
		/* Cortex side SPI3 interrupt enable */
		NVIC_Reg->ISER[IRQ_NO_SPI3>>5] |= 1<<(IRQ_NO_SPI3%32);

		break;
	}
}

/**
* @brief Enable the SPI interrupt
* @return None
*/
void SPI::DisableInterrupt()
{
	switch(SPIx)
	{
	case SPI1:
		/* Cortex side SPI1 interrupt disable */
		NVIC_Reg->ICER[IRQ_NO_SPI1>>5] |= 1<<(IRQ_NO_SPI1%32);

		break;

	case SPI2:
		/* Cortex side SPI2 interrupt disable */
		NVIC_Reg->ICER[IRQ_NO_SPI2>>5] |= 1<<(IRQ_NO_SPI2%32);

		break;

	case SPI3:
		/* Cortex side SPI3 interrupt disable */
		NVIC_Reg->ICER[IRQ_NO_SPI3>>5] |= 1<<(IRQ_NO_SPI3%32);

		break;
	}
}

/**
* @brief This function sets the interrupt priority
* @param [Priority]   : specifies the interrupt priority
* @return None
*/
void SPI::SetInterruptPriority(uint8_t Priority)
{
	switch(SPIx)
	{
	case SPI1:
		/* Set Cortex side SPI1 interrupt priority */
		NVIC_Reg->IPR[IRQ_NO_SPI1] = Priority << 4;

		break;

	case SPI2:
		/* Set Cortex side SPI2 interrupt priority */
		NVIC_Reg->IPR[IRQ_NO_SPI2] = Priority << 4;

		break;

	case SPI3:
		/* Set Cortex side SPI3 interrupt priority */
		NVIC_Reg->IPR[IRQ_NO_SPI3] = Priority << 4;

		break;
	}
}

/*
* SPI Interrupt Handlers
*/
void SPI1_IRQHandler()
{
	if (SPI_Class_Pointers[0] != nullptr)
	{
		SPI_Class_Pointers[0]->ISR();
	}
}

void SPI2_IRQHandler()
{
	if (SPI_Class_Pointers[1] != nullptr)
	{
		SPI_Class_Pointers[1]->ISR();
	}
}

void SPI3_IRQHandler()
{
	if (SPI_Class_Pointers[2] != nullptr)
	{
		SPI_Class_Pointers[2]->ISR();
	}
}
/******************************************************************************/
/* Private functions **********************************************************/

/**
* @brief This function ends the Transmit or Receive operations at given time
* @param [Timeout]   : Timeout in milliseconds
* @return Status of the operation
*/
Status_TypeDef_e SPI::EndTransmitReceive(uint32_t Timeout)
{
	uint32_t TickStart = SystemTick;

	while(1)
	{
		/* Transmission successful Condition */
		if (SPI_Base->SR.BSY==0)
		{
			SPI_Base->CR1.SPE = DISABLE;

			return STATUS_OK;
		}
		/* Timeout Condition */
		if (SystemTick > TickStart + Timeout)
		{
			SPI_Base->CR1.SPE = DISABLE;

			return STATUS_TIMEOUT;
		}
	}

}

/**
* @brief Interrupt service routine for 8-bit
* @return None
*/
void SPI::ISR()
{
	static uint8_t tx_rx_halfcmplt_flag = 0;
	/* If TX empty and there is data to be transmitted. */
	if (SPI_Base->SR.TXE == 1 && TXCount > 0)
	{
		switch (SPI_Base->CR1.DFF)
		{
		case EIGHT_BIT:
			/* Write data to the data register */
			SPI_Base->DR.DR = *pTX_8bit_Buffer;
			/* Increment the transfer buffer address */
			pTX_8bit_Buffer++;

			break;

		case SIXTEEN_BIT:
			/* Write data to the data register */
			SPI_Base->DR.DR = *pTX_16bit_Buffer;
			/* Increment the transfer buffer address */
			pTX_16bit_Buffer++;

			break;
		}
		/* Decrement the transfer data size */
		TXCount--;
		/* If the data to be transferred has reached halfway */
		if( (TXCount * 2) == BufferSize || (TXCount * 2) == (BufferSize - 1) )
		{
			tx_rx_halfcmplt_flag++;
			/* If a TX Half-Completed callback function has been assigned*/
			if(TXHalfCompletedCallback != nullptr)
			{
				/* Execute callback function */
				TXHalfCompletedCallback();
			}
		}
		/* If transfer procces is done */
		else if (TXCount == 0)
		{
			/* If a TX Completed callback function has been assigned*/
			if(TXCompletedCallback != nullptr)
			{
				/* Execute callback function */
				TXCompletedCallback();
			}
			/* Peripheral side SPI TX buffer empty interrupt disable */
			SPI_Base->CR2.TXEIE = 0;
		}
	}
	/* If RX not empty and there is data to be received. */
	if (SPI_Base->SR.RXNE == 1 && RXCount > 0)
	{
		switch (SPI_Base->CR1.DFF)
		{
		case EIGHT_BIT:
			/* Read data to the data register */
			*pRX_8bit_Buffer = SPI_Base->DR.DR;
			/* Increment the receive buffer address */
			pRX_8bit_Buffer++;

			break;
					
		case SIXTEEN_BIT:
			/* Read data to the data register */
			*pRX_16bit_Buffer = SPI_Base->DR.DR;
			/* Increment the receive buffer address */
			pRX_16bit_Buffer++;	

			break;
		}
		/* Decrement the receive data size */
		RXCount--;	
		/* If the data to be received has reached halfway */
		if( (RXCount * 2) == BufferSize || (RXCount * 2) == (BufferSize - 1) )
		{
			tx_rx_halfcmplt_flag++;
			/* If a RX Half-Completed callback function has been assigned*/
			if(RXHalfCompletedCallback != nullptr)
			{
				/* Execute callback function */
				RXHalfCompletedCallback();
			}
		}
		/* If receive procces is done */
		else if (RXCount == 0)
		{
			/* If a RX Completed callback function has been assigned*/
			if(RXCompletedCallback != nullptr)
			{
				/* Execute callback function */
				RXCompletedCallback();
			}
			/* Peripheral side SPI RX buffer not empty interrupt disable */
			SPI_Base->CR2.RXNEIE = 0;
		}
	}
	/* If user callback function has been assigned */
	if(User_callback != nullptr)
	{
		/* Execute callback function */
		User_callback();
	}
	/* If the data to be transfered and received has reached halfway */
	if(tx_rx_halfcmplt_flag == 2)
	{
		tx_rx_halfcmplt_flag = 0;
		/* If a TXRX Half Completed callback function has been assigned */
		if(TXRXCompletedCallback != nullptr)
		{
			/* Execute callback function */
			TXRXCompletedCallback();
		}
	}
	/* If transmission is done */
	else if (TXCount == 0 && RXCount == 0)
	{
		/* If a TXRX Completed callback function has been assigned */
		if (TXRXCompletedCallback != nullptr)
		{
			/* Execute callback function */
			TXRXCompletedCallback();
		}
		/* Peripheral side SPI disable */
		SPI_Base->CR1.SPE = 0;
		/* Cortex side SPI IT disable */
		DisableInterrupt();
		/* If user callback function has been assigned */
		if(User_callback != nullptr)
		{
			/* Detach callback function */
			User_callback = nullptr;
		}
	}
}
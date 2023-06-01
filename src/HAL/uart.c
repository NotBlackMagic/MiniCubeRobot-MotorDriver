#include "uart.h"

#include "string.h"

#define UART_WRITE_TIMEOUT									1000	//Timeout to wait for TX Buffer empty in ms

#define UART_RX_BUFFER_SIZE									256
#define UART_TX_BUFFER_SIZE									2048

uint16_t uart1DMARXLength;
uint16_t uart1RXBufferLength;
uint8_t uart1RXBuffer[UART_RX_BUFFER_SIZE];
uint16_t uart1TXBufferLength;
uint8_t uart1TXBuffer[UART_TX_BUFFER_SIZE];

/**
  * @brief	This function initializes the UART1 interface, also sets the respective GPIO pins
  * @param	baud: UART Baud rate to use
  * @return	None
  */
void UART1Init(uint32_t baud) {
	//Enable bus clocks
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	//Configure GPIOs
//	LL_GPIO_AF_EnableRemap_USART1();	//Remap UART1 GPIOs to Alternative GPIOs
	//Set UART1 TX (PA9) as AF push-pull
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
	//Set UART1 RX (PA10) as input floating
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_FLOATING);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

	//Configure UART Interface
	LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
	LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
	LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
	LL_USART_SetBaudRate(USART1, SystemCoreClock, baud);

	//Configure UART Interrupts
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);
//	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_ERROR(USART1);
//	LL_USART_EnableIT_IDLE(USART1);
//	LL_USART_EnableIT_TXE(USART1);

	//Configure DMA
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	//TX DMA Configuration
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_4,
	                        	LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
								LL_DMA_PRIORITY_HIGH              |
								LL_DMA_MODE_NORMAL                |
								LL_DMA_PERIPH_NOINCREMENT         |
								LL_DMA_MEMORY_INCREMENT           |
								LL_DMA_PDATAALIGN_BYTE            |
								LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)uart1TXBuffer, LL_USART_DMA_GetRegAddr(USART1), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, 0);
	//RX DMA Configuration
	LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5,
								LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
								LL_DMA_PRIORITY_HIGH              |
								LL_DMA_MODE_NORMAL                |
								LL_DMA_PERIPH_NOINCREMENT         |
								LL_DMA_MEMORY_INCREMENT           |
								LL_DMA_PDATAALIGN_BYTE            |
								LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5, LL_USART_DMA_GetRegAddr(USART1), (uint32_t)uart1RXBuffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, 2);

	//Configure DMA Interrupts
	NVIC_SetPriority(DMA1_Channel4_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
	NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);

	//Enable DMA RX Request
	LL_USART_EnableDMAReq_RX(USART1);

	//Enable DMA Channel RX
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

	//Enable UART
	LL_USART_Enable(USART1);
}

/**
  * @brief	This function sets the UART1 Baudrate
  * @param	baudrate: The baudrate to use/set
  * @return	None
  */
void UART1SetBaudrate(uint32_t baudrate) {
	LL_USART_SetBaudRate(USART1, SystemCoreClock, baudrate);
}

/**
  * @brief	This function sends data over UART1
  * @param	data: data array to transmit
  * @param	length: length of the transmit data array
  * @return	0 -> if all good, no errors; 1 -> TX Buffer is full
  */
uint8_t UART1Write(uint8_t* data, uint16_t length) {
	//Wait for buffer empty, if is not
	uint32_t timestamp = GetSysTick();
	while(uart1TXBufferLength != 0x00 && (timestamp + UART_WRITE_TIMEOUT) > GetSysTick());

	if(uart1TXBufferLength == 0x00) {
		//**********************//
		//		UART Frame		//
		//**********************//
		//|  u16   | n*u8 | u16 |
		//|--------|------|-----|
		//| Length | Data | CRC |
		uint16_t index = 0;

		//Add length field
		uart1TXBuffer[index++] = (uint8_t)((length >> 8) & 0xFF);
		uart1TXBuffer[index++] = (uint8_t)((length) & 0xFF);

		//Add payload/data
		memcpy(&uart1TXBuffer[index], data, length);
		index = index + length;

		//Calculate and add CRC
		uint16_t crc;
		CRCReset(&crc);
		uint16_t i;
		for(i = 0; i < (length + 2); i++) {
			CRCWrite(uart1TXBuffer[i], &crc);
		}
		crc = CRCRead(&crc);
		uart1TXBuffer[index++] = (uint8_t)((crc >> 8) & 0xFF);
		uart1TXBuffer[index++] = (uint8_t)((crc) & 0xFF);

		uart1TXBufferLength = index;

		//Set DMA TX Data Length
		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, uart1TXBufferLength);

		//Enable DMA TX Request
		LL_USART_EnableDMAReq_TX(USART1);

		//Enable DMA Channel TX
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

//		LL_USART_TransmitData8(USART1, uart1TXBuffer[uart1TXBufferIndex++]);
//
//		LL_USART_EnableIT_TXE(USART1);
	}
	else {
		//Buffer full, return error
		return 1;
	}

	return 0;
}

/**
  * @brief	This function reads the UART1 RX buffer
  * @param	data: data array to where the received data should be copied to
  * @param	length: length of the received array, is set in this function
  * @return	Returns 0 if no new data, 1 if new data
  */
uint8_t UART1Read(uint8_t* data, uint16_t* length) {
	if(uart1RXBufferLength > 0) {
		//Calculate CRC over received data
		uint16_t crc;
		CRCReset(&crc);
		CRCWrite((uart1RXBufferLength >> 8), &crc);
		CRCWrite(uart1RXBufferLength, &crc);
		uint16_t i;
		for(i = 0; i < uart1RXBufferLength; i++) {
			CRCWrite(uart1RXBuffer[i], &crc);
		}
		crc = CRCRead(&crc);

		//Extract CRC from received data
		uint16_t rxCRC = (uart1RXBuffer[uart1RXBufferLength] << 8) + uart1RXBuffer[uart1RXBufferLength + 1];
		if(crc != rxCRC) {
			//Invalid CRC
			uart1RXBufferLength = 0;

			return 0;
		}

		for(i = 0; i < uart1RXBufferLength; i++) {
			data[i] = uart1RXBuffer[i];
		}

		*length = uart1RXBufferLength;

		uart1RXBufferLength = 0;

		return 1;
	}
	else {
		return 0;
	}
}

/**
  * @brief	UART1 IRQ Handler
  * @param	None
  * @return	None
  */
void USART1_IRQHandler(void) {
	//**********************//
	//		UART Frame		//
	//**********************//
	//|  u16   | n*u8 | u16 |
	//|--------|------|-----|
	//| Length | Data | CRC |
//	if(LL_USART_IsEnabledIT_TXE(USART1) == 0x01 && LL_USART_IsActiveFlag_TXE(USART1) == 0x01) {
//		if(uart1TXBufferIndex >= uart1TXBufferLength) {
//			//Transmission complete
//			uart1TXBufferLength = 0;
//			uart1TXBufferIndex = 0;
//
//			//Disable TX done interrupt
//			LL_USART_DisableIT_TXE(USART1);
//		}
//		else {
//			//Transmit another byte
//			LL_USART_TransmitData8(USART1, uart1TXBuffer[uart1TXBufferIndex++]);
//		}
//	}
//
//	if(LL_USART_IsActiveFlag_RXNE(USART1) == 0x01) {
//		if(uart1RXBufferLength != 0x00) {
//			//RX Buffer full, has a complete frame in it
//			uint8_t dummy = LL_USART_ReceiveData8(USART1);
//
//			uart1RXBufferIndex = 0;
//		}
//		else if(uart1RXBufferIndex >= UART_RX_BUFFER_SIZE) {
//			//RX Buffer overflow
//			uint8_t dummy = LL_USART_ReceiveData8(USART1);
//
//			uart1RXBufferIndex = 0;
//		}
//		else {
//			//All good, read received byte to RX buffer
//			uint8_t rxByte = LL_USART_ReceiveData8(USART1);
//			uart1RXBuffer[uart1RXBufferIndex++] = rxByte;
//
//			//Payload length byte received
//			uint16_t payloadLength = (uart1RXBuffer[0] << 8) + uart1RXBuffer[1];
//
//			//Update CRC
//			if(uart1RXBufferIndex == 1) {
//				CRCReset(&uart1RXCRC);
//				CRCWrite(rxByte, &uart1RXCRC);
//			}
//			else if(uart1RXBufferIndex <= (payloadLength + 2)) {
//				CRCWrite(rxByte, &uart1RXCRC);
//			}
//
//			//End of transmission detection based on UART frame length, in bytes 0 and 1
//			if(uart1RXBufferIndex >= 2) {
//				//Check for end of packet
//				if(payloadLength < 1) {
//					//Invalid payload length
//					uart1RXBufferIndex = 0;
//				}
//				else if(payloadLength > UART_RX_BUFFER_SIZE) {
//					//Buffer overflow
//					uart1RXBufferIndex = 0;
//				}
//				else if(uart1RXBufferIndex >= (payloadLength + 4)) {
//					//Full packet received, get CRC from package (last two bytes)
//					//CRC Check
//					uint16_t crc = (uart1RXBuffer[uart1RXBufferIndex - 2] << 8) + uart1RXBuffer[uart1RXBufferIndex - 1];
//					if (crc != CRCRead(&uart1RXCRC)) {
//						//Packet CRC failed
//						uart1RXBufferIndex = 0;
//					}
//					else {
//						uart1RXBufferLength = uart1RXBufferIndex;
//					}
//				}
//			}
//		}
//	}

	if(LL_USART_IsActiveFlag_IDLE(USART1) == 0x01) {
		//End of frame transmission, detected by receiver timeout
//		uart1RXBufferLength = uart1RXBufferIndex;

		//Clear IDLE Flag
		LL_USART_ClearFlag_IDLE(USART1);
	}

	if(LL_USART_IsActiveFlag_FE(USART1) == 0x01 || LL_USART_IsActiveFlag_NE(USART1) == 0x01 || LL_USART_IsActiveFlag_ORE(USART1) == 0x01) {
		//Some USART Error Interrupt

		//Clear all Error Flags
		LL_USART_ClearFlag_ORE(USART1);
	}
}

/**
  * @brief	DMA1 Channel 4 IRQ Handler
  * @param	None
  * @return	None
  */
void DMA1_Channel4_IRQHandler(void) {
	if(LL_DMA_IsActiveFlag_TC4(DMA1) == 0x01) {
		//Transmission complete interrupt
		uart1TXBufferLength = 0;

		LL_DMA_ClearFlag_TC4(DMA1);

		//Disable DMA Channel
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	}
	else if(LL_DMA_IsActiveFlag_TE4(DMA1) == 0x01) {
		//Some DMA Error Interrupt
		uart1TXBufferLength = 0;

		//Disable DMA Channel
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
	}
}

/**
  * @brief	DMA1 Channel 5 IRQ Handler
  * @param	None
  * @return	None
  */
void DMA1_Channel5_IRQHandler(void) {
	if(LL_DMA_IsActiveFlag_TC5(DMA1) == 0x01) {
		//Transmission complete interrupt
		if(uart1DMARXLength == 0x00) {
			//First transfer complete, get RX length and restart DMA
			uart1DMARXLength = (uart1RXBuffer[0] << 8) + uart1RXBuffer[1];

			//Disable DMA Channel
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

			if((uart1DMARXLength + 4) > UART_RX_BUFFER_SIZE) {
				//Invalid payload length
				uart1DMARXLength = 0;

				//Update DMA RX length: Set to 2 bytes, the length field length of the next packet
				LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, 2);
			}
			else {
				//Update DMA RX length: Packet payload length + CRC field (uint16_t)
				LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, uart1DMARXLength + 2);
			}

			//Re-enable DMA Channel
			LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
		}
		else {
			//Second transfer complete received, full data received
			uart1RXBufferLength = uart1DMARXLength;
			uart1DMARXLength = 0;

			//Disable DMA Channel
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

			//Change DMA RX buffer back to start of RX buffer
			LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5, LL_USART_DMA_GetRegAddr(USART1), (uint32_t)uart1RXBuffer, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

			//Update DMA RX length: Set to 2 bytes, the length field length of the next packet
			LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, 2);

			//Re-enable DMA Channel
			LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
		}

		LL_DMA_ClearFlag_TC5(DMA1);
	}
	else if(LL_DMA_IsActiveFlag_TE5(DMA1) == 0x01) {
		//Some DMA Error Interrupt
		uart1RXBufferLength = 0;

		//Disable DMA Channel
		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
	}
}

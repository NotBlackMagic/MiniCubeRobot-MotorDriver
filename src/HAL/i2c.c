#include "i2c.h"

uint8_t i2c1RXDone;
uint8_t i2c1SlaveAddress;
uint8_t i2c1RXBufferIndex;
uint8_t i2c1RXLength;
uint8_t i2c1RXBuffer[100];

void I2C1Init(I2CMode mode) {
	//Enable bus clocks
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	//Configure GPIOs
	//Set I2C2 SCL (PB6) as AF push-pull
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_OPENDRAIN);
//	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_6, LL_GPIO_PULL_UP);
	//Set I2C2 SDA (PB7) as AF push-pull
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
//	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);

	//Configure I2C Interface
	LL_I2C_Disable(I2C1);
	if(mode == I2CMode_SM) {
		LL_I2C_ConfigSpeed(I2C1, (SystemCoreClock >> 1), 100000, LL_I2C_DUTYCYCLE_2);
	}
	else if(mode == I2CMode_FM) {
		LL_I2C_ConfigSpeed(I2C1, (SystemCoreClock >> 1), 400000, LL_I2C_DUTYCYCLE_2);
	}
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

	//Configure I2C Interrupts
//	NVIC_SetPriority(I2C1_EV_IRQn, 0);
//	NVIC_EnableIRQ(I2C1_EV_IRQn);
//	NVIC_SetPriority(I2C1_ER_IRQn, 0);
//	NVIC_EnableIRQ(I2C1_ER_IRQn);

	//Enable I2C
	LL_I2C_Enable(I2C1);
}

void I2C2Init(I2CMode mode) {
	//Enable bus clocks
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);

	//Configure GPIOs
	//Set I2C2 SCL (PB10) as AF push-pull
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
//	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
	//Set I2C2 SDA (PB11) as AF push-pull
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
//	LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

	//Configure I2C Interface
	LL_I2C_Disable(I2C2);
	if(mode == I2CMode_SM) {
		LL_I2C_ConfigSpeed(I2C2, (SystemCoreClock >> 1), 100000, LL_I2C_DUTYCYCLE_2);
	}
	else if(mode == I2CMode_FM) {
		LL_I2C_ConfigSpeed(I2C2, (SystemCoreClock >> 1), 400000, LL_I2C_DUTYCYCLE_2);
	}
	LL_I2C_SetOwnAddress1(I2C2, 0x08, LL_I2C_OWNADDRESS1_7BIT);		//Configure the Own Address1
//	LL_I2C_SetOwnAddress2(I2C2, 0x00);
//	LL_I2C_DisableOwnAddress2(I2C2);
	LL_I2C_EnableClockStretching(I2C2);				//Enable Clock stretching
	LL_I2C_DisableGeneralCall(I2C2);				//Disable General Call
	LL_I2C_SetMode(I2C2, LL_I2C_MODE_I2C);			//Enable Peripheral in I2C mode
//	LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);

	//Configure I2C Interrupts
//	NVIC_SetPriority(I2C1_EV_IRQn, 0);
//	NVIC_EnableIRQ(I2C1_EV_IRQn);
//	NVIC_SetPriority(I2C1_ER_IRQn, 0);
//	NVIC_EnableIRQ(I2C1_ER_IRQn);

	//Enable I2C
	LL_I2C_Enable(I2C2);
}

uint8_t I2C1Write(uint8_t address, uint8_t* data, uint8_t length) {
	//Check/wait for I2C to be free/not busy
	while(LL_I2C_IsActiveFlag_BUSY(I2C1));

	//Generate I2C start condition
	LL_I2C_GenerateStartCondition(I2C1);
	//Wait until I2C start condition bit is transmitted
	while(!LL_I2C_IsActiveFlag_SB(I2C1));

	//Send slave address as Write
	LL_I2C_TransmitData8(I2C1, (address << 1) | 0x00);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);			//Clear ADDR flag value in ISR register

	uint8_t i;
	for(i = 0; i < length; i++) {
		while(!LL_I2C_IsActiveFlag_TXE(I2C1));	//Wait for I2C EV8_2: It means that the data has been physically shifted out and output on the bus
		LL_I2C_TransmitData8(I2C1, data[i]);	//Send data byte
	}

	LL_I2C_GenerateStopCondition(I2C1);		//Generate I2C stop condition
	while(LL_I2C_IsActiveFlag_STOP(I2C1));	//Wait until I2C stop condition is finished

	return 0;
}

uint8_t I2C2Write(uint8_t address, uint8_t* data, uint8_t length) {
	//Check/wait for I2C to be free/not busy
	while(LL_I2C_IsActiveFlag_BUSY(I2C2));

	LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);

	//Generate I2C start condition
	LL_I2C_GenerateStartCondition(I2C2);
	//Wait until I2C start condition bit is transmitted
	while(!LL_I2C_IsActiveFlag_SB(I2C2));

	//Send slave address as Write
	LL_I2C_TransmitData8(I2C2, (address << 1) | 0x00);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C2));
	LL_I2C_ClearFlag_ADDR(I2C2);			//Clear ADDR flag value in ISR register

	uint8_t i;
	for(i = 0; i < length; i++) {
		while(!LL_I2C_IsActiveFlag_TXE(I2C2));	//Wait for I2C EV8_2: It means that the data has been physically shifted out and output on the bus
		LL_I2C_TransmitData8(I2C2, data[i]);	//Send data byte
	}

	LL_I2C_GenerateStopCondition(I2C2);		//Generate I2C stop condition
	while(LL_I2C_IsActiveFlag_STOP(I2C2));	//Wait until I2C stop condition is finished

	return 0;
}

uint8_t I2C1Read(uint8_t address, uint8_t data[], uint8_t length) {
	while(LL_I2C_IsActiveFlag_BUSY(I2C1));	//Wait until I2Cx is not busy anymore

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);	//Enable ACK of received data

	LL_I2C_GenerateStartCondition(I2C1);	//Generate I2C start condition
	while(!LL_I2C_IsActiveFlag_SB(I2C1));	//Wait until I2C start condition bit is transmitted

//	LL_I2C_TransmitData8(I2C1, (address << 1) | 0x00);	//Send slave address as Write
//	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
//	LL_I2C_ClearFlag_ADDR(I2C1);
//
//	LL_I2C_TransmitData8(I2C1, reg);		//Send register address
//	while(!LL_I2C_IsActiveFlag_TXE(I2C1));	//Wait for I2C EV8_2: It means that the data has been physically shifted out and output on the bus
//
//	LL_I2C_GenerateStartCondition(I2C1);	//Generate I2C repeated start condition
//	while(!LL_I2C_IsActiveFlag_SB(I2C1));	//Wait until I2C start condition bit is transmitted

	LL_I2C_TransmitData8(I2C1, (address << 1) | 0x01);	//Send slave address as Read
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	uint8_t i;
	for(i = 0; i < length; i++) {
		LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);	//Enable ACK of received data
		while(!LL_I2C_IsActiveFlag_RXNE(I2C1));			//Wait for I2C EV7: It means that the data has been received in I2C data register
		data[i] = LL_I2C_ReceiveData8(I2C1);			//Read and return data byte from I2C data register
	}

	/*enable NACK bit */
	LL_I2C_DisableBitPOS(I2C1);
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

	LL_I2C_GenerateStopCondition(I2C1);		//Send STOP Condition
	while (LL_I2C_IsActiveFlag_STOP(I2C1));

	return 1;
}

uint8_t I2C2Read(uint8_t address, uint8_t data[], uint8_t length) {
	while(LL_I2C_IsActiveFlag_BUSY(I2C2));	//Wait until I2Cx is not busy anymore

	LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);	//Enable ACK of received data

	LL_I2C_GenerateStartCondition(I2C2);	//Generate I2C start condition
	while(!LL_I2C_IsActiveFlag_SB(I2C2));	//Wait until I2C start condition bit is transmitted

//	LL_I2C_TransmitData8(I2C2, (address << 1) | 0x00);	//Send slave address as Write
//	while(!LL_I2C_IsActiveFlag_ADDR(I2C2));
//	LL_I2C_ClearFlag_ADDR(I2C2);
//
//	LL_I2C_TransmitData8(I2C2, reg);		//Send register address
//	while(!LL_I2C_IsActiveFlag_TXE(I2C2));	//Wait for I2C EV8_2: It means that the data has been physically shifted out and output on the bus
//
//	LL_I2C_GenerateStartCondition(I2C2);	//Generate I2C repeated start condition
//	while(!LL_I2C_IsActiveFlag_SB(I2C2));	//Wait until I2C start condition bit is transmitted

	LL_I2C_TransmitData8(I2C2, (address << 1) | 0x01);	//Send slave address as Read
	while (!LL_I2C_IsActiveFlag_ADDR(I2C2));
	LL_I2C_ClearFlag_ADDR(I2C2);

	uint8_t i;
	for(i = 0; i < length; i++) {
		LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);	//Enable ACK of received data
		while(!LL_I2C_IsActiveFlag_RXNE(I2C2));			//Wait for I2C EV7: It means that the data has been received in I2C data register
		data[i] = LL_I2C_ReceiveData8(I2C2);			//Read and return data byte from I2C data register
	}

	/*enable NACK bit */
	LL_I2C_DisableBitPOS(I2C2);
	LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);

	LL_I2C_GenerateStopCondition(I2C2);		//Send STOP Condition
	while (LL_I2C_IsActiveFlag_STOP(I2C2));

	return 1;
}

void I2C1StartRead(uint8_t address, uint8_t reg, uint8_t length) {
	if(i2c1RXLength != 0x00 || i2c1RXDone == 0x01) {
		//Either in process of reading some data or not yet read the received buffer
		return;
	}

	while(LL_I2C_IsActiveFlag_BUSY(I2C1));	//Wait until I2Cx is not busy anymore

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);	//Enable ACK of received data

	LL_I2C_GenerateStartCondition(I2C1);	//Generate I2C start condition
	while(!LL_I2C_IsActiveFlag_SB(I2C1));	//Wait until I2C start condition bit is transmitted

	LL_I2C_TransmitData8(I2C1, (address << 1) | 0x00);	//Send slave address as Write
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, reg);		//Send register address
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));	//Wait for I2C EV8_2: It means that the data has been physically shifted out and output on the bus

	i2c1SlaveAddress = address;
	i2c1RXBufferIndex = 0;
	i2c1RXLength = length;
	i2c1RXDone = 0;

	LL_I2C_EnableIT_EVT(I2C1);
	LL_I2C_EnableIT_ERR(I2C1);

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);	//Enable ACK of received data
	LL_I2C_GenerateStartCondition(I2C1);			//Generate I2C repeated start condition
}

uint8_t I2C1ReadBuffer(uint8_t data[]) {
	uint8_t i;

	if(i2c1RXDone == 0x00) {
		return 1;
	}

	for(i = 0; i < i2c1RXBufferIndex; i++) {
		data[i] = i2c1RXBuffer[i];
	}

	i2c1RXBufferIndex = 0;
	i2c1RXDone = 0;

	return 0;
}

void I2C1_EV_IRQHandler() {
	if(LL_I2C_IsActiveFlag_SB(I2C1)) {
		//Start condition generated interrupt, send address as read
		LL_I2C_TransmitData8(I2C1, (i2c1SlaveAddress << 1) | 0x01);
	}
	else if(LL_I2C_IsActiveFlag_ADDR(I2C1)) {
		//Address interrupt
//		LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

		//Enable Buffer Interrupt
		LL_I2C_EnableIT_BUF(I2C1);

		//Clear Flag
		LL_I2C_ClearFlag_ADDR(I2C1);
	}
	else if(LL_I2C_IsActiveFlag_TXE(I2C1)) {
		//Transmit complete interrupt
	}
//	else if(LL_I2C_IsActiveFlag_BTF(I2C1)) {
//		//Byte transfer finished flag
//	}
	else if(LL_I2C_IsActiveFlag_RXNE(I2C1)) {
		//Data receive interrupt
		if(i2c1RXLength == 2) {
			LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
			LL_I2C_GenerateStopCondition(I2C1);

			i2c1RXBuffer[i2c1RXBufferIndex++] = LL_I2C_ReceiveData8(I2C1);
			i2c1RXLength -= 1;
		}
		else {
			i2c1RXBuffer[i2c1RXBufferIndex++] = LL_I2C_ReceiveData8(I2C1);
			i2c1RXLength -= 1;

			if(i2c1RXLength == 0x00) {
				i2c1RXDone = 1;

				LL_I2C_DisableIT_BUF(I2C1);
				LL_I2C_DisableIT_EVT(I2C1);
				LL_I2C_DisableIT_ERR(I2C1);
			}
		}
	}
}

void I2C1_ER_IRQHandler() {
	//Error Interrupt
}

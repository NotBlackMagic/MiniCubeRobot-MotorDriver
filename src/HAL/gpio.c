#include "gpio.h"

#include "pinMaping.h"

/**
  * @brief	This function initializes the GPIO that don't use a peripheral (ADC/UART/SPI etc)
  * @param	None
  * @return	None
  */
void GPIOInit() {
	//Enable Port Clocks
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);

	//Enable EXTI/AFIO Clocks and Power
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	//Enable PC14 and PC15 as general IO
	LL_GPIO_AF_Remap_SWJ_NOJTAG();
	//Enable PD0 and PD1 as general IO
	LL_GPIO_AF_EnableRemap_PD01();

	//Set ADC Input GPIO's
	GPIOSetPinMode(GPIO_ADC_HAL_S1, GPIO_Mode_Analog);	//ADC Input HAL Sensor 1
	GPIOSetPinMode(GPIO_ADC_HAL_S2, GPIO_Mode_Analog);	//ADC Input HAL Sensor 2
	GPIOSetPinMode(GPIO_ADC_SEN_A, GPIO_Mode_Analog);	//ADC Input Reflective Sensor A
	GPIOSetPinMode(GPIO_ADC_SEN_B, GPIO_Mode_Analog);	//ADC Input Reflective Sensor B
	GPIOSetPinMode(GPIO_ADC_SEN_C, GPIO_Mode_Analog);	//ADC Input Reflective Sensor C
	GPIOSetPinMode(GPIO_ADC_BAT_V, GPIO_Mode_Analog);	//ADC Input Battery Voltage
	GPIOSetPinMode(GPIO_ADC_BAT_I, GPIO_Mode_Analog);	//ADC Input Battery Current
	GPIOSetPinMode(GPIO_ADC_ML_I, GPIO_Mode_Analog);	//ADC Input Motor Left Current
	GPIOSetPinMode(GPIO_ADC_MR_I, GPIO_Mode_Analog);	//ADC Input Motor Right Current

	//Set Input GPIO's
	GPIOSetPinMode(GPIO_IN_BTN_FR, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_BTN_FL, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_BTN_BR, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_BTN_BL, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_M_FAULT, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_BAT_CHG, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_ML_ENC, GPIO_Mode_Input);
	GPIOSetPinMode(GPIO_IN_MR_ENC, GPIO_Mode_Input);

	//Set Input Pins Interrupts
	//Motor Encoder Left
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTC, LL_GPIO_AF_EXTI_LINE15);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_15);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_15);
	//Motor Encoder Right
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE7);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_7);
//	//Collision Button Front Right
//	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE8);
//	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_8);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_8);
//	//Collision Button Front Left
//	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE11);
//	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_11);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_11);
//	//Collision Button Back Right
//	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE12);
//	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_12);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_12);
//	//Collision Button Back Left
//	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE11);
//	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_11);
//	LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_11);

	//Set Output GPIO's
	GPIOSetPinMode(GPIO_OUT_LED, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_M_EN, GPIO_Mode_Output);
//	GPIOSetPinMode(GPIO_OUT_M_REF, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_ML_PH, GPIO_Mode_Output);
//	GPIOSetPinMode(GPIO_OUT_ML_PWM, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_MR_PH, GPIO_Mode_Output);
//	GPIOSetPinMode(GPIO_OUT_MR_PWM, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_SEN_F_EN, GPIO_Mode_Output);
	GPIOSetPinMode(GPIO_OUT_SEN_B_EN, GPIO_Mode_Output);
//	GPIOSetPinMode(GPIO_OUT_SPI_CS, GPIO_Mode_Output);

	//Set Output GPIOs
	GPIOWrite(GPIO_OUT_LED, 0x00);
	GPIOWrite(GPIO_OUT_M_EN, 0x00);
//	GPIOWrite(GPIO_OUT_M_REF, 0x00);
	GPIOWrite(GPIO_OUT_ML_PH, 0x00);
//	GPIOWrite(GPIO_OUT_ML_PWM, 0x00);
	GPIOWrite(GPIO_OUT_MR_PH, 0x00);
//	GPIOWrite(GPIO_OUT_MR_PWM, 0x00);
	GPIOWrite(GPIO_OUT_SEN_F_EN, 0x00);
	GPIOWrite(GPIO_OUT_SEN_B_EN, 0x00);
//	GPIOWrite(GPIO_OUT_SPI_CS, 0x00);

	//Enable EXTI Interrupts
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriority(EXTI9_5_IRQn, 0);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 0);
}

/**
  * @brief	This function sets the output mode type of a pin
  * @param	gpio: Pin to define output mode
  * @param	mode: Output mode of this pin
  * @return	None
  */
void GPIOSetPinMode(uint8_t gpio, GPIOOutputMode mode) {
	uint8_t port = (gpio >> 4);
	uint8_t pin = gpio & 0x0F;
	LL_GPIO_SetPinMode(gpioPorts[port], gpioPins[pin], gpioOutputMode[mode]);
}

/**
  * @brief	This function sets the output of a pin
  * @param	gpio: Pin to set output
  * @param	on: 1 output is set high, 0 output is set low
  * @return	None
  */
void GPIOWrite(uint8_t gpio, uint8_t on) {
	uint8_t port = (gpio >> 4);
	uint8_t pin = gpio & 0x0F;
	if(on == 1) {
		LL_GPIO_SetOutputPin(gpioPorts[port], gpioPins[pin]);
	}
	else {
		LL_GPIO_ResetOutputPin(gpioPorts[port], gpioPins[pin]);
	}
}

/**
  * @brief	This function gets the input state of a pin
  * @param	gpio: Pin to set output
  * @return	1 input is set high, 0 input is set low
  */
uint8_t GPIORead(uint8_t gpio) {
	uint8_t port = (gpio >> 4);
	uint8_t pin = gpio & 0x0F;
	return LL_GPIO_IsInputPinSet(gpioPorts[port], gpioPins[pin]);
}

/**
  * @brief	This function toggles the output state of a pin
  * @param	gpio: Pin to toggle output
  * @return	1 input is set high, 0 input is set low
  */
void GPIOToggle(uint8_t gpio) {
	uint8_t port = (gpio >> 4);
	uint8_t pin = gpio & 0x0F;
	uint8_t status = LL_GPIO_IsOutputPinSet(gpioPorts[port], gpioPins[pin]);
	if(status == 1) {
		LL_GPIO_SetOutputPin(gpioPorts[port], gpioPins[pin]);
	}
	else {
		LL_GPIO_ResetOutputPin(gpioPorts[port], gpioPins[pin]);
	}
}

__attribute__((weak)) void EXTI0Callback() {}

/**
  * @brief	This function is the Handler for GPIO0s
  * @param	None
  * @return	None
  */
void EXTI0_IRQHandler(void) {
	//EXTI0 Triggered, call callback function
	EXTI0Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

__attribute__((weak)) void EXTI1Callback() {}

/**
  * @brief	This function is the Handler for GPIO1s
  * @param	None
  * @return	None
  */
void EXTI1_IRQHandler(void) {
	//EXTI1 Triggered, call callback function
	EXTI1Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
}

__attribute__((weak)) void EXTI2Callback() {}

/**
  * @brief	This function is the Handler for GPIO2s
  * @param	None
  * @return	None
  */
void EXTI2_IRQHandler(void) {
	//EXTI2 Triggered, call callback function
	EXTI2Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
}

__attribute__((weak)) void EXTI3Callback() {}

/**
  * @brief	This function is the Handler for GPIO3s
  * @param	None
  * @return	None
  */
void EXTI3_IRQHandler(void) {
	//EXTI3 Triggered, call callback function
	EXTI3Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
}

__attribute__((weak)) void EXTI4Callback() {}

/**
  * @brief	This function is the Handler for GPIO4s
  * @param	None
  * @return	None
  */
void EXTI4_IRQHandler(void) {
	//EXTI4 Triggered, call callback function
	EXTI4Callback();

	//Clear Interrupt Flag
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
}

__attribute__((weak)) void EXTI5Callback() {}
__attribute__((weak)) void EXTI6Callback() {}
__attribute__((weak)) void EXTI7Callback() {}
__attribute__((weak)) void EXTI8Callback() {}
__attribute__((weak)) void EXTI9Callback() {}

/**
  * @brief	This function is the Handler for GPIO5s to GPIO9s
  * @param	None
  * @return	None
  */
void EXTI9_5_IRQHandler(void) {
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_5) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) == 0x01) {
		//EXTI5 Triggered, call callback function
		EXTI5Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_6) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) == 0x01) {
		//EXTI6 Triggered, call callback function
		EXTI6Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_7) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) == 0x01) {
		//EXTI7 Triggered, call callback function
		EXTI7Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_8) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) == 0x01) {
		//EXTI8 Triggered, call callback function
		EXTI8Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_9) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) == 0x01) {
		//EXTI9 Triggered, call callback function
		EXTI9Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
	}
}

__attribute__((weak)) void EXTI10Callback() {}
__attribute__((weak)) void EXTI11Callback() {}
__attribute__((weak)) void EXTI12Callback() {}
__attribute__((weak)) void EXTI13Callback() {}
__attribute__((weak)) void EXTI14Callback() {}
__attribute__((weak)) void EXTI15Callback() {}

/**
  * @brief	This function is the Handler for GPIO10s to GPIO15s
  * @param	None
  * @return	None
  */
void EXTI15_10_IRQHandler(void) {
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_10) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) == 0x01) {
		//EXTI10 Triggered, call callback function
		EXTI10Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_11) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) == 0x01) {
		//EXTI11 Triggered, call callback function
		EXTI11Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_12) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) == 0x01) {
		//EXTI12 Triggered, call callback function
		EXTI12Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_13) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) == 0x01) {
		//EXTI13 Triggered, call callback function
		EXTI13Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_14) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) == 0x01) {
		//EXTI14 Triggered, call callback function
		EXTI14Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
	}
	if(LL_EXTI_IsEnabledIT_0_31(LL_EXTI_LINE_15) == 0x01 && LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) == 0x01) {
		//EXTI15 Triggered, call callback function
		EXTI15Callback();

		//Clear Interrupt Flag
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
	}
}

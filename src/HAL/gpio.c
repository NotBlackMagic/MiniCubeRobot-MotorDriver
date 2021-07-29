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
//	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE0);
//	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
//	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

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

//	NVIC_EnableIRQ(EXTI0_IRQn);
//	NVIC_SetPriority(EXTI0_IRQn, 0);
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

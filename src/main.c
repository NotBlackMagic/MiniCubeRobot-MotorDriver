#include "gpio.h"
#include "pwm.h"
#include "rcc.h"
#include "uart.h"

#include "pinMaping.h"

int main(void) {
	//Configure the system clock
	SystemClockInit();
	SystemTickInit();

	//Initialize all configured peripherals
	GPIOInit();
	UART1Init(115200);
	ADC1Init();
//	PWM4Init();

	//Set LED ON
	GPIOWrite(GPIO_OUT_LED, 0x01);

	GPIOWrite(GPIO_OUT_M_EN, 0x01);
//	GPIOWrite(GPIO_OUT_M_REF, 0x01);

	GPIOWrite(GPIO_OUT_ML_PH, 0x01);
	GPIOWrite(GPIO_OUT_ML_PWM, 0x01);
	GPIOWrite(GPIO_OUT_MR_PH, 0x01);
	GPIOWrite(GPIO_OUT_MR_PWM, 0x01);

	while(1) {

	}
}

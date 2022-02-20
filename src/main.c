#include "math.h"
#include "stdio.h"
#include "string.h"

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "pwm.h"
#include "rcc.h"
#include "rtc.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"

#include "pinMaping.h"

#include "batteryManager.h"
#include "collisionSensorBoard.h"
#include "encoder.h"
#include "motorDrive.h"
#include "robotMsgs.h"

int main(void) {
	//Configure the system clock
	SystemClockInit();
	SystemTickInit();
	RTCInit();

	//Initialize all configured peripherals
	GPIOInit();
	UART1Init(115200);	//For Bluetooth: 115200: FTDI: 962100
//	I2C2Init(I2CMode_SM);
	SPI2Init();
	ADC1Init();
	ADC2Init();
	PWM4Init();
	TIM3Init();

	//Initialize all "devices"/drivers
	EncoderInit();
	CollisionSensorInit();
	MotorDriveInit();
	BatteryManagerInit();
	RobotMessageInit();

	//Start ADC
	ADC1Start();

	//Set Time
	RTCSetTime(14, 25, 00);

	//Set LED ON
	GPIOWrite(GPIO_OUT_LED, 0x01);

	//Set Drive Speed (Twist Command)
	Vector3_q31 linear = {.x = 0, .y = 0, .z = 0};
	Vector3_q31 angular = {.x = 0, .y = 0, .z = 0};
	MotorDriveTwist(linear, angular);

	uint32_t timestamp = GetSysTick();
	while(1) {
		//Update Calls
		EncoderUpdate();
		MotorDriveUpdate();
		BatteryManagerUpdate();
		CollisionSensorUpdate();
		RobotMessageUpdate();
	}
}

//void EXTI8Callback() {
//	GPIOWrite(GPIO_OUT_LED, 0x00);
//	GPIOWrite(GPIO_OUT_M_EN, 0x00);		//Disable Motor Drive
//}
//
//void EXTI11Callback() {
//	GPIOWrite(GPIO_OUT_LED, 0x00);
//	GPIOWrite(GPIO_OUT_M_EN, 0x00);		//Disable Motor Drive
//}

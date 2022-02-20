#include "encoder.h"

#include "pinMaping.h"

#define IIR_LPF_ALPHA					8028
#define IIR_LPF_BETA_0					16712
#define IIR_LPF_BETA_1					16712

#define WHEEL_ENCODER_TH_DOWN			1300
#define WHEEL_ENCODER_TH_UP				1800

volatile uint8_t wheelEncL;
volatile uint8_t wheelEncR;
volatile uint16_t wheelEncDTL;
volatile uint16_t wheelEncDTR;
volatile uint32_t wheelEncTimL;
volatile uint32_t wheelEncTimR;

volatile uint32_t motorEncCntL;
volatile uint32_t motorEncCntR;
volatile uint32_t motorEncRPSL;
volatile uint32_t motorEncRPSR;

/**
  * @brief	This function initializes the Motor Encoder System
  * @param	None
  * @return	None
  */
void EncoderInit() {
	motorEncCntR = 0;
	motorEncCntL = 0;
}

/**
  * @brief	Update call for the Motor Encoder System, gets the magnetic wheel encoder values
  * @param	None
  * @return	None
  */
uint32_t timestampEnc = 0;
void EncoderUpdate() {
	if((timestampEnc + 10) < GetSysTick()) {
		uint16_t adcL = ADC1Read(1);
		uint16_t adcR = ADC1Read(0);

		if(wheelEncL == 0x01 && adcL < WHEEL_ENCODER_TH_DOWN) {
			//Wheel Encoder (HAL sensor) falling edge detection
			wheelEncDTL = GetSysTick() - wheelEncTimL;
			wheelEncTimL = GetSysTick();

			wheelEncL = 0x00;
		}
		else if(wheelEncL == 0x00 && adcL > WHEEL_ENCODER_TH_UP) {
			//Wheel Encoder (HAL sensor) rising edge detection
			wheelEncL = 0x01;
		}

		if(wheelEncR == 0x01 && adcR < WHEEL_ENCODER_TH_DOWN) {
			//Wheel Encoder (HAL sensor) falling edge detection
			wheelEncDTR = GetSysTick() - wheelEncTimR;
			wheelEncTimR = GetSysTick();

			wheelEncR = 0x00;
		}
		else if(wheelEncR == 0x00 && adcR > WHEEL_ENCODER_TH_UP) {
			//Wheel Encoder (HAL sensor) rising edge detection
			wheelEncR = 0x01;
		}

		timestampEnc = GetSysTick();
	}
}

/**
  * @brief	This function gets the motor encoder measurements in RPS
  * @param	rpsLeft: pointer to write left motor encoder speed to (in rotations per second (RPS))
  * @param	rpsLeft: pointer to write right motor encoder speed to (in rotations per second (RPS))
  * @return	None
  */
void EncoderGetRPS(uint32_t* rpsLeft, uint32_t* rpsRight) {
	*rpsLeft = motorEncRPSL;
	*rpsRight = motorEncRPSR;
}

/**
  * @brief	This function gets the motor encoder measurements in RPM
  * @param	rpsLeft: pointer to write left motor encoder speed to (in rotations per minute (RPM))
  * @param	rpsLeft: pointer to write right motor encoder speed to (in rotations per minute (RPM))
  * @return	None
  */
void EncoderGetRPM(uint32_t* rpmLeft, uint32_t* rpmRight) {
	*rpmLeft = (motorEncRPSL * 60);
	*rpmRight = (motorEncRPSR * 60);
}

/**
  * @brief	This function gets the wheel encoder delta time in ms, time between two triggers
  * @param	wheelLeft: pointer to write left wheel encoder delta time to (in ms)
  * @param	wheelRright: pointer to write right wheel encoder delta time to (in ms)
  * @return	None
  */
void EncoderWheelGetdT(uint16_t* wheelLeft, uint16_t* wheelRright) {
	*wheelLeft = wheelEncDTL;
	*wheelRright = wheelEncDTR;
}

/**
  * @brief	TIM3 Interrupt service routine, used to calculate the motor RPS/RPM
  * @param	None
  * @return	None
  */
uint32_t motorEncCntR_prev;
uint32_t motorEncCntL_prev;
void TIM3UpdateCallback() {
	//Low pass filtered RPS value (fc = 0.1*fs)
//	motorEncRPSR = ((motorEncCntR * 10) * IIR_LPF_BETA_0 + (motorEncCntR_prev * 10) * IIR_LPF_BETA_1 + motorEncRPSR * IIR_LPF_ALPHA) >> 15;
//	motorEncRPSL = ((motorEncCntL * 10) * IIR_LPF_BETA_0 + (motorEncCntL_prev * 10) * IIR_LPF_BETA_1 + motorEncRPSL * IIR_LPF_ALPHA) >> 15;

	//Timer is called every 100ms (10Hz), and encoder gives 2 pulses per rotation (for each side of the plastic leaf)
	motorEncRPSR = ((motorEncCntR * 10) >> 1);
	motorEncRPSL = ((motorEncCntL * 10) >> 1);

	motorEncCntR_prev = motorEncCntR;
	motorEncCntL_prev = motorEncCntL;
	motorEncCntR = 0;
	motorEncCntL = 0;
}

/**
  * @brief	EXTI7 Interrupt service routine, left motor encoder trigger interrupt
  * @param	None
  * @return	None
  */
void EXTI7Callback() {
	motorEncCntL += 1;
}

/**
  * @brief	EXTI15 Interrupt service routine, right motor encoder trigger interrupt
  * @param	None
  * @return	None
  */
void EXTI15Callback() {
	motorEncCntR += 1;
}

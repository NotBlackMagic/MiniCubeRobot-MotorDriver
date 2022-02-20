#ifndef DRIVER_ENCODER_H_
#define DRIVER_ENCODER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "adc.h"
#include "gpio.h"
#include "timer.h"
#include "rcc.h"

void EncoderInit();
void EncoderUpdate();
void EncoderGetRPS(uint32_t* rpsLeft, uint32_t* rpsRight);
void EncoderGetRPM(uint32_t* rpmLeft, uint32_t* rpmRight);
void EncoderWheelGetdT(uint16_t* wheelLeft, uint16_t* wheelRright);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_ENCODER_H_ */

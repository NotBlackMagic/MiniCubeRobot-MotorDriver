#ifndef DRIVER_MOTORDRIVE_H_
#define DRIVER_MOTORDRIVE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "math.h"

#include "dspPIDController.h"

#include "adc.h"
#include "gpio.h"
#include "pwm.h"
#include "rcc.h"
#include "timer.h"

#include "pinMaping.h"

#include "encoder.h"

#include "commonTypes.h"

#define MOTOR_PLANETARY_GEAR_RATIO			171		//Gear Ratio 1:171
#define MOTOR_DRIVE_GEAR					11		//Motor Gear 11 teeth
#define MOTOR_WHEEL_GEAR					24		//Wheel Gear 24 teeth
#define WHEEL_CIRCUMFERENCE					89		//Wheel circumference in mm (88.6 mm)
#define WHEEL_SPACING						43		//Wheel Spacing in mm

#define MAX_WHEEL_SPEED						70		//Limit wheel speed to this value in mm/s

void MotorDriveInit();
void MotorDriveUpdate();
void MotorDriveTwist(Vector3_q31 linear, Vector3_q31 angular);
void MotorDriveGetWheelVelocity(int32_t* wheelLeft, int32_t* wheelRight);
void MotorDriveGetVelocity(Vector3_q31* linear, Vector3_q31* angular);
void MotorOdomGetVelocity(Vector3_q31* linear, Vector3_q31* angular);
void MotorOdomGetPose(Vector3_q31* pose, Vector3_q31* orientation);
void MotorDriveGetPWM(int16_t* pwmLeft, int16_t* pwmRight);
void MotorDriveGetCurrent(uint16_t* currentLeft, uint16_t* currentRight);
void MotorDriveGetPID(int32_t* pLeft, int32_t* iLeft, int32_t* dLeft, int32_t* pRight, int32_t* iRight, int32_t* dRight);
uint8_t MotorDriveGetFault();
void MotorDriveDisable();
void MotorDriveEnable();

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_MOTORDRIVE_H_ */

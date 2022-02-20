#ifndef DSPPIDCONTROLLER_H_
#define DSPPIDCONTROLLER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef struct {
	int16_t kp;
	int16_t ki;
	int16_t kd;
	uint8_t pShift;
	int32_t windupLimit;
	int16_t prevError;
	int32_t pid_i;

	//Internal PID variables, provided to facilitate PID tuning/debugging
	int32_t p;
	int32_t i;
	int32_t d;
} DSPPIDCtrlInstQ15;

typedef struct {
	float kp;
	float ki;
	float kd;
	float windupLimit;
	float prevError;
	float pid_i;
} DSPPIDCtrlInstFloat;

void DSPPIDCtrlInitQ15(DSPPIDCtrlInstQ15* instance, int16_t kp, int16_t ki, int16_t kd, uint8_t shift);
int16_t DSPPIDCtrlQ15(DSPPIDCtrlInstQ15* instance, int16_t error);

void DSPPIDCtrlInitFloat(DSPPIDCtrlInstFloat* instance, float kp, float ki, float kd);
float DSPPIDCtrlFloat(DSPPIDCtrlInstFloat* pidStruct, float error);

#ifdef __cplusplus
}
#endif

#endif /* DSPPIDCONTROLLER_H_ */

#include "dspPIDController.h"

/**
  * @brief	Initialization function for the Q15 PID Controller
  * @param[in]	instance: Pointer to Q15 PID Structure
  * @param[in]	kp: Proportional gain
  * @param[in]	ki: Integral gain
  * @param[in]	kd: Derivative gain
  * @param[in]	shift: Shift (left) to be applied to output/sum
  * @return	None
  */
void DSPPIDCtrlInitQ15(DSPPIDCtrlInstQ15* instance, int16_t kp, int16_t ki, int16_t kd, uint8_t shift) {
	instance->kp = kp;
	instance->ki = ki;
	instance->kd = kd;
	instance->pShift = shift;

	instance->windupLimit = (INT16_MAX << instance->pShift);

	//Initialize state variables
	instance->prevError = 0;
	instance->pid_i = 0;
}

/**
  * @brief	Processing function for the Q15 PID Controller
  * @param[in]	instance: Pointer to Q15 PID Structure
  * @param[in]	error: New error value
  * @return	New control value
  */
int16_t DSPPIDCtrlQ15(DSPPIDCtrlInstQ15* instance, int16_t error) {
	//Calculate Proportional (P) part
	instance->p = (instance->kp * error);

	//Calculate Integral (I) part
	instance->i = instance->pid_i + (error * instance->ki);

	//Integral windup limiter
	if(instance->i > instance->windupLimit) {
		instance->i = instance->windupLimit;
	}
	else if(instance->i < -instance->windupLimit) {
		instance->i = -instance->windupLimit;
	}
	instance->pid_i = instance->i;

	//Calculate Derivative (D) part
	instance->d = (error - instance->prevError) * instance->kd;
	instance->prevError = error;

	//Sum
	int32_t sum = (instance->p + instance->i + instance->d) >> instance->pShift;
	return sum;
}

/**
  * @brief	Initialization function for the Float PID Controller
  * @param[in]	instance: Pointer to Float PID Structure
  * @param[in]	kp: Proportional gain
  * @param[in]	ki: Integral gain
  * @param[in]	kd: Derivative gain
  * @param[in]	shift: Shift (left) to be applied to output/sum
  * @return	None
  */
void DSPPIDCtrlInitFloat(DSPPIDCtrlInstFloat* instance, float kp, float ki, float kd) {
	instance->kp = kp;
	instance->ki = ki;
	instance->kd = kd;
	instance->windupLimit = INT16_MAX;

	//Initialize state variables
	instance->prevError = 0;
	instance->pid_i = 0;
}

/**
  * @brief	Processing function for the Float PID Controller
  * @param[in]	instance: Pointer to Float PID Structure
  * @param[in]	error: New error value
  * @return	New control value
  */
float DSPPIDCtrlFloat(DSPPIDCtrlInstFloat* pidStruct, float error) {
	//Calculate P part
	float p = pidStruct->kp * error;

	//Calculate I part
	float i = pidStruct->pid_i + (error * pidStruct->ki);

	//Integral windup limiter
	if(i > pidStruct->windupLimit) {
		i = pidStruct->windupLimit;
	}
	else if(i < -pidStruct->windupLimit) {
		i = -pidStruct->windupLimit;
	}
	pidStruct->pid_i = i;

	//Calculate D part
	//D part filtering, using Fc = 0.1 fs (recommended 0.1 to 0.2) https://www.controlglobal.com/articles/2019/signal-filtering-why-and-how/
	float d = (error - pidStruct->prevError) * pidStruct->kd;
	pidStruct->prevError = error;

	//Sum
	float pid = (p + i + d);
	return pid;
}

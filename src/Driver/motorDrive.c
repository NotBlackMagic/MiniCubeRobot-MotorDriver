#include "motorDrive.h"

#define ANGLE_RAD_MAX				1686629713

int32_t setWheelVelocityLeft;		//In Q17.15 m/s
int32_t wheelVelocityLeft;			//In Q17.15 m/s
int32_t setWheelVelocityRight;		//In Q17.15 m/s
int32_t wheelVelocityRight;			//In Q17.15 m/s
int32_t wheelVelocityLeftError;		//In Q17.15 m/s
int32_t wheelVelocityRightError;	//In Q17.15 m/s

int16_t drivePID_kp = (16384 >> 5);		//0.5 in Q15
int16_t drivePID_ki = (19661 >> 5);		//12 * Ts in Q15
int16_t drivePID_kd = 0;				//In Q15

DSPPIDCtrlInstQ15 drivePIDLeft;
DSPPIDCtrlInstQ15 drivePIDRight;

int16_t drivePWMLeft = 0;
int16_t drivePWMRight = 0;

Vector3_q31 setRobotLinearSpeed;	//In Q17.15 m/s
Vector3_q31 setRobotAngularSpeed;	//In Q17.15 rad/s

Vector3_q31 odomRobotLinearSpeed;	//In Q17.15 m/s
Vector3_q31 odomRobotAngularSpeed;	//In Q17.15 rad/s

Vector3_q31 odomRobotPosePosition;		//In Q17.15 m
Vector3_q31 odomRobotPoseOrientation;	//In Q3.29 rad

/**
  * @brief	This function initializes the Motor Drive System
  * @param	None
  * @return	None
  */
void MotorDriveInit() {
	//Init variables
	setWheelVelocityLeft = 0;
	setWheelVelocityRight = 0;

	//Init Control variables
	setRobotLinearSpeed = (struct Vector3_q31) {.x = 0, .y = 0, .z = 0};
	setRobotAngularSpeed = (struct Vector3_q31) {.x = 0, .y = 0, .z = 0};

	//Init Odometry variables
	odomRobotLinearSpeed = (struct Vector3_q31) {.x = 0, .y = 0, .z = 0};
	odomRobotAngularSpeed = (struct Vector3_q31) {.x = 0, .y = 0, .z = 0};
	odomRobotPosePosition = (struct Vector3_q31) {.x = 0, .y = 0, .z = 0};
	odomRobotPoseOrientation = (struct Vector3_q31) {.x = 0, .y = 0, .z = 0};

	//Init PID Controllers
	DSPPIDCtrlInitQ15(&drivePIDLeft, drivePID_kp, drivePID_ki, drivePID_kd, 15);
	DSPPIDCtrlInitQ15(&drivePIDRight, drivePID_kp, drivePID_ki, drivePID_kd, 15);

	//Enable Motor Drive
	GPIOWrite(GPIO_OUT_M_EN, 0x01);
//	GPIOWrite(GPIO_OUT_M_REF, 0x01);	//Over-Current Limit

	//Init PWM and Phase/Direction
	GPIOWrite(GPIO_OUT_ML_PH, 0x01);
	PWM4Set(4, 0);
	GPIOWrite(GPIO_OUT_MR_PH, 0x00);
	PWM4Set(2, 0);

	//Enable PWM
	PWM4Enable();
}

/**
  * @brief	Update call for the Motor Drive System, performs the drive PID controller and odometry calculations
  * @param	None
  * @return	None
  */
uint32_t timestampMD = 0;
void MotorDriveUpdate() {
	//First Check for Motor Driver (STSPIN240) Fault
	if(GPIORead(GPIO_OUT_M_EN) == 0x01 && GPIORead(GPIO_IN_M_FAULT) == 0x00) {
		//Fault detected at the motor driver (is Enabled but EN/Fault pin is low)
	}

	//Run motor drive PID controller
	if((timestampMD + 50) < GetSysTick()) {
		//Get encoder values, in rotations per second (RPS)
		uint32_t motorLeftRPS, motorRightRPS;
		EncoderGetRPS(&motorLeftRPS, &motorRightRPS);

		//Convert encoder RPS to wheel velocity
		wheelVelocityLeft = (motorLeftRPS * MOTOR_DRIVE_GEAR * WHEEL_CIRCUMFERENCE) / (MOTOR_PLANETARY_GEAR_RATIO * MOTOR_WHEEL_GEAR);
		wheelVelocityRight = (motorRightRPS * MOTOR_DRIVE_GEAR * WHEEL_CIRCUMFERENCE) / (MOTOR_PLANETARY_GEAR_RATIO * MOTOR_WHEEL_GEAR);

		//Get Rotation Direction from Motor Driver Phase (STSPIN240)
		int8_t dirLeft = 0;
		if(GPIORead(GPIO_OUT_ML_PH) == 0x01) {
			dirLeft = 1;
		}
		else {
			dirLeft = -1;
		}

		int8_t dirRight = 0;
		if(GPIORead(GPIO_OUT_MR_PH) == 0x01) {
			dirRight = -1;
		}
		else {
			dirRight = 1;
		}

		wheelVelocityLeft *= dirLeft;
		wheelVelocityRight *= dirRight;

		//Motor Speed Controller

		//Calculate motor wheel velocity error
		wheelVelocityLeftError = setWheelVelocityLeft - wheelVelocityLeft;
		wheelVelocityRightError = setWheelVelocityRight - wheelVelocityRight;

		//Motor + Wheel (Drive) PID Controller
		drivePWMLeft = DSPPIDCtrlQ15(&drivePIDLeft, wheelVelocityLeftError);
		drivePWMRight = DSPPIDCtrlQ15(&drivePIDRight, wheelVelocityRightError);

		//Limit PWM Range
		if(drivePWMLeft > 255) {
			drivePWMLeft = 255;
		}
		else if(drivePWMLeft < -255) {
			drivePWMLeft = -255;
		}
//		else if(drivePWMLeft < 32 && drivePWMLeft > -32) {
//			drivePWMLeft = 0;
//		}

		if(drivePWMRight > 255) {
			drivePWMRight = 255;
		}
		else if(drivePWMRight < -255) {
			drivePWMRight = -255;
		}
//		else if(drivePWMRight < 32 && drivePWMRight > -32) {
//			drivePWMRight = 0;
//		}

		//Apply PWM to motor drive, with correct direction/phase
		if(drivePWMLeft < 0) {
			GPIOWrite(GPIO_OUT_ML_PH, 0x00);
			PWM4Set(4, -drivePWMLeft);
		}
		else {
			GPIOWrite(GPIO_OUT_ML_PH, 0x01);
			PWM4Set(4, drivePWMLeft);
		}

		if(drivePWMRight < 0) {
			GPIOWrite(GPIO_OUT_MR_PH, 0x01);
			PWM4Set(2, -drivePWMRight);
		}
		else {
			GPIOWrite(GPIO_OUT_MR_PH, 0x00);
			PWM4Set(2, drivePWMRight);
		}

		//Calculate odometry data (current odometry pose estimation)
		//https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/

		//Compute velocities in the robot frame
		setRobotLinearSpeed.y = ((wheelVelocityRight + wheelVelocityLeft) >> 1); //Y (forward velocity in Q17.15 m/s)
		setRobotLinearSpeed.x = 0; //X (side velocity in mm/s)
		setRobotAngularSpeed.z = (((wheelVelocityLeft - wheelVelocityRight) << 16) / WHEEL_SPACING);	//Angular velocity in Q17.15 rad/s

		//Transform velocities into the odometry (global) frame
		float th = odomRobotPoseOrientation.z * 1.0f/536870912.0f;		//Convert from Q3.29 rad to float rad
		float sinZ = sinf(th);
		float cosZ = cosf(th);
		odomRobotLinearSpeed.y = setRobotLinearSpeed.y * cosZ + setRobotLinearSpeed.x * sinZ;
		odomRobotLinearSpeed.x = setRobotLinearSpeed.y * sinZ - setRobotLinearSpeed.x * cosZ;
		odomRobotAngularSpeed.z = setRobotAngularSpeed.z;

		//Position = Velocity * dTime, dTime = 50ms -> 0.05s
		odomRobotPosePosition.x += (odomRobotLinearSpeed.x * 1638) >> 15;
		odomRobotPosePosition.y += (odomRobotLinearSpeed.y * 1638) >> 15;
		odomRobotPoseOrientation.z += (odomRobotAngularSpeed.z * 1638) >> 2;

		//Handle angle wrap around
		if(odomRobotPoseOrientation.z > ANGLE_RAD_MAX) {
			odomRobotPoseOrientation.z = -ANGLE_RAD_MAX + (odomRobotPoseOrientation.z - ANGLE_RAD_MAX);
		}
		else if(odomRobotPoseOrientation.z < -ANGLE_RAD_MAX) {
			odomRobotPoseOrientation.z = ANGLE_RAD_MAX + (odomRobotPoseOrientation.z + ANGLE_RAD_MAX);
		}

		timestampMD = GetSysTick();
	}
}

/**
  * @brief	This function sets the Twist motion of the robot
  * @param	linear: linear velocities (x, y, z) in Q17.15 m/s
  * @param	angular: Angular velocities (x, y, z) in Q17.15 rad/s
  * @return	None
  */
void MotorDriveTwist(Vector3_q31 linear, Vector3_q31 angular) {
	//Convert twist parameters to wheel velocities (setVelocityLeft and setVelocityRight)
	setWheelVelocityRight = (((int32_t)angular.z * WHEEL_SPACING) >> 16) + (int32_t)linear.x;
	setWheelVelocityLeft = ((int32_t)linear.x * 2) - setWheelVelocityRight;

	//Limit max wheel speed
	if(setWheelVelocityLeft > MAX_WHEEL_SPEED) {
		setWheelVelocityLeft = MAX_WHEEL_SPEED;
	}
	else if(setWheelVelocityLeft < -MAX_WHEEL_SPEED) {
		setWheelVelocityLeft = -MAX_WHEEL_SPEED;
	}

	if(setWheelVelocityRight > MAX_WHEEL_SPEED) {
		setWheelVelocityRight = MAX_WHEEL_SPEED;
	}
	else if(setWheelVelocityRight < -MAX_WHEEL_SPEED) {
		setWheelVelocityRight = -MAX_WHEEL_SPEED;
	}
}

/**
  * @brief	This function gets the wheel speeds, in Q17.15 m/s, taken into account gear ratios and wheel circumference
  * @param	wheelLeft: pointer to write left wheel speed to (in Q17.15 m/s)
  * @param	wheelRight: pointer to write right wheel speed to (in Q17.15 m/s)
  * @return	None
  */
void MotorDriveGetWheelVelocity(int32_t* wheelLeft, int32_t* wheelRight) {
	*wheelLeft = wheelVelocityRight;
	*wheelRight = wheelVelocityLeft;
}

/**
  * @brief	This function gets the robot velocities/speeds in robot (local) coordinate frame
  * @param	linear: pointer to write the linear speeds (x, y, z) to (in Q17.15 m/s)
  * @param	angular: pointer to write the linear speeds (x, y, z) to (in Q17.15 rad/s)
  * @return	None
  */
void MotorDriveGetVelocity(Vector3_q31* linear, Vector3_q31* angular){
	*linear = setRobotLinearSpeed;
	*angular = setRobotAngularSpeed;
}

/**
  * @brief	This function gets the robot velocities/speeds in the odometry (global) coordinate frame
  * @param	linear: pointer to write the linear speeds (x, y, z) to (in Q17.15 m/s)
  * @param	angular: pointer to write the linear speeds (x, y, z) to (in Q17.15 rad/s)
  * @return	None
  */
void MotorOdomGetVelocity(Vector3_q31* linear, Vector3_q31* angular) {
	*linear = odomRobotLinearSpeed;
	*angular = odomRobotAngularSpeed;
}

/**
  * @brief	This function gets the robot position and orientation/rotation in the odometry (global) coordinate frame
  * @param	pose: pointer to write the position (x, y, z) to (in Q17.15 m)
  * @param	orientation: pointer to write the orientation/rotation (x, y, z) to (in Q3.29 rad/s)
  * @return	None
  */
void MotorOdomGetPose(Vector3_q31* pose, Vector3_q31* orientation) {
	*pose = odomRobotPosePosition;
	*orientation = odomRobotPoseOrientation;
}

/**
  * @brief	This function gets the motor PWM value
  * @param	pwmLeft: pointer to write left motor PWM value to (in 0-255)
  * @param	pwmRight: pointer to write right motor PWM value to (in 0-255)
  * @return	None
  */
void MotorDriveGetPWM(int16_t* pwmLeft, int16_t* pwmRight) {
	*pwmLeft = drivePWMLeft;
	*pwmRight = drivePWMRight;
}

/**
  * @brief	This function gets the motor current values
  * @param	currentLeft: pointer to write left motor current value to (in mA)
  * @param	currentRight: pointer to write right motor current value to (in mA)
  * @return	None
  */
void MotorDriveGetCurrent(uint16_t* currentLeft, uint16_t* currentRight) {
	//Voltage to current conversion (INA180 A3, 100 V/V): I = V * 1 / Rsense * 1 / 100 (Rsense = 0.0511) => I = V * 5.11 (A)
	*currentLeft = (ADC1Read(0x09) * 646) >> 12;	//In mA (3300 / 5.11 = 645.79)
	*currentRight = (ADC1Read(0x08) * 646) >> 12;	//In mA
}

/**
  * @brief	This function gets the internal drive PID state, used for tuning and debugging
  * @param	pLeft: PID left proportional value
  * @param	iLeft: PID left integral value
  * @param	dLeft: PID left derivative value
  * @param	pRight: PID right proportional value
  * @param	iRight: PID right integral value
  * @param	dRight: PID right derivative value
  * @return	0: No Fault, 1: Fault
  */
void MotorDriveGetPID(int32_t* pLeft, int32_t* iLeft, int32_t* dLeft, int32_t* pRight, int32_t* iRight, int32_t* dRight) {
	*pLeft = drivePIDLeft.p;
	*iLeft = drivePIDLeft.i;
	*dLeft = drivePIDLeft.d;
	*pRight = drivePIDRight.p;
	*iRight = drivePIDRight.i;
	*dRight = drivePIDRight.d;
}

/**
  * @brief	This function gets the motor driver fault status
  * @param	None
  * @return	0: No Fault, 1: Fault
  */
uint8_t MotorDriveGetFault() {
	return GPIORead(GPIO_IN_M_FAULT);
}

/**
  * @brief	This function disables the motor driver
  * @param	None
  * @return	None
  */
void MotorDriveDisable() {
	GPIOWrite(GPIO_OUT_M_EN, 0x00);
}

/**
  * @brief	This function enables the motor driver
  * @param	None
  * @return	None
  */
void MotorDriveEnable() {
	GPIOWrite(GPIO_OUT_M_EN, 0x01);
}

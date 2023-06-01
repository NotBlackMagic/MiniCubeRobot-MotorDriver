#ifndef URMS_URMSODOMETRY_H_
#define URMS_URMSODOMETRY_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotOdometryTopic {
	Vector3_q31 velocityLinear;		//In Q17.15 m/s
	Vector3_q31 velocityAngular;	//In Q17.15 rad/s
	Vector3_q31 posePoint;			//In Q17.15 m
	Vector3_q31 poseOrientation;	//In Q3.29 rad
} RobotOdometryTopic;

uint16_t RobotSerialize_RobotOdometryTopic(RobotOdometryTopic src, uint8_t* dst);
uint16_t RobotDeserialize_RobotOdometryTopic(uint8_t* src, RobotOdometryTopic* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSODOMETRY_H_ */

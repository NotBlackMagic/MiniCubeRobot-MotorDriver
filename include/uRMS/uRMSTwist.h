#ifndef URMS_URMSTWIST_H_
#define URMS_URMSTWIST_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotTwistTopic {
	Vector3_q31 velocityLinear;		//In Q17.15 m/s
	Vector3_q31 velocityAngular;	//In Q17.15 rad/s
} RobotTwistTopic;

uint16_t RobotSerialize_RobotTwistTopic(RobotTwistTopic src, uint8_t* dst);
uint16_t RobotDeserialize_RobotTwistTopic(uint8_t* src, RobotTwistTopic* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSTWIST_H_ */

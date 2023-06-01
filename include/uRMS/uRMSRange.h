#ifndef URMS_URMSRANGE_H_
#define URMS_URMSRANGE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotRangeTopic {
	uint8_t count;
	uint16_t ranges[6];
} RobotRangeTopic;

uint16_t RobotSerialize_RobotRangeTopic(RobotRangeTopic src, uint8_t* dst);
uint16_t RobotDeserialize_RobotRangeTopic(uint8_t* src, RobotRangeTopic* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSRANGE_H_ */

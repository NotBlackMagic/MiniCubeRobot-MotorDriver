#ifndef URMS_URMSCONTACT_H_
#define URMS_URMSCONTACT_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotContactTopic {
	uint8_t count;			//Contact array length
	uint8_t contact[4];		//0: No contact; 1: Contact
} RobotContactTopic;

uint16_t RobotSerialize_RobotContactTopic(RobotContactTopic src, uint8_t* dst);
uint16_t RobotDeserialize_RobotContactTopic(uint8_t* src, RobotContactTopic* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSCONTACT_H_ */

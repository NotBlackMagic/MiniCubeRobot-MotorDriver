#ifndef URMS_URMSDRIVE_H_
#define URMS_URMSDRIVE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotDriveTopic {
	int32_t wheelRPMLeft;
	int32_t wheelRPMRight;
	uint16_t motorCurrentLeft;
	uint16_t motorCurrentRight;
	uint8_t motorDriveStatus;
} RobotDriveTopic;

uint16_t RobotSerialize_RobotDriveTopic(RobotDriveTopic src, uint8_t* dst);
uint16_t RobotDeserialize_RobotDriveTopic(uint8_t* src, RobotDriveTopic* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSDRIVE_H_ */

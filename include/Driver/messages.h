#ifndef MESSAGES_H_
#define MESSAGES_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uart.h"

#include "batteryManager.h"
#include "collisionSensorBoard.h"
#include "encoder.h"
#include "motorDrive.h"

#include "robotMsgs_defines.h"

#include "uRMSMessage.h"

#include "uRMSBattery.h"
#include "uRMSContact.h"
#include "uRMSDrive.h"
#include "uRMSLaserScan.h"
#include "uRMSOdometry.h"
#include "uRMSRange.h"
#include "uRMSTwist.h"

typedef struct RobotMessageStruct{
	RobotMsgStruct packet;
	uint32_t timestamp;
	uint16_t packetRate;
} RobotMessageStruct;

void RobotMessageInit();
void RobotMessageUpdate();

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_H_ */

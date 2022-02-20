#ifndef ROBOTMSGS_H_
#define ROBOTMSGS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uart.h"

#include "batteryManager.h"
#include "collisionSensorBoard.h"
#include "encoder.h"
#include "motorDrive.h"

#include "robotMsgs_defines.h"

#include "commonTypes.h"

typedef struct RobotDataPacket{
	uint8_t src;
	uint8_t msgID;
	uint8_t payloadLength;
	uint8_t* payload;
	uint16_t crc;
} RobotDataPacket;

typedef struct RobotMessageStruct{
	RobotDataPacket packet;
	uint32_t timestamp;
	uint16_t packetRate;
} RobotMessageStruct;

void RobotMessageInit();
void RobotMessageUpdate();
uint8_t RobotMessageEncode(RobotDataPacket packet, uint8_t* data, uint8_t* dataLength);
uint8_t RobotMessageDecode(RobotDataPacket* packet, uint8_t* data, uint8_t dataLength);

#ifdef __cplusplus
}
#endif

#endif /* ROBOTMSGS_H_ */

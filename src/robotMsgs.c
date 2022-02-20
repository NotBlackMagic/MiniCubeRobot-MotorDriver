#include "robotMsgs.h"

RobotMessageStruct batteryInfoMsg;
RobotMessageStruct odometryInfoMsg;
RobotMessageStruct rangeInfoMsg;
RobotMessageStruct contactInfoMsg;
RobotMessageStruct driveInfoMsg;

RobotMessageStruct driveDebugMsg;

/**
  * @brief	This function encodes/converts a uint16 into a byte array
  * @param	src: uint16 to encode
  * @param	dst: pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint8_t RobotToArray_uint16(uint16_t src, uint8_t* dst) {
	uint8_t i = 0;
	dst[i++] = (uint8_t)(src >> 8);
	dst[i++] = (uint8_t)(src);
	return i;
}

/**
  * @brief	This function decodes/converts a byte array to a uint16
  * @param	src: pointer to byte array to decode
  * @param	dst: pointer to uint16 to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint8_t RobotFromArray_uint16(uint8_t* src, uint16_t* dst) {
	uint8_t i = 0;
	*dst = (src[i++] << 8);
	*dst += (src[i++]);
	return i;
}

/**
  * @brief	This function encodes/converts a int32 into a byte array
  * @param	src: int32 to encode
  * @param	dst: pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint8_t RobotToArray_int32(int32_t src, uint8_t* dst) {
	uint8_t i = 0;
	dst[i++] = (uint8_t)(src >> 24);
	dst[i++] = (uint8_t)(src >> 16);
	dst[i++] = (uint8_t)(src >> 8);
	dst[i++] = (uint8_t)(src);
	return i;
}

/**
  * @brief	This function decodes/converts a byte array to a int32
  * @param	src: pointer to byte array to decode
  * @param	dst: pointer to int32 to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint8_t RobotFromArray_int32(uint8_t* src, int32_t* dst) {
	uint8_t i = 0;
	*dst = (src[i++] << 24);
	*dst += (src[i++] << 16);
	*dst += (src[i++] << 8);
	*dst += (src[i++]);
	return i;
}

/**
  * @brief	This function encodes/converts a Vector3_q31 into a byte array
  * @param	src: Vector3_q31 to encode
  * @param	dst: pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint8_t RobotToArray_Vector3(Vector3_q31 src, uint8_t* dst) {
	uint8_t i = 0;
	i += RobotToArray_int32(src.x, &dst[i]);
	i += RobotToArray_int32(src.y, &dst[i]);
	i += RobotToArray_int32(src.z, &dst[i]);
	return i;
}

/**
  * @brief	This function decodes/converts a byte array to a Vector3_q31
  * @param	src: pointer to byte array to decode
  * @param	dst: pointer to Vector3_q31 to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint8_t RobotFromArray_Vector3(uint8_t* src, Vector3_q31* dst) {
	uint8_t i = 0;
	i += RobotFromArray_int32(&src[i], &dst->x);
	i += RobotFromArray_int32(&src[i], &dst->y);
	i += RobotFromArray_int32(&src[i], &dst->z);
	return i;
}

/**
  * @brief	This function initializes the Robot Message System
  * @param	None
  * @return	None
  */
void RobotMessageInit() {
	//Init Battery Info Message Struct
	batteryInfoMsg = (struct RobotMessageStruct) {	.packet = {.src = 0x00, .msgID = ROBOT_MSGS_BATTERY, .payloadLength = 0, .crc = 0},
													.timestamp = 0,
													.packetRate = 1000};

	//Init Odometry Info Message Struct
	odometryInfoMsg = (struct RobotMessageStruct) {	.packet = {.src = 0x00, .msgID = ROBOT_MSGS_ODOM, .payloadLength = 0, .crc = 0},
													.timestamp = 0,
													.packetRate = 200};

	//Init Range Info Message Struct
	rangeInfoMsg = (struct RobotMessageStruct) {	.packet = {.src = 0x00, .msgID = ROBOT_MSGS_RANGE, .payloadLength = 0, .crc = 0},
													.timestamp = 0,
													.packetRate = 200};

	//Init Contact Info Message Struct
	contactInfoMsg = (struct RobotMessageStruct) {	.packet = {.src = 0x00, .msgID = ROBOT_MSGS_CONTACT, .payloadLength = 0, .crc = 0},
													.timestamp = 0,
													.packetRate = UINT16_MAX};

	//Init Drive Info Message Struct
	driveInfoMsg = (struct RobotMessageStruct) {	.packet = {.src = 0x00, .msgID = ROBOT_MSGS_DRIVE, .payloadLength = 0, .crc = 0},
													.timestamp = 0,
													.packetRate = 200};

	//Init Drive Debug Message Struct
	driveDebugMsg = (struct RobotMessageStruct) {	.packet = {.src = 0x00, .msgID = ROBOT_DBG_DRIVE, .payloadLength = 0, .crc = 0},
													.timestamp = 0,
													.packetRate = UINT16_MAX};
}

/**
  * @brief	Update call for the Robot Message System, performs message interpretation/execution and sends programmed messages
  * @param	None
  * @return	None
  */
uint16_t rxLength;
uint8_t rxData[512];
uint16_t txLength;
uint8_t txData[512];
void RobotMessageUpdate() {
	//Handle received messages
	if(UART1Read(rxData, &rxLength) == 0x01) {
		//New command received, decode byte array into packet
		uint8_t pktPayload[100];
		RobotDataPacket packet = {.payload = pktPayload};
		uint8_t error = RobotMessageDecode(&packet, rxData, (uint8_t)rxLength);

		if(error == 0x00) {
			switch(packet.msgID) {
				case ROBOT_CMD_REBOOT: {
					break;
				}
				case ROBOT_CMD_ABORT: {
					//Abort Command, stop all movement
					//First stop/disable motor drive
//					MotorDriveDisable();

					//Set Motor Drive velocities to 0
					MotorDriveTwist((Vector3_q31){(0,0,0)}, (Vector3_q31){(0,0,0)});

					//Re-enable stop motor drive
//					MotorDriveEnable();
					break;
				}
				case ROBOT_CMD_TWIST: {
					//Validate if payload length is as expected
					if(packet.payloadLength != 24) {
						break;
					}

					//Get twist information
					Vector3_q31 velocityLinear, velocityAngular;

					uint8_t dataIndex = 0;
					dataIndex += RobotFromArray_Vector3(&packet.payload[dataIndex], &velocityLinear);
					dataIndex += RobotFromArray_Vector3(&packet.payload[dataIndex], &velocityAngular);

					//Execute twist command
					MotorDriveTwist(velocityLinear, velocityAngular);

					break;
				}
				case ROBOT_CMD_TRANSFORM: {
					//Validate if payload length is as expected
					if(packet.payloadLength != 24) {
						break;
					}

					//Get transform information
					Vector3_q31 translation, rotation;

					uint8_t dataIndex = 0;
					dataIndex += RobotFromArray_Vector3(&packet.payload[dataIndex], &translation);
					dataIndex += RobotFromArray_Vector3(&packet.payload[dataIndex], &rotation);

					//Execute transform command
					break;
				}
			}
		}
	}

	//Handle Periodic messages transmissions
	if((batteryInfoMsg.timestamp + batteryInfoMsg.packetRate) < GetSysTick()) {
		//Send Battery Info Message
		uint8_t pktPayload[100];

		//Get Battery Info and assemble into packet payload
		uint16_t current, voltage, charge;
		uint8_t percentage, status;
		BatterManagerGetStatus(&current, &voltage, &charge, &percentage, &status);

		uint8_t dataIndex = 0;
		dataIndex += RobotToArray_uint16(voltage, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(current, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(charge, &pktPayload[dataIndex]);
		pktPayload[dataIndex++] = percentage;
		pktPayload[dataIndex++] = status;

		//Add payload length
		batteryInfoMsg.packet.payload = pktPayload;
		batteryInfoMsg.packet.payloadLength = dataIndex;

		//Encode packet into byte array
		uint8_t txLength;
		uint8_t txData[512];
		RobotMessageEncode(batteryInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		batteryInfoMsg.timestamp = GetSysTick();
	}

	if((odometryInfoMsg.timestamp + odometryInfoMsg.packetRate) < GetSysTick()) {
		//Send Odometry Info Message
		uint8_t pktPayload[100];

		//Get Odometry Info and assemble into packet payload
		Vector3_q31 velocityLinear, velocityAngular;
		MotorOdomGetVelocity(&velocityLinear, &velocityAngular);

		Vector3_q31 posePoint, poseOrientation;
		MotorOdomGetPose(&posePoint, &poseOrientation);

		uint8_t dataIndex = 0;
		dataIndex += RobotToArray_Vector3(velocityLinear, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_Vector3(velocityAngular, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_Vector3(posePoint, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_Vector3(poseOrientation, &pktPayload[dataIndex]);

		//Add payload length
		odometryInfoMsg.packet.payload = pktPayload;
		odometryInfoMsg.packet.payloadLength = dataIndex;

		//Encode packet into byte array
		uint8_t txLength;
		uint8_t txData[512];
		RobotMessageEncode(odometryInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		odometryInfoMsg.timestamp = GetSysTick();
	}

	if((rangeInfoMsg.timestamp + rangeInfoMsg.packetRate) < GetSysTick()) {
		//Send Range Info Message
		uint8_t pktPayload[100];

		//Get Range (IR Collision) Sensor Info and assemble into packet payload
		uint16_t ranges[6];
		uint8_t cnt = CollisionSensorGetRange(ranges);

		uint8_t dataIndex = 0;
		pktPayload[dataIndex++] = cnt;
		dataIndex += RobotToArray_uint16(ranges[0], &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(ranges[1], &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(ranges[2], &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(ranges[3], &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(ranges[4], &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(ranges[5], &pktPayload[dataIndex]);

		//Add payload length
		rangeInfoMsg.packet.payload = pktPayload;
		rangeInfoMsg.packet.payloadLength = dataIndex;

		//Encode packet into byte array
		uint8_t txLength;
		uint8_t txData[512];
		RobotMessageEncode(rangeInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		rangeInfoMsg.timestamp = GetSysTick();
	}

	if(contactInfoMsg.packetRate < UINT16_MAX && (contactInfoMsg.timestamp + contactInfoMsg.packetRate) < GetSysTick()) {
		//Send Contact Sensor Info Message
		uint8_t pktPayload[100];

		//Get Contact Sensor Info and assemble into packet payload
		uint8_t dataIndex = 0;
		pktPayload[dataIndex++] = !GPIORead(GPIO_IN_BTN_FL);
		pktPayload[dataIndex++] = !GPIORead(GPIO_IN_BTN_FR);
		pktPayload[dataIndex++] = !GPIORead(GPIO_IN_BTN_BL);
		pktPayload[dataIndex++] = !GPIORead(GPIO_IN_BTN_BR);

		//Add payload length
		contactInfoMsg.packet.payload = pktPayload;
		contactInfoMsg.packet.payloadLength = dataIndex;

		//Encode packet into byte array
		uint8_t txLength;
		uint8_t txData[512];
		RobotMessageEncode(contactInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		contactInfoMsg.timestamp = GetSysTick();
	}

	if((driveInfoMsg.timestamp + driveInfoMsg.packetRate) < GetSysTick()) {
		//Send Drive Info Message
		uint8_t pktPayload[100];

		//Get Drive, Motor and drive system, Info and assemble into packet payload
		int32_t wheelRPML, wheelRPMR;
		MotorDriveGetWheelVelocity(&wheelRPML, &wheelRPMR);

		uint16_t currentL, currentR;
		MotorDriveGetCurrent(&currentL, &currentR);

		uint8_t motorStatus = MotorDriveGetFault();

		uint8_t dataIndex = 0;
		dataIndex += RobotToArray_int32(wheelRPML, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_int32(wheelRPMR, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(currentL, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(currentR, &pktPayload[dataIndex]);
		pktPayload[dataIndex++] = motorStatus;

		//Add payload length
		driveInfoMsg.packet.payload = pktPayload;
		driveInfoMsg.packet.payloadLength = dataIndex;

		//Encode packet into byte array
		uint8_t txLength;
		uint8_t txData[512];
		RobotMessageEncode(driveInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		driveInfoMsg.timestamp = GetSysTick();
	}

	if((driveDebugMsg.timestamp + driveDebugMsg.packetRate) < GetSysTick()) {
		//Send Drive Debug Message
		uint8_t pktPayload[100];

		//Get Drive PID and PWM Information
		int32_t pLeft, iLeft, dLeft, pRight, iRight, dRight;
		MotorDriveGetPID(&pLeft, &iLeft, &dLeft, &pRight, &iRight, &dRight);

		int16_t pwmLeft, pwmRight;
		MotorDriveGetPWM(&pwmLeft, &pwmRight);

		uint8_t dataIndex = 0;
		dataIndex += RobotToArray_int32(pLeft, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_int32(iLeft, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_int32(dLeft, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_int32(pRight, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_int32(iRight, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_int32(dRight, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(pwmLeft, &pktPayload[dataIndex]);
		dataIndex += RobotToArray_uint16(pwmRight, &pktPayload[dataIndex]);

		//Add payload length
		driveDebugMsg.packet.payload = pktPayload;
		driveDebugMsg.packet.payloadLength = dataIndex;

		//Encode packet into byte array
		uint8_t txLength;
		uint8_t txData[512];
		RobotMessageEncode(driveDebugMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		driveDebugMsg.timestamp = GetSysTick();
	}
}

/**
  * @brief	This function encodes a robot message from the packet struct to a byte array
  * @param	packet: Robot message to encode
  * @param	data: Pointer, byte array, to write the encoded message to
  * @param	dataLength: Pointer to write the encoded message length to
  * @return	0: Success, 1: Error/fail
  */
uint8_t RobotMessageEncode(RobotDataPacket packet, uint8_t* data, uint8_t* dataLength) {
	//Assemble data packet into byte array
	//|  1 byte  |  1 byte  |  1 byte  | n bytes | 2 bytes |
	//|----------|----------|----------|---------|---------|
	//| Src ID   | Msg ID   | Pkt Len  | Data    | CRC     |

	//Add Header
	uint8_t dataIndex = 0;
	data[dataIndex++] = packet.src;
	data[dataIndex++] = packet.msgID;
	data[dataIndex++] = packet.payloadLength;

	//Add payload
	uint8_t i;
	for(i = 0; i < packet.payloadLength; i++) {
		data[dataIndex++] = packet.payload[i];
	}

	//Add CRC
	uint16_t crc = 0;
	data[dataIndex++] = (uint8_t)(crc >> 8);
	data[dataIndex++] = (uint8_t)(crc);

	*dataLength = dataIndex;

	return 0;
}

/**
  * @brief	This function decodes a robot message from a byte array to a packet struct
  * @param	packet: Pointer, packet, to write the decoded message to
  * @param	data: Byte array to decode into a robot message packet
  * @param	dataLength: Length of the byte array to decode
  * @return	0: Success, 1: Error/fail
  */
uint8_t RobotMessageDecode(RobotDataPacket* packet, uint8_t* data, uint8_t dataLength) {
	//De-assemble data packet into byte array
	//|  1 byte  |  1 byte  |  1 byte  | n bytes | 2 bytes |
	//|----------|----------|----------|---------|---------|
	//| Src ID   | Msg ID   | Pkt Len  | Data    | CRC     |

	//Get Header
	uint8_t dataIndex = 0;
	packet->src = data[dataIndex++];
	packet->msgID = data[dataIndex++];
	packet->payloadLength = data[dataIndex++];

	//Check if payload length can be valid/correct
	if((packet->payloadLength + 5) > dataLength) {
		return 0x01;
	}

	//Get payload
	uint8_t i;
	for(i = 0; i < packet->payloadLength; i++) {
		packet->payload[i] = data[dataIndex++];
	}

	//Get CRC and check if correct
	uint16_t pktCRC = (data[dataIndex++] << 8) + data[dataIndex++];
	uint16_t calcCRC = 0;
	if(pktCRC != calcCRC) {
		//CRC validation failed
		return 0x01;
	}

	return 0;
}

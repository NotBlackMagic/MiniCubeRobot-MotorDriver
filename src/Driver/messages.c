#include "messages.h"

RobotBatteryTopic batteryMessageTopic;
RobotMessageStruct batteryInfoMsg;

RobotOdometryTopic odometryMessageTopic;
RobotMessageStruct odometryInfoMsg;

RobotRangeTopic rangeMessageTopic;
RobotMessageStruct rangeInfoMsg;

RobotContactTopic contactMessageTopic;
RobotMessageStruct contactInfoMsg;

RobotDriveTopic driveMessageTopic;
RobotMessageStruct driveInfoMsg;

//RobotBatteryTopic driveDebugMessageTopic;
RobotMessageStruct driveDebugMsg;

/**
  * @brief	This function initializes the Robot Message System
  * @param	None
  * @return	None
  */
void RobotMessageInit() {
	//Init Battery Info Message Struct
	batteryInfoMsg = (struct RobotMessageStruct) {	.packet = {.frameID = 0x00, .nsec = 0, .topicID = RobotTopic_Battery, .payloadLength = 0},
													.timestamp = 0,
													.packetRate = 1000};

	//Init Odometry Info Message Struct
	odometryInfoMsg = (struct RobotMessageStruct) {	.packet = {.frameID = 0x00, .nsec = 0, .topicID = RobotTopic_Odom, .payloadLength = 0},
													.timestamp = 0,
													.packetRate = 200};

	//Init Range Info Message Struct
	rangeInfoMsg = (struct RobotMessageStruct) {	.packet = {.frameID = 0x00, .nsec = 0, .topicID = RobotTopic_Range, .payloadLength = 0},
													.timestamp = 0,
													.packetRate = 200};

	//Init Contact Info Message Struct
	contactInfoMsg = (struct RobotMessageStruct) {	.packet = {.frameID = 0x00, .nsec = 0, .topicID = RobotTopic_Contact, .payloadLength = 0},
													.timestamp = 0,
													.packetRate = UINT16_MAX};

	//Init Drive Info Message Struct
	driveInfoMsg = (struct RobotMessageStruct) {	.packet = {.frameID = 0x00, .nsec = 0, .topicID = RobotTopic_Drive, .payloadLength = 0},
													.timestamp = 0,
													.packetRate = 200};

	//Init Drive Debug Message Struct
//	driveDebugMsg = (struct RobotMessageStruct) {	.packet = {.frameID = 0x00, .nsec = 0, .topicID = ROBOT_DBG_DRIVE, .payloadLength = 0},
//													.timestamp = 0,
//													.packetRate = UINT16_MAX};
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
		RobotMsgStruct packet = {.payload = pktPayload};
		uint8_t error = RobotMessageDecode(&packet, rxData, rxLength);

		if(error == 0x00) {
			switch(packet.topicID) {
//				case ROBOT_CMD_REBOOT: {
//					break;
//				}
//				case ROBOT_CMD_ABORT: {
//					//Abort Command, stop all movement
//					//First stop/disable motor drive
////					MotorDriveDisable();
//
//					//Set Motor Drive velocities to 0
//					MotorDriveTwist((Vector3_q31){0,0,0}, (Vector3_q31){0,0,0});
//
//					//Re-enable stop motor drive
////					MotorDriveEnable();
//					break;
//				}
				case RobotTopic_Twist: {
					//Validate if payload length is as expected
					if(packet.payloadLength != 24) {
						break;
					}

					//Get twist information
					Vector3_q31 velocityLinear, velocityAngular;

					uint8_t dataIndex = 0;
					dataIndex += RobotDeserialize_Vector3(&packet.payload[dataIndex], &velocityLinear);
					dataIndex += RobotDeserialize_Vector3(&packet.payload[dataIndex], &velocityAngular);

					//Execute twist command
					MotorDriveTwist(velocityLinear, velocityAngular);

					break;
				}
//				case ROBOT_CMD_TRANSFORM: {
//					//Validate if payload length is as expected
//					if(packet.payloadLength != 24) {
//						break;
//					}
//
//					//Get transform information
//					Vector3_q31 translation, rotation;
//
//					uint8_t dataIndex = 0;
//					dataIndex += RobotDeserialize_Vector3(&packet.payload[dataIndex], &translation);
//					dataIndex += RobotDeserialize_Vector3(&packet.payload[dataIndex], &rotation);
//
//					//Execute transform command
//					break;
//				}
			}
		}
	}

	//Handle Periodic messages transmissions
	if((batteryInfoMsg.timestamp + batteryInfoMsg.packetRate) < GetSysTick()) {
		//Send Battery Info Message
		uint8_t pktPayload[100];

		//Get uptime in ms
		batteryInfoMsg.packet.nsec = GetSysTick();

		//Get Battery Info and assemble into packet payload
		BatterManagerGetStatus(&batteryMessageTopic.current, &batteryMessageTopic.voltage, &batteryMessageTopic.charge, &batteryMessageTopic.percentage, &batteryMessageTopic.status);
		batteryInfoMsg.packet.payloadLength = 0;
		batteryInfoMsg.packet.payload = pktPayload;
		batteryInfoMsg.packet.payloadLength += RobotSerialize_RobotBatteryTopic(batteryMessageTopic, batteryInfoMsg.packet.payload);

		//Encode packet into byte array
		RobotMessageEncode(batteryInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		batteryInfoMsg.timestamp = GetSysTick();
	}

	if((odometryInfoMsg.timestamp + odometryInfoMsg.packetRate) < GetSysTick()) {
		//Send Odometry Info Message
		uint8_t pktPayload[100];

		//Get uptime in ms
		odometryInfoMsg.packet.nsec = GetSysTick();

		//Get Odometry Info and assemble into packet payload
		MotorOdomGetVelocity(&odometryMessageTopic.velocityLinear, &odometryMessageTopic.velocityAngular);
		MotorOdomGetPose(&odometryMessageTopic.posePoint, &odometryMessageTopic.poseOrientation);
		odometryInfoMsg.packet.payloadLength = 0;
		odometryInfoMsg.packet.payload = pktPayload;
		odometryInfoMsg.packet.payloadLength += RobotSerialize_RobotOdometryTopic(odometryMessageTopic, odometryInfoMsg.packet.payload);

		//Encode packet into byte array
		RobotMessageEncode(odometryInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		odometryInfoMsg.timestamp = GetSysTick();
	}

	if((rangeInfoMsg.timestamp + rangeInfoMsg.packetRate) < GetSysTick()) {
		//Send Range Info Message
		uint8_t pktPayload[100];

		//Get uptime in ms
		rangeInfoMsg.packet.nsec = GetSysTick();

		//Get Range Info and assemble into packet payload
		rangeMessageTopic.count = CollisionSensorGetRange(rangeMessageTopic.ranges);
		rangeInfoMsg.packet.payloadLength = 0;
		rangeInfoMsg.packet.payload = pktPayload;
		rangeInfoMsg.packet.payloadLength += RobotSerialize_RobotRangeTopic(rangeMessageTopic, rangeInfoMsg.packet.payload);

		//Encode packet into byte array
		RobotMessageEncode(rangeInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		rangeInfoMsg.timestamp = GetSysTick();
	}

	if(contactInfoMsg.packetRate < UINT16_MAX && (contactInfoMsg.timestamp + contactInfoMsg.packetRate) < GetSysTick()) {
		//Send Contact Sensor Info Message
		uint8_t pktPayload[100];

		//Get uptime in ms
		contactInfoMsg.packet.nsec = GetSysTick();

		//Get Contact Info and assemble into packet payload
		contactMessageTopic.count = 4;
		contactMessageTopic.contact[0] = !GPIORead(GPIO_IN_BTN_FL);
		contactMessageTopic.contact[1] = !GPIORead(GPIO_IN_BTN_FR);
		contactMessageTopic.contact[2] = !GPIORead(GPIO_IN_BTN_BL);
		contactMessageTopic.contact[3] = !GPIORead(GPIO_IN_BTN_BR);
		contactInfoMsg.packet.payloadLength = 0;
		contactInfoMsg.packet.payload = pktPayload;
		contactInfoMsg.packet.payloadLength += RobotSerialize_RobotContactTopic(contactMessageTopic, contactInfoMsg.packet.payload);

		//Encode packet into byte array
		RobotMessageEncode(contactInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		contactInfoMsg.timestamp = GetSysTick();
	}

	if((driveInfoMsg.timestamp + driveInfoMsg.packetRate) < GetSysTick()) {
		//Send Drive Info Message
		uint8_t pktPayload[100];

		//Get uptime in ms
		driveInfoMsg.packet.nsec = GetSysTick();

		//Get Drive Info and assemble into packet payload
		MotorDriveGetWheelVelocity(&driveMessageTopic.wheelRPMLeft, &driveMessageTopic.wheelRPMRight);
		MotorDriveGetCurrent(&driveMessageTopic.motorCurrentLeft, &driveMessageTopic.motorCurrentRight);
		driveMessageTopic.motorDriveStatus = MotorDriveGetFault();
		driveInfoMsg.packet.payloadLength = 0;
		driveInfoMsg.packet.payload = pktPayload;
		driveInfoMsg.packet.payloadLength += RobotSerialize_RobotDriveTopic(driveMessageTopic, driveInfoMsg.packet.payload);

		//Encode packet into byte array
		RobotMessageEncode(driveInfoMsg.packet, txData, &txLength);

		//Write Data over UART
		UART1Write(txData, txLength);

		driveInfoMsg.timestamp = GetSysTick();
	}

//	if((driveDebugMsg.timestamp + driveDebugMsg.packetRate) < GetSysTick()) {
//		//Send Drive Debug Message
//		uint8_t pktPayload[100];
//
//		//Get Drive PID and PWM Information
//		int32_t pLeft, iLeft, dLeft, pRight, iRight, dRight;
//		MotorDriveGetPID(&pLeft, &iLeft, &dLeft, &pRight, &iRight, &dRight);
//
//		int16_t pwmLeft, pwmRight;
//		MotorDriveGetPWM(&pwmLeft, &pwmRight);
//
//		uint8_t dataIndex = 0;
//		dataIndex += RobotToArray_int32(pLeft, &pktPayload[dataIndex]);
//		dataIndex += RobotToArray_int32(iLeft, &pktPayload[dataIndex]);
//		dataIndex += RobotToArray_int32(dLeft, &pktPayload[dataIndex]);
//		dataIndex += RobotToArray_int32(pRight, &pktPayload[dataIndex]);
//		dataIndex += RobotToArray_int32(iRight, &pktPayload[dataIndex]);
//		dataIndex += RobotToArray_int32(dRight, &pktPayload[dataIndex]);
//		dataIndex += RobotToArray_uint16(pwmLeft, &pktPayload[dataIndex]);
//		dataIndex += RobotToArray_uint16(pwmRight, &pktPayload[dataIndex]);
//
//		//Add payload length
//		driveDebugMsg.packet.payload = pktPayload;
//		driveDebugMsg.packet.payloadLength = dataIndex;
//
//		//Encode packet into byte array
//		uint8_t txLength;
//		uint8_t txData[512];
//		RobotMessageEncode(driveDebugMsg.packet, txData, &txLength);
//
//		//Write Data over UART
//		UART1Write(txData, txLength);
//
//		driveDebugMsg.timestamp = GetSysTick();
//	}
}

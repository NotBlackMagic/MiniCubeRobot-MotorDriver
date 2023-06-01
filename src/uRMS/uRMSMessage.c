#include "uRMSMessage.h"

/**
  * @brief	This function encodes a robot message from the packet struct to a byte array
  * @param	msg: Robot message to encode
  * @param	data: Pointer, byte array, to write the encoded message to
  * @param	dataLength: Pointer to write the encoded message length to
  * @return	0: Success, 1: Error/fail
  */
uint8_t RobotMessageEncode(RobotMsgStruct msg, uint8_t* data, uint16_t* dataLength) {
	//Add Header
	uint16_t i = 0;
	i += RobotSerialize_uint16(msg.topicID, &data[i]);
	i += RobotSerialize_uint32(msg.nsec, &data[i]);
	i += RobotSerialize_uint16(msg.frameID, &data[i]);

	//Add payload
	memcpy(&data[i], msg.payload, msg.payloadLength);
	i = i + msg.payloadLength;

	*dataLength = i;

	return 0;
}

/**
  * @brief	This function decodes a robot message from a byte array to a msg struct
  * @param	packet: Pointer, msg, to write the decoded message to
  * @param	data: Byte array to decode into a robot message packet
  * @param	dataLength: Length of the byte array to decode
  * @return	0: Success, 1: Error/fail
  */
uint8_t RobotMessageDecode(RobotMsgStruct* msg, uint8_t* data, uint16_t dataLength) {
	//Get Header
	uint16_t i = 0;
	i += RobotDeserialize_uint16(&data[i], &msg->topicID);
	i += RobotDeserialize_uint32(&data[i], &msg->nsec);
	i += RobotDeserialize_uint16(&data[i], &msg->frameID);

	msg->payloadLength = dataLength - ROBOT_MSG_PKT_HEADER_SIZE;

	//Get payload
	memcpy(msg->payload, &data[i], msg->payloadLength);
	i = i + msg->payloadLength;

	return 0;
}


#include "uRMSContact.h"

/**
  * @brief	This function serializes a RobotContactTopic into a byte array
  * @param	src: RobotContactTopic to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotContactTopic(RobotContactTopic src, uint8_t* dst) {
	uint8_t i = 0;
	dst[i++] = src.count;
	uint8_t j;
	for(j = 0; j < src.count; j++) {
		dst[i++] = src.contact[j];
	}
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotContactTopic
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotContactTopic to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotContactTopic(uint8_t* src, RobotContactTopic* dst) {
	uint8_t i = 0;
	dst->count = src[i++];
	uint8_t j;
	for(j = 0; j < dst->count; j++) {
		dst->contact[j] = src[i++];
	}
	return i;
}

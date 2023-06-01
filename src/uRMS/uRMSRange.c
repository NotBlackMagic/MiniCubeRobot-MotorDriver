#include "uRMSRange.h"

/**
  * @brief	This function serializes a RobotRangeTopic into a byte array
  * @param	src: RobotRangeTopic to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotRangeTopic(RobotRangeTopic src, uint8_t* dst) {
	uint8_t i = 0;
	dst[i++] = src.count;
	uint8_t j;
	for(j = 0; j < src.count; j++) {
		i += RobotSerialize_uint16(src.ranges[j], &dst[i]);
	}
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotRangeTopic
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotRangeTopic to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotRangeTopic(uint8_t* src, RobotRangeTopic* dst) {
	uint8_t i = 0;
	dst->count = src[i++];
	uint8_t j;
	for(j = 0; j < dst->count; j++) {
		i += RobotDeserialize_uint16(&src[i], &dst->ranges[j]);
	}
	return i;
}

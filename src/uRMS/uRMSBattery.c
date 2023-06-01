#include "uRMSBattery.h"

/**
  * @brief	This function serializes a RobotBatteryTopic into a byte array
  * @param	src: RobotBatteryTopic to serialize
  * @param	dst: Pointer to byte array to write to
  * @return	Number of bytes written to byte array
  */
uint16_t RobotSerialize_RobotBatteryTopic(RobotBatteryTopic src, uint8_t* dst) {
	uint8_t i = 0;
	i += RobotSerialize_uint16(src.voltage, &dst[i]);
	i += RobotSerialize_uint16(src.current, &dst[i]);
	i += RobotSerialize_uint16(src.charge, &dst[i]);
	dst[i++] = src.percentage;
	dst[i++] = src.status;
	return i;
}

/**
  * @brief	This function de-serializess a byte array to a RobotBatteryTopic
  * @param	src: pointer to byte array to de-serializes
  * @param	dst: pointer to RobotBatteryTopic to write to
  * @return	Number of bytes read/converted from the byte array
  */
uint16_t RobotDeserialize_RobotBatteryTopic(uint8_t* src, RobotBatteryTopic* dst) {
	uint8_t i = 0;
	i += RobotDeserialize_uint16(&src[i], &dst->voltage);
	i += RobotDeserialize_uint16(&src[i], &dst->current);
	i += RobotDeserialize_uint16(&src[i], &dst->charge);
	dst->percentage = src[i++];
	dst->status = src[i++];
	return i;
}

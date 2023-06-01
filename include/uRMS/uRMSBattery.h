#ifndef URMS_URMSBATTERY_H_
#define URMS_URMSBATTERY_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "uRMSStdTypes.h"

typedef struct RobotBatteryTopic{
	uint16_t current;		//In mA
	uint16_t voltage;		//In mV
	uint16_t charge;		//In mAh
	uint8_t percentage;		//In %
	uint8_t status;			//0: Not Charging; 1: Charging
} RobotBatteryTopic;

uint16_t RobotSerialize_RobotBatteryTopic(RobotBatteryTopic src, uint8_t* dst);
uint16_t RobotDeserialize_RobotBatteryTopic(uint8_t* src, RobotBatteryTopic* dst);

#ifdef __cplusplus
}
#endif

#endif /* URMS_URMSBATTERY_H_ */

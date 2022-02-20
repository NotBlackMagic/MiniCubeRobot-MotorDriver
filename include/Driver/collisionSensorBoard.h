#ifndef DRIVER_COLLISIONSENSORBOARD_H_
#define DRIVER_COLLISIONSENSORBOARD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "adc.h"
#include "gpio.h"
#include "rcc.h"
#include "timer.h"

#include "pinMaping.h"

typedef struct {
	//Retro-reflective Optical Sensor Values
	uint16_t rroValue[6];		//Front Left, Front Center, Front Right, Back Right, Back Center, Back Left
	uint16_t rroDistance[6];	//
	//Retro-reflective Optical Sensor Collision (under threshold)
	uint8_t rroCollision[6];			//Front Left, Front Center, Front Right, Back Right, Back Center, Back Left
	uint8_t contactCollision[4];		//Front Left, Front Right, Back Right, Back Left
} CollisionSensorStruct;

uint16_t reflectiveSensorThreshold[6];

void CollisionSensorInit();
void CollisionSensorUpdate();
uint8_t CollisionSensorGetRange(uint16_t* ranges);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_COLLISIONSENSORBOARD_H_ */

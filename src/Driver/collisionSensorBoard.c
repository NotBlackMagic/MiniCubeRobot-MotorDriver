#include "collisionSensorBoard.h"

uint8_t calibrationTableLength = 18;
uint16_t calibrationTableDistance[18] = {250, 200, 150, 120, 100, 80, 60, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};							//Distance in mm
uint16_t calibrationTableADCCount[18] = {4096, 3964, 3940, 3907, 3865, 3790, 3636, 3480, 3365, 3215, 3010, 2705, 2285, 1540, 512, 215, 180, 0};	//ADC Counts

CollisionSensorStruct sensorStruct;

uint16_t CollisionSensorADCToDistance(uint16_t adcCount);

/**
  * @brief	This function initializes the Collision (IR and Contact) Sensor System
  * @param	None
  * @return	None
  */
void CollisionSensorInit() {
	reflectiveSensorThreshold[0] = 800;		//Front Left: > 1.46cm -> min. 260 (White w. Sun) max. 710 (Black no Sun)
	reflectiveSensorThreshold[1] = 300;		//Front Center: > 1.04cm -> min. 200 (White w. Sun) max. 230 (Black no Sun)
	reflectiveSensorThreshold[2] = 800;		//Front Right: > 1.46cm -> min. 260 (White w. Sun) max. 710 (Black no Sun)
	reflectiveSensorThreshold[3] = 800;		//Back Right: > 1.46cm -> min. 260 (White w. Sun) max. 710 (Black no Sun)
	reflectiveSensorThreshold[4] = 200;		//Back Center (Down): > 0.5cm -> min. 170 (White w. Sun) max. 190 (Black no Sun)
	reflectiveSensorThreshold[5] = 800;		//Back Left: > 1.46cm ->min. 260 (White w. Sun) max. 710 (Black no Sun)
}

/**
  * @brief	Update call for the Collision (IR and Contact) Sensor System, reads the IR collision sensors and calculates distances
  * @param	None
  * @return	None
  */
uint32_t timestamp = 0;
void CollisionSensorUpdate() {
	if((timestamp + 50) < GetSysTick()) {
		//Enable/power Front reflective sensors
		GPIOWrite(GPIO_OUT_SEN_F_EN, 0x01);
		uint16_t i;
		for(i = 0; i < 512; i++) {
			asm("NOP");
		}

		//Read Front reflective sensors
		sensorStruct.rroValue[0] = ADC2Read(2);	//Front Left
		sensorStruct.rroValue[1] = ADC2Read(3);	//Front Center
		sensorStruct.rroValue[2] = ADC2Read(4);	//Front Right

		//Disable/power off Front reflective sensors
		GPIOWrite(GPIO_OUT_SEN_F_EN, 0x00);

		//Enable/power back reflective sensors
		GPIOWrite(GPIO_OUT_SEN_B_EN, 0x01);
		for(i = 0; i < 512; i++) {
			asm("NOP");
		}

		//Read Front reflective sensors
		sensorStruct.rroValue[5] = ADC2Read(2);	//Back Left
		sensorStruct.rroValue[4] = ADC2Read(3);	//Back Center
		sensorStruct.rroValue[3] = ADC2Read(4);	//Back Right

		//Disable/power off Back reflective sensors
		GPIOWrite(GPIO_OUT_SEN_B_EN, 0x00);

		//Calculate distance from obstacle
		for(i = 0; i < 6; i++) {
			sensorStruct.rroDistance[i] = CollisionSensorADCToDistance(sensorStruct.rroValue[i]);
		}

		//Check for threshold
		for(i = 0; i < 6; i++) {
			if(sensorStruct.rroValue[i] <= reflectiveSensorThreshold[i]) {
				sensorStruct.rroCollision[i] = 0x01;
			}
			else {
				sensorStruct.rroCollision[i] = 0x00;
			}
		}

		//Get Contact collision sensors
		sensorStruct.contactCollision[0] = !GPIORead(GPIO_IN_BTN_FL);	//Front Left
		sensorStruct.contactCollision[1] = !GPIORead(GPIO_IN_BTN_FR);	//Front Right
		sensorStruct.contactCollision[2] = !GPIORead(GPIO_IN_BTN_BR);	//Back Right
		sensorStruct.contactCollision[3] = !GPIORead(GPIO_IN_BTN_BL);	//Back Left

		timestamp = GetSysTick();
	}
}

/**
  * @brief	This function gets the ranges/distances measured/calculted based on the IR collision sensors
  * @param	ranges: pointer to array to write the distances to (in mm)
  * @return	Number of sensors read
  */
uint8_t CollisionSensorGetRange(uint16_t* ranges) {
	ranges[0] = sensorStruct.rroDistance[0];
	ranges[1] = sensorStruct.rroDistance[1];
	ranges[2] = sensorStruct.rroDistance[2];
	ranges[3] = sensorStruct.rroDistance[3];
	ranges[4] = sensorStruct.rroDistance[4];
	ranges[5] = sensorStruct.rroDistance[5];
	return 6;
}

/**
  * @brief	This function estimates the distance to obstacle from a ADC count reading of a IR sensor
  * @param	adcCount: ADC reading
  * @return	Estimated distance in mm
  */
uint16_t CollisionSensorADCToDistance(uint16_t adcCount) {
	uint16_t highIndex = 0;
	uint16_t lowIndex = 0;

	uint8_t i;
	for(i = 1; i < calibrationTableLength; i++) {
		if(calibrationTableADCCount[i] <= adcCount) {
			lowIndex = i;
			highIndex = lowIndex - 1;
			break;
		}
	}

	//Linear interpolation between HIGH and LOW values
	uint16_t dCount = calibrationTableADCCount[highIndex] - calibrationTableADCCount[lowIndex];
	uint16_t dHigh = dCount - (calibrationTableADCCount[highIndex] - adcCount);
	uint16_t dLow = dCount - (adcCount - calibrationTableADCCount[lowIndex]);
	uint16_t distance = (calibrationTableDistance[highIndex]*dHigh + calibrationTableDistance[lowIndex]*dLow) / dCount;

	return distance;
}

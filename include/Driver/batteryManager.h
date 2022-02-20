#ifndef DRIVER_BATTERYMANAGER_H_
#define DRIVER_BATTERYMANAGER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "adc.h"
#include "gpio.h"

/********************************************************/
/*					BATTERY BASE INFO					*/
/********************************************************/
#define BATTERY_TECHNOLOGY						0x03	//LiPo
#define BATTERY_CAPACITY						1200	//In mAh
#define BATTERY_CELLS							1		//Battery Series Cell Count (S)

void BatteryManagerInit();
void BatteryManagerUpdate();
void BatterManagerGetStatus(uint16_t* current, uint16_t* voltage, uint16_t* charge, uint8_t* percentage, uint8_t* status);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_BATTERYMANAGER_H_ */

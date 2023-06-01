#include "batteryManager.h"

#include "pinMaping.h"

//Table to estimate battery charge percentage from cell voltage
//Corresponding Percentage:				  0%   10%   20%   30%   40%   50%   60%   70%   80%   90%  100%
const uint16_t lipoChargeVoltage[11] = {3500, 3570, 3640, 3710, 3780, 3850, 3920, 3990, 4060, 4130, 4200};

uint16_t batteryCurrent;
uint16_t batteryVoltage;
uint8_t batteryChargingStatus;
uint32_t batteryCharge;
uint8_t batteryPercentage;

void BatteryGetInfo(uint16_t* current, uint16_t* voltage, uint8_t* charging);

/**
  * @brief	This function initializes the Battery Manager System
  * @param	None
  * @return	None
  */
void BatteryManagerInit() {
	batteryCurrent = 0;
	batteryVoltage = 0;
	batteryChargingStatus = 0;
	batteryCharge = (BATTERY_CAPACITY << 15);		//Battery charge in Q17.15 mAh
}

/**
  * @brief	Update call for the Battery Manager System, reads and monitors the battery status (voltage, current)
  * @param	None
  * @return	None
  */
uint32_t batteryLowToggleTimestamp = 0;
uint32_t batteryManagerTimestamp = 0;
void BatteryManagerUpdate() {
	//Battery Manager Update Call
	if((batteryManagerTimestamp + 1000) < GetSysTick()) {
		//Update Frequency of 1Hz
		uint8_t isCharging;
		BatteryGetInfo(&batteryCurrent, &batteryVoltage, &isCharging);
		batteryChargingStatus = isCharging;

		//Calculate used power and battery charge
		uint16_t power = (uint16_t)((batteryVoltage * batteryCurrent) / 1000);		//Power in uW
	//	powerConsumption += (power * 596523) >> 16;									//Power consumption in mWh in Q17.15 (1s in hour: 1/3600 in Q31: 596523)
		uint32_t discharge = (batteryCurrent * 596523) >> 16;						//Discharge in Q17.15 mAh
		batteryCharge -= discharge;

		//Estimate battery charge %
		int8_t i;
		for(i = 9; i >= 0; i--) {
			if(lipoChargeVoltage[i] < batteryVoltage) {
				//Calculate distance, for linear interpolation between tabled values
				uint16_t rTop = lipoChargeVoltage[i + 1] - batteryVoltage;
				uint16_t rBottom = batteryVoltage - lipoChargeVoltage[i];

				batteryPercentage = ((i * 10 * rTop) + ((i+1) * 10 * rBottom)) / (rTop + rBottom);
				break;
			}
		}

		batteryManagerTimestamp = GetSysTick();
	}

	//Signal Battery Low through power LED
	if((batteryLowToggleTimestamp + 100) < GetSysTick()) {
		if(batteryChargingStatus == 0x00 && batteryPercentage < 20) {
			GPIOToggle(GPIO_OUT_LED);
		}
		else {
			GPIOWrite(GPIO_OUT_LED, 0x01);
		}

		batteryLowToggleTimestamp = GetSysTick();
	}
}

/**
  * @brief	This function gets the battery status
  * @param	current: pointer to write the battery discharge current to (in mA)
  * @param	voltage: pointer to write the battery voltage to (in mV)
  * @param	charge: pointer to write the estimated remaining battery charge to (in mAh)
  * @param	percentage: pointer to write the estimated remaining battery charge to (in %)
  * @param	status: pointer to write the battery status to: 0 for NOT charging and 1 for charging
  * @return	None
  */
void BatterManagerGetStatus(uint16_t* current, uint16_t* voltage, uint16_t* charge, uint8_t* percentage, uint8_t* status) {
	*current = batteryCurrent;
	*voltage = batteryVoltage;
	*charge = (uint16_t)(batteryCharge >> 15);
	*percentage = batteryPercentage;
	*status = batteryChargingStatus;
}

/**
  * @brief	This function gets the raw battery status from ADC (converted) and GPIOs
  * @param	current: pointer to write the battery discharge current to (in mA)
  * @param	voltage: pointer to write the battery voltage to (in mV)
  * @param	status: pointer to write the battery status to: 0 for NOT charging and 1 for charging
  * @return	None
  */
void BatteryGetInfo(uint16_t* current, uint16_t* voltage,  uint8_t* charging) {
	//Voltage to current conversion (INA180 A3, 100 V/V): I = V * 1 / Rsense * 1 / 100 (Rsense = 0.0511) => I = V * 5.11 (A)
	uint16_t raw = ADC1Read(0x06);
	*current = (raw * 646) >> 12;	//In mA (3300 / 5.11 = 645.79)

	//Voltage from voltage divider with R1=5.1k, R2=10k: V = (R1+R2)/R2 * Vadc = 151/51 * Vadc
	*voltage = (ADC1Read(0x05) * 4983) >> 12;	//In mV

	//Check if battery is charging
	if(GPIORead(GPIO_IN_BAT_CHG) == 0x01) {
		//Battery NOT charging
		*charging = 0;
	}
	else {
		//Battery charging
		*charging = 1;
	}
}

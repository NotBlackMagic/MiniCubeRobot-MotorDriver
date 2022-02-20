#ifndef PINMAPING_H_
#define PINMAPING_H_

#ifdef __cplusplus
extern "C" {
#endif

//Input Pin Mapping
#define GPIO_IN_BTN_BL						8		//Input Button Back Left
#define GPIO_IN_BTN_BR						11		//Input Button Back Right
#define GPIO_IN_BTN_FR						12		//Input Button Front Right
#define GPIO_IN_BTN_FL						15		//Input Button Front Left
#define GPIO_IN_M_FAULT						18		//Input Motor Fault
#define GPIO_IN_BAT_CHG						21		//Input Battery Charging
#define GPIO_IN_ML_ENC						47		//Input Motor Left Encoder
#define GPIO_IN_MR_ENC						7		//Input Motor Right Encoder

//Output Pin Mapping
#define GPIO_OUT_LED						48		//Output LED
#define GPIO_OUT_M_EN						19		//Output Motor Enable
#define GPIO_OUT_M_REF						20		//Output Motor REF (over-current limit)
#define GPIO_OUT_MR_PH						22		//Output Motor Right Direction
#define GPIO_OUT_MR_PWM						23		//Output Motor Right PWM
#define GPIO_OUT_ML_PH						24		//Output Motor Left Direction
#define GPIO_OUT_ML_PWM						25		//Output Motor Left PWM
#define GPIO_OUT_SEN_B_EN					45		//Output Front Sensors Enable
#define GPIO_OUT_SEN_F_EN					46		//Output Back Sensors Enable
#define GPIO_OUT_SPI_CS						28		//Output SPI2 CS

//Peripheral Pin Mapping
#define GPIO_I2C2_SCL						26		//I2C2 SCL
#define GPIO_I2C2_SDA						27		//I2C2 SDA

//ADC Pin Mapping
#define GPIO_ADC_HAL_S1						0		//ADC Input HAL S1					ADC1 CH0
#define GPIO_ADC_HAL_S2						1		//ADC Input HAL S2					ADC1 CH1
#define GPIO_ADC_SEN_A						2		//ADC Input Reflective Sensor A		ADC1 CH2
#define GPIO_ADC_SEN_B						3		//ADC Input Reflective Sensor B		ADC1 CH3
#define GPIO_ADC_SEN_C						4		//ADC Input Reflective Sensor C		ADC1 CH4
#define GPIO_ADC_BAT_V						5		//ADC Input Battery Voltage			ADC1 CH5
#define GPIO_ADC_BAT_I						6		//ADC Input Battery Current			ADC1 CH6
#define GPIO_ADC_MR_I						16		//ADC Input Motor Right Current		ADC1 CH8
#define GPIO_ADC_ML_I						17		//ADC Input Motor Left Current		ADC1 CH9

#ifdef __cplusplus
}
#endif

#endif /* PINMAPING_H_ */

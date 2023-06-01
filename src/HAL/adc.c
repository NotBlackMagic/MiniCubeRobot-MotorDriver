#include "adc.h"

//LPF 1st Order with fc=0.01*fs
#define CURRENT_FILTER_A1									30771		//Alpha 1: 0.9390625058174924
#define CURRENT_FILTER_B0									998			//Beta 0: 0.030468747091253797
#define CURRENT_FILTER_B1									998			//Beta 1: 0.030468747091253797

uint16_t adcPreviousValues[6];

uint16_t adcChannelValues[6];
uint16_t adcFilteredValues[6];

/**
  * @brief	This function initializes the ADC1
  * @param	None
  * @return	None
  */
void ADC1Init() {
	//Enable bus clocks
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

	//Configure the ADC
	LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_NONE);
	LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_MULTI_INDEPENDENT);
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
	LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS);
	LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

	//Configure Input Channel
	//Sample time (Fadc = 8Mhz): 1/Fadc * (Ts + 12.5)
	//Scan mode sample frequency: Fs = Fadc / ((Ts + 12.5) * channels) = 8MHz / ((239.5 + 12.5) * 6) = 5.3ksps
	//ADC Input HAL S1 (ADC1 CH0)
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_239CYCLES_5);		//Minimum used for 12bits resolution: LL_ADC_SAMPLINGTIME_7CYCLES_5
	//ADC Input HAL S2 (ADC1 CH1)
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	//ADC Input Battery Voltage (ADC1 CH5)
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	//ADC Input Battery Current (ADC1 CH6)
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_6);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	//ADC Input Motor Right Current (ADC1 CH8)
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_8);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	//ADC Input Motor Left Current (ADC1 CH9)
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_6, LL_ADC_CHANNEL_9);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_239CYCLES_5);

	//Configure Interrupts
//	NVIC_SetPriority(ADC1_2_IRQn, 0);
//	NVIC_EnableIRQ(ADC1_2_IRQn);
//	LL_ADC_EnableIT_EOS(ADC1);

	//Configure DMA
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_DMA_ConfigTransfer(	DMA1,
							LL_DMA_CHANNEL_1,
							LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
							LL_DMA_MODE_CIRCULAR              |
							LL_DMA_PERIPH_NOINCREMENT         |
							LL_DMA_MEMORY_INCREMENT           |
							LL_DMA_PDATAALIGN_HALFWORD        |
							LL_DMA_MDATAALIGN_HALFWORD        |
							LL_DMA_PRIORITY_LOW);
	LL_DMA_ConfigAddresses(	DMA1,
							LL_DMA_CHANNEL_1,
							LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
							(uint32_t)&adcChannelValues,
							LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(	DMA1,
							LL_DMA_CHANNEL_1,
							0x06);

	//Configure Interrupts
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);		//Enable DMA transfer interruption: transfer complete

	//Enable DMA
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

	//Enable ADC
	LL_ADC_Enable(ADC1);
//	while(LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);

	//IMPORTANT: Delay between ADC end of calibration and ADC enable
	uint32_t wait_loop_index = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
	while(wait_loop_index != 0) {
		wait_loop_index--;
	}

	//Run ADC self calibration
	LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
}

/**
  * @brief	This function initializes the ADC2
  * @param	None
  * @return	None
  */
void ADC2Init() {
	//Enable bus clocks
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

	//Configure the ADC
	LL_ADC_SetDataAlignment(ADC2, LL_ADC_DATA_ALIGN_RIGHT);
	LL_ADC_SetSequencersScanMode(ADC2, LL_ADC_SEQ_SCAN_DISABLE);
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC2), LL_ADC_PATH_INTERNAL_NONE);
	LL_ADC_SetMultimode(__LL_ADC_COMMON_INSTANCE(ADC2), LL_ADC_MULTI_INDEPENDENT);
	LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_DISABLE);
	LL_ADC_REG_SetSequencerDiscont(ADC2, LL_ADC_REG_SEQ_DISCONT_DISABLE);
	LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_NONE);

	//Configure Input Channel
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_7CYCLES_5);

	//Configure Interrupts
//	NVIC_SetPriority(ADC1_2_IRQn, 0);
//	NVIC_EnableIRQ(ADC1_2_IRQn);
//	LL_ADC_EnableIT_EOS(ADC2);

	//Enable ADC
	LL_ADC_Enable(ADC2);
//	while(LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0);

	//IMPORTANT: Delay between ADC end of calibration and ADC enable
	uint32_t wait_loop_index = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
	while(wait_loop_index != 0) {
		wait_loop_index--;
	}

	//Run ADC self calibration
	LL_ADC_StartCalibration(ADC2);
	while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0);
}

/**
  * @brief	This function starts the ADC conversion
  * @param	None
  * @return	None
  */
void ADC1Start() {
	LL_ADC_REG_StartConversionSWStart(ADC1);
}


/**
  * @brief	This function reads ADC1 Channels and returns value
  * @param	channel: ADC Channel number to read
  * @return	ADC Read Value (12 bits)
  */
uint16_t ADC1Read(uint8_t channel) {
	uint16_t adcValue = 0;

	switch(channel) {
		case 0:
			adcValue = adcChannelValues[0];
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 1:
			adcValue = adcChannelValues[1];
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 2:
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 3:
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 4:
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 5:
			adcValue = adcFilteredValues[2];
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 6:
			adcValue = adcFilteredValues[3];
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 7:
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 8:
			adcValue = adcFilteredValues[4];
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
		case 9:
			adcValue = adcFilteredValues[5];
//			LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_9);
//			LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
//			LL_ADC_REG_StartConversionSWStart(ADC1);	//Start the conversion
			break;
	}

//	//Wait until conversion completion
//	while (LL_ADC_IsActiveFlag_EOS(ADC1) == 0);
//	//Get the conversion value
//	adcValue = LL_ADC_REG_ReadConversionData12(ADC1);

	return adcValue;
}

/**
  * @brief	This function reads ADC2 Channels and returns value
  * @param	channel: ADC Channel number to read
  * @return	ADC Read Value (12 bits)
  */
uint16_t ADC2Read(uint8_t channel) {
	switch(channel) {
		case 0:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 1:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 2:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_2);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 3:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_3);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 4:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 5:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 6:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 7:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_7);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 8:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
		case 9:
			LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_9);
			LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_7CYCLES_5);	//Set the input channel
			LL_ADC_REG_StartConversionSWStart(ADC2);	//Start the conversion
			break;
	}

	//Wait until conversion completion
	while (LL_ADC_IsActiveFlag_EOS(ADC2) == 0);
	//Get the conversion value
	return LL_ADC_REG_ReadConversionData12(ADC2);
}

/**
  * @brief	ADC1/2 IRQ Handler
  * @param	None
  * @return	None
  */
//void ADC1_2_IRQHandler(void) {
//	if(LL_ADC_IsActiveFlag_EOS(ADC1) != 0) {
//		//New conversion complete
//		adcChannelValues[adcChannelIndex++] = LL_ADC_REG_ReadConversionData12(ADC1);
//
//		if(adcChannelIndex >= 9) {
//			adcChannelIndex = 0;
//		}
//
//		//Clear conversion complete EOS flag
//		LL_ADC_ClearFlag_EOS(ADC1);
//	}
//}

/**
  * @brief  DMA1 Channel1 IRQ Handler
  * @param  None
  * @return None
  */
void DMA1_Channel1_IRQHandler(void) {
	if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1) {
		//Conversion complete interrupt
		adcFilteredValues[2] = ((uint32_t)adcFilteredValues[2] * CURRENT_FILTER_A1 + (uint32_t)adcPreviousValues[2] * CURRENT_FILTER_B1 + (uint32_t)adcChannelValues[2] * CURRENT_FILTER_B0) >> 15;
		adcPreviousValues[2] = adcChannelValues[2];
		adcFilteredValues[3] = ((uint32_t)adcFilteredValues[3] * CURRENT_FILTER_A1 + (uint32_t)adcPreviousValues[3] * CURRENT_FILTER_B1 + (uint32_t)adcChannelValues[3] * CURRENT_FILTER_B0) >> 15;
		adcPreviousValues[3] = adcChannelValues[3];
		adcFilteredValues[4] = ((uint32_t)adcFilteredValues[4] * CURRENT_FILTER_A1 + (uint32_t)adcPreviousValues[4] * CURRENT_FILTER_B1 + (uint32_t)adcChannelValues[4] * CURRENT_FILTER_B0) >> 15;
		adcPreviousValues[4] = adcChannelValues[4];
		adcFilteredValues[5] = ((uint32_t)adcFilteredValues[5] * CURRENT_FILTER_A1 + (uint32_t)adcPreviousValues[5] * CURRENT_FILTER_B1 + (uint32_t)adcChannelValues[5] * CURRENT_FILTER_B0) >> 15;
		adcPreviousValues[5] = adcChannelValues[5];

		//Clear transfer complete TC1 flag
		LL_DMA_ClearFlag_TC1(DMA1);
	}

	if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1) {
		//Transfer error interrupt

		//Clear transfer error TE1 flag
		LL_DMA_ClearFlag_TE1(DMA1);
	}
}

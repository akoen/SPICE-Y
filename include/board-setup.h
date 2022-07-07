#ifndef INCLUDE_BOARD_SETUP
#define INCLUDE_BOARD_SETUP

#include <stm32f1xx_hal.h>

#include <config.h>

extern ADC_HandleTypeDef AdcHandle;
extern uint16_t DMA1Data[IR_SENS_NUM_READINGS];
extern bool DMA1DataAvailable;

void ADC_Config();
void timerHeartbeatInit();

#endif
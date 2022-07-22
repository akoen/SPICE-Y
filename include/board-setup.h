#ifndef INCLUDE_BOARD_SETUP
#define INCLUDE_BOARD_SETUP

#include <stm32f1xx_hal.h>
#include <Adafruit_SSD1306.h>

#include "config.h"

extern ADC_HandleTypeDef AdcHandle;
extern uint16_t DMA1Data[IR_SENS_NUM_READINGS];
extern bool DMA1DataAvailable;

extern Adafruit_SSD1306 OLEDDisplayHandler;;


namespace Setup {
    void ADC();
    void timerHeartbeatInit();
    void OLED();
}

#endif
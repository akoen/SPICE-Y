#include <HardwareTimer.h>

#include <board-setup.h>
#include <config.h>

ADC_HandleTypeDef AdcHandle;
uint16_t DMA1Data[IR_SENS_NUM_READINGS];
bool DMA1DataAvailable = false;

HardwareTimer timerHeartbeat(TIM1);


namespace Setup {
    bool ledOn = false;
    
    void timerHeartbeatInterrupt() {
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
        digitalWrite(PB2, ledOn ? HIGH : LOW);
    }

    void timerHeartbeatInit() {
        timerHeartbeat.setOverflow(2, HERTZ_FORMAT);
        timerHeartbeat.attachInterrupt(timerHeartbeatInterrupt);
        timerHeartbeat.resume();
    }

/**
    @brief  ADC configuration
    @param  None
    @retval None
*/
    void ADC() {
        ADC_ChannelConfTypeDef sConfig;

        /* Configuration of ADCx init structure: ADC parameters and regular group */
        AdcHandle.Instance = ADC1;
        AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;                  /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
        AdcHandle.Init.ContinuousConvMode = ENABLE;                    /* Continuous mode to have maximum conversion speed (no delay between conversions) */
        AdcHandle.Init.NbrOfConversion = 2;                             /* Parameter discarded because sequencer is disabled */
        AdcHandle.Init.DiscontinuousConvMode = DISABLE;                 /* Parameter discarded because sequencer is disabled */
        AdcHandle.Init.NbrOfDiscConversion = 1;                         /* Parameter discarded because sequencer is disabled */
        AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */

        if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
            /* ADC initialization error */
            Error_Handler();
        }

        /* Configuration of channel on ADCx regular group on sequencer rank 1 */
        /* Note: Considering IT occurring after each ADC conversion if ADC          */
        /*       conversion is out of the analog watchdog window selected (ADC IT   */
        /*       enabled), select sampling time and ADC clock with sufficient       */
        /*       duration to not create an overhead situation in IRQHandler.        */
        sConfig.Channel = ADC_CHANNEL_4;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_FAST_SAMPLETIME;
        HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);

        sConfig.Channel = ADC_CHANNEL_5;
        sConfig.Rank = ADC_REGULAR_RANK_2;
        sConfig.SamplingTime = ADC_FAST_SAMPLETIME;
        HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
    }
}

extern "C" void SystemClock_Config() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
    @brief  This function handles ADC interrupt request.
    @param  None
    @retval None
*/
extern "C" void ADC1_2_IRQHandler(void) {
    HAL_ADC_IRQHandler(&AdcHandle);
}

/**
  @brief  This function handles DMA interrupt request.
  @param  None
  @retval None
*/
extern "C" void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

/**
    @brief ADC MSP initialization
           This function configures the hardware resources used in this example:
             - Enable clock of ADC peripheral
             - Configure the GPIO associated to the peripheral channels
             - Configure the DMA associated to the peripheral
             - Configure the NVIC associated to the peripheral interruptions
    @param hadc: ADC handle pointer
    @retval None
*/
extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
    GPIO_InitTypeDef GPIO_InitStruct;
    static DMA_HandleTypeDef DmaHandle;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable clock of GPIO associated to the peripheral channels */
    //  __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Enable clock of ADCx peripheral */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* Configure ADCx clock prescaler */
    /* Caution: On STM32F1, ADC clock frequency max is 14MHz (refer to device   */
    /*          datasheet).                                                     */
    /*          Therefore, ADC clock prescaler must be configured in function   */
    /*          of ADC clock source frequency to remain below this maximum      */
    /*          frequency.                                                      */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /* Enable clock of DMA associated to the peripheral */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* Configure GPIO pin of the selected ADC channel */
    // TODO: Remove HAL code if works
    pinMode(PA4, INPUT_ANALOG);
    // GPIO_InitStruct.Pin = ADC_CHANNEL_0;
    // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    // GPIO_InitStruct.Pull = GPIO_PIN_0;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // GPIO_InitStruct.Pin = ADC_CHANNEL_1;
    // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    // GPIO_InitStruct.Pull = GPIO_PIN_1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    pinMode(PA5, INPUT_ANALOG);
    /*##-3- Configure the DMA ##################################################*/
    /* Configure DMA parameters */
    DmaHandle.Instance = DMA1_Channel1;

    DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
    DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* Transfer to memory by half-word to match with buffer variable type: half-word */
    DmaHandle.Init.Mode = DMA_CIRCULAR;                           /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
    DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;

    /* Deinitialize  & Initialize the DMA for new transfer */
    HAL_DMA_DeInit(&DmaHandle);
    HAL_DMA_Init(&DmaHandle);

    /* Associate the initialized DMA handle to the ADC handle */
    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

    /*##-4- Configure the NVIC #################################################*/

    /* NVIC configuration for DMA interrupt (transfer completion or error) */
    /* Priority: high-priority */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* NVIC configuration for ADC interrupt */
    /* Priority: high-priority */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
    @brief ADC MSP de-initialization
           This function frees the hardware resources used in this example:
             - Disable clock of ADC peripheral
             - Revert GPIO associated to the peripheral channels to their default state
             - Revert DMA associated to the peripheral to its default state
             - Revert NVIC associated to the peripheral interruptions to its default state
    @param hadc: ADC handle pointer
    @retval None
*/
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
    /*##-1- Reset peripherals ##################################################*/
    __HAL_RCC_ADC1_FORCE_RESET();
    __HAL_RCC_ADC1_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* De-initialize GPIO pin of the selected ADC channel */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /*##-3- Disable the DMA ####################################################*/
    /* De-Initialize the DMA associated to the peripheral */
    if (hadc->DMA_Handle != NULL) {
        HAL_DMA_DeInit(hadc->DMA_Handle);
    }

    /*##-4- Disable the NVIC ###################################################*/
    /* Disable the NVIC configuration for DMA interrupt */
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

    /* Disable the NVIC configuration for ADC interrupt */
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
}

/**
    @brief  Conversion DMA half-transfer callback in non blocking mode
    @param  hadc: ADC handle
    @retval None
*/
extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *) {
}

/**
    @brief  Conversion complete callback in non blocking mode
    @param  AdcHandle : AdcHandle handle
    @note   This example shows a simple way to report end of conversion
            and get convers100 Hzion result. You can add your own implementation.
    @retval None
*/
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *) {
    HAL_ADC_Stop_DMA(&AdcHandle);
    DMA1DataAvailable = true;
}

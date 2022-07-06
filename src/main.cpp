#include "Arduino.h"
#include "HardwareTimer.h"

/* Private define ------------------------------------------------------------*/
#define IR_SENS_NUM_READINGS 256

/* Private variables ---------------------------------------------------------*/
/* ADC handler declaration */
ADC_HandleTypeDef AdcHandle;
/* Variable containing ADC conversions results */
uint16_t DMA1Data[IR_SENS_NUM_READINGS];
bool DMA1DataAvailable = false;

HardwareTimer timerHeartbeat(TIM1);
bool ledOn = false;
void timerHeartbeatInterrupt()
{
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

/**
    @brief  This function handles ADC interrupt request.
    @param  None
    @retval None
*/
extern "C" void ADC1_2_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&AdcHandle);
}

/**
  @brief  This function handles DMA interrupt request.
  @param  None
  @retval None
*/
extern "C" void DMA1_Channel1_IRQHandler(void)
{
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
extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
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
  GPIO_InitStruct.Pin = ADC_CHANNEL_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  /*##-1- Reset peripherals ##################################################*/
  __HAL_RCC_ADC1_FORCE_RESET();
  __HAL_RCC_ADC1_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize GPIO pin of the selected ADC channel */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

  /*##-3- Disable the DMA ####################################################*/
  /* De-Initialize the DMA associated to the peripheral */
  if (hadc->DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }

  /*##-4- Disable the NVIC ###################################################*/
  /* Disable the NVIC configuration for DMA interrupt */
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

  /* Disable the NVIC configuration for ADC interrupt */
  HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
}

/**
    @brief  ADC configuration
    @param  None
    @retval None
*/
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef sConfig;

  /* Configuration of ADCx init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADC1;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode = ADC_SCAN_DISABLE;                  /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode = ENABLE;                    /* Continuous mode to have maximum conversion speed (no delay between conversions) */
  AdcHandle.Init.NbrOfConversion = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                 /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion = 1;                         /* Parameter discarded because sequencer is disabled */
  // AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO; /* Trig of conversion start done by external event */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
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
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
}

/**
    @brief  Conversion DMA half-transfer callback in non blocking mode
    @param  hadc: ADC handle
    @retval None
*/
extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *)
{
}

/**
    @brief  Conversion complete callback in non blocking mode
    @param  AdcHandle : AdcHandle handle
    @note   This example shows a simple way to report end of conversion
            and get convers100 Hzion result. You can add your own implementation.
    @retval None
*/
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *)
{
  HAL_ADC_Stop_DMA(&AdcHandle);
  DMA1DataAvailable = true;

}

// the setup routine runs once when you press reset:
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PB0, OUTPUT);

  Serial.begin(115200, SERIAL_8N1);

  timerHeartbeat.setOverflow(2, HERTZ_FORMAT);
  timerHeartbeat.attachInterrupt(timerHeartbeatInterrupt);
  timerHeartbeat.resume();


  /* Configure the ADC peripheral */
  ADC_Config();

  /* Run the ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  /* Start ADC conversion on regular group with transfer by DMA */
  HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
  // HAL_TIM_Base_Start_IT(timerSensorReadFast.getHandle());
}

// the loop routine runs over and over again forever:
void loop()
{
  if(DMA1DataAvailable) {
    digitalWrite(PB0, HIGH);
    while (!Serial)
      ;
    Serial.write((uint8_t *) DMA1Data, sizeof(DMA1Data));
    digitalWrite(PB0, LOW);
    HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)DMA1Data, IR_SENS_NUM_READINGS);
    DMA1DataAvailable = false;
  }
}
/**
* @brief Blink: Turns on an LED for one second then off for
* one second and then repeats.
*/

#include "config.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include "Arduino.h"
#include <vector>

#ifndef ADC_CLOCK_DIV
#ifdef ADC_CLOCK_SYNC_PCLK_DIV4
#define ADC_CLOCK_DIV       ADC_CLOCK_SYNC_PCLK_DIV4
#elif ADC_CLOCK_SYNC_PCLK_DIV2
#define ADC_CLOCK_DIV       ADC_CLOCK_SYNC_PCLK_DIV2
#elif defined(ADC_CLOCK_ASYNC_DIV1)
#define ADC_CLOCK_DIV       ADC_CLOCK_ASYNC_DIV1
#endif
#endif /* !ADC_CLOCK_DIV */

#ifndef ADC_SAMPLINGTIME
#if defined(ADC_SAMPLETIME_8CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_8CYCLES_5;
#elif defined(ADC_SAMPLETIME_12CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_12CYCLES_5;
#elif defined(ADC_SAMPLETIME_13CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_13CYCLES_5;
#elif defined(ADC_SAMPLETIME_15CYCLES)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_15CYCLES;
#elif defined(ADC_SAMPLETIME_16CYCLES)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_16CYCLES;
#elif defined(ADC_SAMPLETIME_19CYCLES_5)
#define ADC_SAMPLINGTIME        ADC_SAMPLETIME_19CYCLES_5;
#endif
#endif /* !ADC_SAMPLINGTIME */

//Set LED_BUILTIN if it is not defined by Arduino framework
#define LED_BUILTIN PC13


HardwareTimer timerHeartbeat(TIM1);
bool ledOn = false;

void timerHeartbeatInterrupt() {
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}

std::vector<float> phototransistorVoltage {};
void goertzel(std::vector<float>* phototransistorVoltage, uint16_t SAMPLING_RATE) {
  const float TARGET_FREQUENCY = 10000;
  const uint16_t numSamples = phototransistorVoltage->size();

  const uint16_t scalingFactor = numSamples / 2;

  const float k = (0.5 + ((float) (numSamples * TARGET_FREQUENCY) / SAMPLING_RATE));

  const float omega = (2.0 * M_PI);
}

ADC_HandleTypeDef hadc;
static void MX_ADC1_Init();

/**
 * @brief Initialize LED pin as digital write.
 * @param none
 * @retval none
 */
void setup()
{
  //initialize LED digital pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PA0, INPUT);
  pinMode(PB0, OUTPUT);


  Serial.begin(115200);

  timerHeartbeat.setOverflow(1, HERTZ_FORMAT); // Set overflow to 32761 => timer frequency = 65522 Hz / 32761 = 2 Hz
  timerHeartbeat.attachInterrupt(timerHeartbeatInterrupt);
  timerHeartbeat.resume(); // Start

  MX_ADC1_Init();
}


/**
 * @brief Turn LED on for 1 sec and off for 1 sec.
 * @param none
 * @retval none
 */
void loop()
{
  digitalWrite(PB0, HIGH);
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1); // polling timeout 1ms - cannot go lower
  uint32_t value = HAL_ADC_GetValue(&hadc);
  digitalWrite(PB0, LOW);
  //Serial.println(value);
  }

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc.Instance = ADC1;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
    while (1);

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    while(1);

  pinmap_pinout(analogInputToPinName(A0), PinMap_ADC);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

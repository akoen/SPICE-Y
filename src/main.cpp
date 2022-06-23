/**
* @brief Blink: Turns on an LED for one second then off for
* one second and then repeats.
*/

#include "config.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include "Arduino.h"
#include <vector>

//Set LED_BUILTIN if it is not defined by Arduino framework
#define LED_BUILTIN PC13

HardwareTimer timerHeartbeat(TIM1);
bool ledOn = false;

void timerHeartbeatInterrupt() {
  ledOn = !ledOn;
  digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
}


ADC_HandleTypeDef hadc;
static void MX_ADC1_Init();

HardwareTimer timerSensorReadFast(TIM2);
void timerSensorReadFastInterrupt() {

  digitalWrite(PB5, HIGH);


  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    while(1);

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1); // polling timeout 1ms - cannot go lower
  uint32_t valA = HAL_ADC_GetValue(&hadc);
  //Serial.println(value);
  HAL_ADC_Stop(&hadc);

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    while(1);

  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, 1); // polling timeout 1ms - cannot go lower
  uint32_t valB = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);

  // Serial.print(valA);
  // Serial.print(" ");
  // Serial.println(valB);

  digitalWrite(PB5, LOW);


}

std::vector<float> phototransistorVoltage {};
void goertzel(std::vector<float>* phototransistorVoltage, uint16_t SAMPLING_RATE) {
  const float TARGET_FREQUENCY = 10000;
  const uint16_t numSamples = phototransistorVoltage->size();

  const uint16_t scalingFactor = numSamples / 2;

  const float k = (0.5 + ((float) (numSamples * TARGET_FREQUENCY) / SAMPLING_RATE));

  const float omega = (2.0 * M_PI);
}


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
  pinMode(PB5, OUTPUT);
  pinMode(PA8, INPUT_PULLUP);

  Serial.begin(SERIAL_BAUD_RATE);

  timerHeartbeat.setOverflow(2, HERTZ_FORMAT);
  timerHeartbeat.attachInterrupt(timerHeartbeatInterrupt);
  timerHeartbeat.resume();

  timerSensorReadFast.setOverflow(10000, HERTZ_FORMAT);
  timerSensorReadFast.attachInterrupt(timerSensorReadFastInterrupt);
  timerSensorReadFast.resume();

  MX_ADC1_Init();
}


/**
 * @brief Turn LED on for 1 sec and off for 1 sec.
 * @param none
 * @retval none
 */
void loop()
{
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


  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc.Instance = ADC1;
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
    while (1);

  /** Configure Regular Channel
  */


  

  //pinmap_pinout(analogInputToPinName(A8), PinMap_ADC);
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


extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
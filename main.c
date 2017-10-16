/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
bool Phase_1_High,Phase_1_Low, Phase_2_High, Phase_2_Low, Phase_3_High, Phase_3_Low, Hall_1, Hall_2, Hall_3;

const uint8_t BRIDGE_STEPS_FORWARD[8][6] =   // Motor step //A B C
{
    { false,false   ,   false,false   ,  false,false },  // HighZ
    { false,false   ,   false, true   ,  true, false },  // h 1, step 6
    { false,true    ,   true ,false   ,  false,false },  // h 2, step 4
    { false,true    ,   false,false   ,  true ,false },  // h 3, step 5
    { true ,false   ,   false,false   ,  false,true  },  // h 4, step 2
    { true ,false   ,   false,true    ,  false,false },  // h 5, step 1
    { false,false   ,   true, false   ,  false,true  },  // h 6, step 3
    { false,false   ,   false,false   ,  false,false },  // HighZ
};

const uint8_t BRIDGE_STEPS_REVERSE[8][6] =   // Motor step //A B C
{
    { false,false   ,   false,false   ,  false,false },  // HighZ

    { false,false   ,   true ,false   ,  false, true },  // h 1, step 6
    { true ,false   ,   false,true    ,  false,false },  // h 2, step 4
    { true ,false   ,   false,false   ,  false,true  },  // h 3, step 5
    { false,true    ,   false,false   ,  true ,false },  // h 4, step 2
    { false,true    ,   true ,false   ,  false,false },  // h 5, step 1
    { false,false   ,   false, true   ,  true ,false },  // h 6, step 3

    { false,false   ,   false,false   ,  false,false },  // HighZ

};


uint32_t globalHeartbeat_50us = 0, heartbeat_100us = 0, heartbeat_1ms = 0, heartbeat_10ms = 0, led_state = 0, hallEffectTick = 0, diff = 0;
uint16_t brakePedalVlaue_raw = 0, brakePedalVlaue_scaled = 0, accelPedalValue_scaled = 0, accelPedalValue_raw = 0;
uint16_t brakeMin_in = 1080, brakeMax_in = 2895, accelMin_in = 1080, accelMax_in = 2895;
uint16_t brakeRange = 0, accelRange = 0;
uint16_t hallSpeed = 0, maxMotorSpeed = 3000;

uint8_t systemState = 0, hallPosition = 0, lastHallPosition = 0;
uint8_t  avgCurrent = 0, currentSum = 0, supplyVoltage = 24;

float Kp = 1;
float Ki = 0.1;

float motorSpeedConstant = 0.004; //volt per rpm
float motorBrakeConstant = 0.001;

int demandedSpeed = 0, measuredSpeed = 0, speedError = 0, speedErrorSum = 0, controlOutput = 0;

bool pidEnabled = false;
bool gearForward = true;
bool deadManSwitch = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    globalHeartbeat_50us++;
    //HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
}

void LED_stateMachine (void) {
    switch (systemState) {
    case 0:
      if ((globalHeartbeat_50us - led_state) > 20000) {
        HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
        led_state = globalHeartbeat_50us;
      }

    	if (Hall_1)
    		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
    	else
    		HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);

    	if (Hall_2)
    		HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
    	else
    		HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);

    	if (Hall_3)
    		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
    	else
    		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);

    break;

    case 1: //hall effect problem
      if ((globalHeartbeat_50us - led_state) > 10000) {
        HAL_GPIO_WritePin(GPIOD, LD4_Pin,GPIO_PIN_RESET);

    		//toggle hall effect LEDs
    		HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
    		HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
    		HAL_GPIO_TogglePin(GPIOD, LD6_Pin);

    		led_state = globalHeartbeat_50us;
    	}
    break;

    case 2 :
      if ((globalHeartbeat_50us - led_state) > 10000) {
        HAL_GPIO_WritePin(GPIOD, LD4_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LD3_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LD5_Pin,GPIO_PIN_RESET);

        //toggle red LED
        HAL_GPIO_TogglePin(GPIOD, LD6_Pin);

        led_state = globalHeartbeat_50us;
      }
    break;

    case 99: //Dead Man
      if ((globalHeartbeat_50us - led_state) > 5000) {
        HAL_GPIO_WritePin(GPIOD, LD4_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, LD3_Pin,GPIO_PIN_RESET);
  		  HAL_GPIO_WritePin(GPIOD, LD6_Pin,GPIO_PIN_RESET);

    		//toggle red LED
    		HAL_GPIO_TogglePin(GPIOD, LD5_Pin);

    		led_state = globalHeartbeat_50us;
    	}
    break;
    }
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  //initialise counters

  //initialise mosfet states
  Phase_1_High = false; Phase_1_Low = false;
  Phase_2_High = false; Phase_2_Low = false;
  Phase_3_High = false; Phase_3_Low = false;
  Hall_1 = true; Hall_2 = true; Hall_3 = false;

  brakeRange = (brakeMax_in - brakeMin_in);
  accelRange = (accelMax_in - accelMin_in);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);


	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);

  //HAL_Delay(5000);
  //Hall_1 = true; Hall_2 = true; Hall_3 = false;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	Hall_1 = HAL_GPIO_ReadPin(GPIOD, Hall1_Pin);
	Hall_2 = HAL_GPIO_ReadPin(GPIOD, Hall2_Pin);
	Hall_3 = HAL_GPIO_ReadPin(GPIOD, Hall3_Pin);

	hallPosition = (Hall_1<<2) + (Hall_2<<1) + (Hall_3);
	lastHallPosition = hallPosition;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
		diff = globalHeartbeat_50us - heartbeat_100us;
		if (diff & 0x80000000) {
     diff = ~diff + 1;
		}

    if (diff > 2) {
      heartbeat_100us = globalHeartbeat_50us;
			//100us stuff
			//commutation
			//calculate speed and position for control


      Hall_1 = HAL_GPIO_ReadPin(GPIOD, Hall1_Pin);
      Hall_2 = HAL_GPIO_ReadPin(GPIOD, Hall2_Pin);
      Hall_3 = HAL_GPIO_ReadPin(GPIOD, Hall3_Pin);


      //compile hall effect values
			hallPosition = (Hall_1<<2) + (Hall_2<<1) + (Hall_3);

      //calcluate speed
//      if (hallPosition != lastHallPosition) {
//        hallSpeed = (int)((((float)1000000)/ (6*(globalHeartbeat_50us - hallEffectTick)*50))*60); //in RPM
//        measuredSpeed = hallSpeed;
//        hallEffectTick = globalHeartbeat_50us;
//        lastHallPosition = hallPosition;
//      }

			if (!deadManSwitch) {
				//person is dead :O !!
				systemState = 99;
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;
				TIM1->CCR3 = 0;
				TIM1->CCR4 = 0;
				TIM8->CCR1 = 0;
				TIM8->CCR2 = 0;

			} else if (brakePedalVlaue_scaled > 40) {
        systemState = 0;
	      //braking
				TIM1->CCR1 = 0;
				TIM1->CCR3 = 0;
				TIM8->CCR1 = 0;

        TIM1->CCR2 = brakePedalVlaue_scaled;
        TIM1->CCR4 = brakePedalVlaue_scaled;
				TIM8->CCR2 = brakePedalVlaue_scaled;

        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

			} else {
        //systemState = 1;

				// TIM1->CCR1 = 0;
				// TIM1->CCR2 = 0;
				// TIM1->CCR3 = 0;
				// TIM1->CCR4 = 0;
				// TIM8->CCR1 = 0;
				// TIM8->CCR2 = 0;

				if ((hallPosition <=6) && (hallPosition >=1)) {
          systemState = 0;

		      if (!gearForward){
						Phase_1_High = BRIDGE_STEPS_REVERSE[hallPosition][0];
						Phase_1_Low = BRIDGE_STEPS_REVERSE[hallPosition][1];

						Phase_2_High = BRIDGE_STEPS_REVERSE[hallPosition][2];
						Phase_2_Low = BRIDGE_STEPS_REVERSE[hallPosition][3];

						Phase_3_High = BRIDGE_STEPS_REVERSE[hallPosition][4];
						Phase_3_Low = BRIDGE_STEPS_REVERSE[hallPosition][5];

					} else {
						Phase_1_High = BRIDGE_STEPS_FORWARD[hallPosition][0];
						Phase_1_Low = BRIDGE_STEPS_FORWARD[hallPosition][1];

						Phase_2_High = BRIDGE_STEPS_FORWARD[hallPosition][2];
						Phase_2_Low = BRIDGE_STEPS_FORWARD[hallPosition][3];

						Phase_3_High = BRIDGE_STEPS_FORWARD[hallPosition][4];
						Phase_3_Low = BRIDGE_STEPS_FORWARD[hallPosition][5];
					}

					// HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					// HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
					// HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
					// HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
          //
					// HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
					// HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);

          if (pidEnabled) {

            if (controlOutput >=0) {
              TIM1->CCR1 = Phase_1_High *controlOutput;
    					TIM1->CCR2 = Phase_1_Low *controlOutput;
    					TIM1->CCR3 = Phase_2_High *controlOutput;
    					TIM1->CCR4 = Phase_2_Low *controlOutput;
    					TIM8->CCR1 = Phase_3_High *controlOutput;
    					TIM8->CCR2 = Phase_3_Low *controlOutput;
            } else {
              TIM1->CCR1 = 0 * abs(controlOutput);
    					TIM1->CCR2 = 1 *abs(controlOutput);
    					TIM1->CCR3 = 0 *abs(controlOutput);
    					TIM1->CCR4 = 1 *abs(controlOutput);
    					TIM8->CCR1 = 0 *abs(controlOutput);
    					TIM8->CCR2 = 1 *abs(controlOutput);
            }
          } else {
  					TIM1->CCR1 = Phase_1_High *accelPedalValue_scaled;
  					TIM1->CCR2 = Phase_1_Low *accelPedalValue_scaled;
  					TIM1->CCR3 = Phase_2_High *accelPedalValue_scaled;
  					TIM1->CCR4 = Phase_2_Low *accelPedalValue_scaled;
  					TIM8->CCR1 = Phase_3_High *accelPedalValue_scaled;
  					TIM8->CCR2 = Phase_3_Low *accelPedalValue_scaled;
          }
					// if (Phase_1_High) {
          //   TIM1->CCR2 = 0;
					//   TIM1->CCR1 = accelPedalValue_scaled;
					// } else {
					//   TIM1->CCR1 = 0;
					//   TIM1->CCR2 = accelPedalValue_scaled;
					// }
          //
					// if (Phase_2_High) {
          //   TIM1->CCR4 = 0;
          //   TIM1->CCR3 = accelPedalValue_scaled;
					// } else {
					//   TIM1->CCR3 = 0;
					//   TIM1->CCR4 = accelPedalValue_scaled;
					// }
          //
					// if (Phase_3_High) {
          //   TIM8->CCR2 = 0;
          //   TIM8->CCR1 = accelPedalValue_scaled;
					// } else {
					//   TIM8->CCR1 = 0;
					//   TIM8->CCR2 = accelPedalValue_scaled;
					// }

					//HAL_TIM_Base_Start_IT(&htim1);

					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
					HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

					HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

				} else {
					//something is wrong with the hall sensors.. STOPPP

					systemState = 1;
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

					HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
					HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
				}
			}
    }


		diff = globalHeartbeat_50us -  heartbeat_1ms;
		if (diff & 0x80000000) {
			diff = ~diff + 1;
		}

		if (diff > 20) {
  		heartbeat_1ms = globalHeartbeat_50us;
  		//1ms stuff
      speedError = demandedSpeed - measuredSpeed;

      //controlOutput = (motorSpeedConstant*(speedError*Kp + speedErrorSum*Ki));

      // if (controlOutput <= supplyVoltage) {
      //   controlOutput = 4200*controlOutput/supplyVoltage;
      // } else {
      //   controlOutput = 4200;
      // }

      if (speedError >= 0) {
        controlOutput = (motorSpeedConstant*(speedError*Kp + speedErrorSum*Ki));
        if (controlOutput <= supplyVoltage) {
          controlOutput = 4200*controlOutput/supplyVoltage;
        } else {
          controlOutput = 4200;
        }
      } else {
        controlOutput = (motorBrakeConstant*(speedError*Kp + speedErrorSum*Ki));
        if (abs(controlOutput) <= supplyVoltage) {
          controlOutput = 4200*controlOutput/supplyVoltage;
        } else {
          controlOutput = -4200;
        }
      }

      speedErrorSum += speedError;
  		//speed regulator
  		//over current protection

  	}

		diff = globalHeartbeat_50us -  heartbeat_10ms;
		if (diff & 0x80000000) {
     diff = ~diff + 1;
		}

  	if (diff > 200) {
      heartbeat_10ms = globalHeartbeat_50us;
  		//10ms stuff
  		//get pedal values

  		brakePedalVlaue_raw = HAL_ADC_GetValue(&hadc2);
			if (brakePedalVlaue_raw <= brakeMin_in)
				brakePedalVlaue_scaled = 0;
			else
				brakePedalVlaue_scaled = (int)(4200*((float)(brakePedalVlaue_raw - brakeMin_in)/brakeRange));
//
//      if ((brakePedalVlaue_scaled<50)) {
//        brakePedalVlaue_scaled = 0;
//      }


  		accelPedalValue_raw = HAL_ADC_GetValue(&hadc1);

			if (accelPedalValue_raw <= accelMin_in) {
				accelPedalValue_scaled = 0;
        demandedSpeed = 0;
			} else {
				accelPedalValue_scaled = (int)(4200*((float)(accelPedalValue_raw-accelMin_in)/accelRange));
        demandedSpeed = (int)(maxMotorSpeed*((float)(accelPedalValue_raw-accelMin_in)/accelRange));
			}

  		if (accelPedalValue_scaled<30) {
  			accelPedalValue_scaled = 0;
  		}

  		gearForward = (bool)HAL_GPIO_ReadPin(Fw_Rev_switch_GPIO_Port, Fw_Rev_switch_Pin);

  		HAL_ADC_Start(&hadc1);
  		HAL_ADC_Start(&hadc2);
  		//slew rate limiting for velocity (acceleration/deceleration control)
			LED_stateMachine();
  	}

    //LED_stateMachine();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4200;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 4200;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim8);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Hall1_Pin Hall2_Pin Hall3_Pin */
  GPIO_InitStruct.Pin = Hall1_Pin|Hall2_Pin|Hall3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Fw_Rev_switch_Pin */
  GPIO_InitStruct.Pin = Fw_Rev_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Fw_Rev_switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#include <stdbool.h>
#include <stdlib.h>

extern const uint8_t BRIDGE_STEPS_FORWARD[8][6];
extern const uint8_t BRIDGE_STEPS_REVERSE[8][6];
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern uint32_t globalHeartbeat_50us;
extern uint32_t led_state;

void readHallSensors(bool* Halls){
	*(Halls + 0) = HAL_GPIO_ReadPin(GPIOD, Hall1_Pin);
	*(Halls + 1) = HAL_GPIO_ReadPin(GPIOD, Hall2_Pin);
	*(Halls + 2) = HAL_GPIO_ReadPin(GPIOD, Hall3_Pin);
}

void getHallPosition(bool Halls[3], uint8_t* hallPosition){
	(*hallPosition) = (Halls[0]<<2) + (Halls[1]<<1) + (Halls[2]);
}

void getPhasesForward(bool* Phases, uint8_t hallPosition){
	for(int i = 0; i < 6; i++){
		*(Phases + i) = BRIDGE_STEPS_FORWARD[hallPosition][i];
	}
}

void getPhasesReverse(bool* Phases, uint8_t hallPosition){
	for(int i = 0; i < 6; i++){
		*(Phases + i) = BRIDGE_STEPS_REVERSE[hallPosition][i];
	}
}

void initPhases(bool* Phases){
	for(int i = 0; i < 6; i++){
		*(Phases + i) = false;
	}
}

void initHalls(bool* Halls){
	*(Halls + 0) = true;
	*(Halls + 1) = true;
	*(Halls + 2) = false;
}

void setDutyCiclePWM(bool Phases[6], int pedalValue){
	TIM1->CCR1 = Phases[0] * pedalValue;
	TIM1->CCR2 = Phases[1] * pedalValue;
	TIM1->CCR3 = Phases[2] * pedalValue;
	TIM1->CCR4 = Phases[3] * pedalValue;
	TIM8->CCR1 = Phases[4] * pedalValue;
	TIM8->CCR2 = Phases[5] * pedalValue;
}

void setBrakingDutyCiclePWM(int pedalValue){
	TIM1->CCR1 = 0;
	TIM1->CCR3 = 0;
	TIM8->CCR1 = 0;
	TIM1->CCR2 = pedalValue;
	TIM1->CCR4 = pedalValue;
	TIM8->CCR2 = pedalValue;
}

void setNullDutyCiclePWM(){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
	TIM8->CCR1 = 0;
	TIM8->CCR2 = 0;
}

void stopTimerPWM(){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
}

void startTimerPWM(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

bool checkHallSensorMalfunction(uint8_t hallPosition){
	return ((hallPosition > 6) || (hallPosition <1));
}

float scaleValue(uint16_t curr_val, uint16_t min_val, uint16_t range){
	return ((curr_val - min_val) / range);
}

uint16_t getRawBrakeValue(){
	return HAL_ADC_GetValue(&hadc2);
}

void getScaledBrakeValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range){
	uint16_t rawValue = getRawBrakeValue();
	if (rawValue <= min_val) {
		(*scaledValue) = 0;
	} else{
		(*scaledValue) = (int)(4200*scaleValue(rawValue, min_val, range));
	}
}

uint16_t getRawAccelValue(){
	return HAL_ADC_GetValue(&hadc1);
}

void getScaledAccelValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range){
	uint16_t rawValue = getRawAccelValue();
	if (rawValue <= min_val) {
		(*scaledValue) = 0;
	} else{
		(*scaledValue) = (int)(4200*scaleValue(rawValue, min_val, range));
	}
	if (*scaledValue < 30){
		(*scaledValue) = 0;
	}
}

void getGearForward(bool* gearForward){
	(*gearForward) = (bool)HAL_GPIO_ReadPin(Fw_Rev_switch_GPIO_Port, Fw_Rev_switch_Pin);
}

void startADC_HALs(){
	HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
}

void LED_stateMachine (uint8_t systemState, bool Halls[3]) {
    switch (systemState) {
			case 0: //Working normally
				if ((globalHeartbeat_50us - led_state) > 20000) {
					HAL_GPIO_TogglePin(GPIOD, LD4_Pin);
					led_state = globalHeartbeat_50us;
				}

				if (Halls[0])
					HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);

				if (Halls[1])
					HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);

				if (Halls[2])
					HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);

				break;

			case 1: //Hall effect problem
				if ((globalHeartbeat_50us - led_state) > 10000) {
					HAL_GPIO_WritePin(GPIOD, LD4_Pin,GPIO_PIN_RESET);

					//toggle hall effect LEDs
					HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
					HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
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

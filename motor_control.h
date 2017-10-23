#include <stdbool.h>
#include <stdlib.h>

extern const uint8_t BRIDGE_STEPS_FORWARD[8][6]; //Motor Phases for forward gear
extern const uint8_t BRIDGE_STEPS_REVERSE[8][6]; //Motor Phases for reverse gear

extern TIM_HandleTypeDef htim1; //Timer for PWM gates 1,2,3,4
extern TIM_HandleTypeDef htim8; //Timer for PWM gates 5,6

extern ADC_HandleTypeDef hadc1; //ADC for Acceleration Pedal
extern ADC_HandleTypeDef hadc2; //ADC for Brake Pedal

extern uint32_t globalHeartbeat_50us; //Main Global Heartbeat 
extern uint32_t led_state; //To flash State Machine LED

/*
	Description: samples all 3 Hall Sensors and stores the values into the input boolean array 'Halls'
	
	Input parameters: 	bool* Halls - sampled hall sensor data will be stored in this array
													Halls[0] = hall sensor 1
													Halls[1] = hall sensor 2
													Halls[2] = hall sensor 3
	
	Return value: None
*/
void readHallSensors(bool* Halls){
	*(Halls + 0) = HAL_GPIO_ReadPin(GPIOD, Hall1_Pin);
	*(Halls + 1) = HAL_GPIO_ReadPin(GPIOD, Hall2_Pin);
	*(Halls + 2) = HAL_GPIO_ReadPin(GPIOD, Hall3_Pin);
}

/*
	Description: computes the hall sensor position given the input sampled hall sensor data
	The hall position serves as an input for the bridge look up table. (BRIDGE_STEPS_REVERSE, 
	BRIDGE_STEPS_FORWARD)
	
	Input parameters: 	bool* Halls - array containing sample hall sensor data
													Halls[0] = hall sensor 1
													Halls[1] = hall sensor 2
													Halls[2] = hall sensor 3
											unit8_t* hallPosition - computed hall position will be stored here
	
	Return value: None
*/
void getHallPosition(bool Halls[3], uint8_t* hallPosition){
	(*hallPosition) = (Halls[0]<<2) + (Halls[1]<<1) + (Halls[2]);
}


/*
	Description: computes all 6 phases given the hall position by looking into the look up table
	stored in BRIDGE_STEPS_FORWARD
	
	Input parameters: 	bool* Phases - the 6 computed phases will be stored here
														Phases[0] = Phase 1 High
														Phases[1] = Phase 1 Low
														Phases[2] = Phase 2 High
														Phases[3] = Phase 2 Low
														Phases[4] = Phase 2 High
														Phases[5] = Phase 2 Low
											unit8_t* hallPosition - contains the computed hall position
	
	Return value: None
*/
void getPhasesForward(bool* Phases, uint8_t hallPosition){
	for(int i = 0; i < 6; i++){ //For all 6 phases
		*(Phases + i) = BRIDGE_STEPS_FORWARD[hallPosition][i]; //Access look up table 
	}
}

/*
	Description: computes all 6 phases given the hall position by looking into the look up table
	stored in BRIDGE_STEPS_REVERSE
	
	Input parameters: 	bool* Phases - the 6 computed phases will be stored here
														Phases[0] = Phase 1 High
														Phases[1] = Phase 1 Low
														Phases[2] = Phase 2 High
														Phases[3] = Phase 2 Low
														Phases[4] = Phase 2 High
														Phases[5] = Phase 2 Low
											unit8_t* hallPosition - contains the computed hall position
	
	Return value: None
*/
void getPhasesReverse(bool* Phases, uint8_t hallPosition){
	for(int i = 0; i < 6; i++){ //For all 6 phases
		*(Phases + i) = BRIDGE_STEPS_REVERSE[hallPosition][i]; //Access look up table
	}
}

/*
	Description: initializes the array containing all 6 phases to false (motor will not move)
	
	Input parameters: 	bool* Phases - the 6 phases will be stored here
														Phases[0] = Phase 1 High
														Phases[1] = Phase 1 Low
														Phases[2] = Phase 2 High
														Phases[3] = Phase 2 Low
														Phases[4] = Phase 2 High
														Phases[5] = Phase 2 Low
	
	Return value: None
*/
void initPhases(bool* Phases){
	for(int i = 0; i < 6; i++){
		*(Phases + i) = false;
	}
}

/*
	Description: initializes the array containing the hall sensor data to an arbitrary value
	
	Input parameters: 	bool* Halls - array containing sample hall sensor data
													Halls[0] = hall sensor 1
													Halls[1] = hall sensor 2
													Halls[2] = hall sensor 3
	
	Return value: None
*/
void initHalls(bool* Halls){
	*(Halls + 0) = true;
	*(Halls + 1) = true;
	*(Halls + 2) = false;
}

/*
	Description: sets the duty cicle of all 6 PWM signals according to the computed phases.
	The duty cicle is given by pedalValue (should be sclaed, that is, between 0 and 4200)
	
	Input parameters: 	bool* Phases - array containing the 6 phases
														Phases[0] = Phase 1 High
														Phases[1] = Phase 1 Low
														Phases[2] = Phase 2 High
														Phases[3] = Phase 2 Low
														Phases[4] = Phase 2 High
														Phases[5] = Phase 2 Low
											int pedalValue - int containing the scaled pedal value (between 0 and 4200)
	
	Return value: None
*/
void setDutyCiclePWM(bool Phases[6], int pedalValue){
	TIM1->CCR1 = Phases[0] * pedalValue; //Phase 1 High
	TIM1->CCR2 = Phases[1] * pedalValue; //Phase 1 Low
	TIM1->CCR3 = Phases[2] * pedalValue; //Phase 2 High
	TIM1->CCR4 = Phases[3] * pedalValue; //Phase 2 Low
	TIM8->CCR1 = Phases[4] * pedalValue; //Phase 3 High
	TIM8->CCR2 = Phases[5] * pedalValue; //Phase 3 Low
}

/*
	Description: sets the duty cicle of the 3 PWM signals going to the high phases to 0, and the 3 PWM
	signals going to the low phases to pedalValue (should be sclaed, that is, between 0 and 4200)
	
	Input parameters: 	bool* Phases - array containing the 6 phases
														Phases[0] = Phase 1 High
														Phases[1] = Phase 1 Low
														Phases[2] = Phase 2 High
														Phases[3] = Phase 2 Low
														Phases[4] = Phase 2 High
														Phases[5] = Phase 2 Low
											int pedalValue - int containing the scaled pedal value (between 0 and 4200)
	
	Return value: None
*/
void setBrakingDutyCiclePWM(int pedalValue){
	TIM1->CCR1 = 0; //Phase 1 High
	TIM1->CCR3 = 0; //Phase 2 High
	TIM8->CCR1 = 0; //Phase 3 High
	TIM1->CCR2 = pedalValue; //Phase 1 Low
	TIM1->CCR4 = pedalValue; //Phase 2 Low
	TIM8->CCR2 = pedalValue; //Phase 3 Low
}

/*
	Description: sets the duty cicle of all 6 PWM signals to 0
	
	Input parameters: 	None
	
	Return value: None
*/
void setNullDutyCiclePWM(){
	TIM1->CCR1 = 0; //Phase 1 High
	TIM1->CCR2 = 0; //Phase 1 Low
	TIM1->CCR3 = 0; //Phase 2 High
	TIM1->CCR4 = 0; //Phase 2 Low
	TIM8->CCR1 = 0; //Phase 3 High
	TIM8->CCR2 = 0; //Phase 3 Low
}

/*
	Description: stops the timers of all 6 PWM signals
	
	Input parameters: 	None
	
	Return value: None
*/
void stopTimerPWM(){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); //Phase 1 High
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2); //Phase 1 Low
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3); //Phase 2 High
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4); //Phase 2 Low
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1); //Phase 3 High
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2); //Phase 3 Low
}

/*
	Description: starts the timers of all 6 PWM signals
	
	Input parameters: 	None
	
	Return value: None
*/
void startTimerPWM(){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Phase 1 High
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //Phase 1 Low
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //Phase 2 High
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //Phase 2 Low
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //Phase 3 High
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); //Phase 3 Low
}

/*
	Description: check if the hall position is in an impossible state (therefore malfunctioning)
	Invalid states are 0 (000) and 7 (111).
	
	Input parameters: 	unit8_t* hallPosition - computed hall position
	
	Return value:		bool - True if hall sensors are in an impossible state, False otherwise
*/
bool checkHallSensorMalfunction(uint8_t hallPosition){
	return ((hallPosition > 6) || (hallPosition <1));
}

/*
	Description: scale a value given the minimum value it can take and its range (difference between
	maximum and minimum possible values). For instance, value 1 with minum value 0 and range 2 is 0.5
	
	Input parameters: 	uint16_t curr_val
	
	Return value:		bool - True if hall sensors are in an impossible state, False otherwise
*/
float scaleValue(uint16_t curr_val, uint16_t min_val, uint16_t range){
	return ((curr_val - min_val) / range);
}

/*
	Description: samples the brake pedal (hadc2)
	
	Input parameters: 	None
	
	Return value:		uint16_t - sampled brake pedal value
*/
uint16_t getRawBrakeValue(){
	return HAL_ADC_GetValue(&hadc2);
}

/*
	Description: returns the brake pedal value scaled to the maximum duty cicle (between 0 and max duty cicle)
	
	Input parameters: 	uint16_t scaledValue - the scaled brake pedal value will be stored here
											uint16_t min_val - the minimum value that the raw pedal value can have
											uint16_t range - the difference between the raw brake pedal maximum value and minimum value
	
	Return value:		None
*/
void getScaledBrakeValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range){
	uint16_t rawValue = getRawBrakeValue(); //Sample raw value
	if (rawValue <= min_val) {
		(*scaledValue) = 0;
	} else{
		(*scaledValue) = (int)(4200*scaleValue(rawValue, min_val, range)); //Scale
	}
}

/*
	Description: samples the acceleration pedal (hadc1)
	
	Input parameters: 	None
	
	Return value:		uint16_t - sampled acceleration pedal value
*/
uint16_t getRawAccelValue(){
	return HAL_ADC_GetValue(&hadc1);
}

/*
	Description: returns the acceleration pedal value scaled to the maximum duty cicle (between 0 
	and max duty cicle)
	
	Input parameters: 	uint16_t scaledValue - the scaled acceleration pedal value will be stored here
											uint16_t min_val - the minimum value that the raw acceleration value can have
											uint16_t range - 	the difference between the raw acceleration pedal maximum 
																				value and minimum value
	
	Return value:		None
*/
void getScaledAccelValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range){
	uint16_t rawValue = getRawAccelValue(); //Sample raw
	if (rawValue <= min_val) {
		(*scaledValue) = 0;
	} else{
		(*scaledValue) = (int)(4200*scaleValue(rawValue, min_val, range)); //Scale
	}
	if (*scaledValue < 30){ //Additional threshold to avoid accidental acceleration
		(*scaledValue) = 0;
	}
}

/*
	Description: samples the gear forward/backward switch (Fw_Rev_switch_Pin)
	
	Input parameters: 	bool gearForward - the gear forward/backward switch value will be stored here
	
	Return value:		None
*/
void getGearForward(bool* gearForward){
	(*gearForward) = (bool)HAL_GPIO_ReadPin(Fw_Rev_switch_GPIO_Port, Fw_Rev_switch_Pin);
}

/*
	Description: enables sampling the brake and acceleration pedals (hadc1, hadc2)
	
	Input parameters: 	None
	
	Return value:		None
*/
void startADC_HALs(){
	HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
}

/*
	Description: uses the on-board LEDs to give information about the state of the system (for
	debugging purposes)
	
	Input parameters: 	None
	
	Return value:		None
*/
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

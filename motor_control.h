#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>

extern TIM_HandleTypeDef htim1; //Timer for PWM gates 1,2,3,4
extern TIM_HandleTypeDef htim8; //Timer for PWM gates 5,6

extern ADC_HandleTypeDef hadc1; //ADC for Acceleration Pedal
extern ADC_HandleTypeDef hadc2; //ADC for Brake Pedal

extern uint32_t globalHeartbeat_50us; //Main Global Heartbeat 
extern uint32_t hallLED_state; //To flash State Machine LED


// FUNCTIONS FOR SAMPLING PEDALS AND FORWARD/REVERSE GEAR
void startADC_HALs(void);
uint16_t getRawBrakeValue(void);
uint16_t getRawAccelValue(void);
float scaleValue(uint16_t curr_val, uint16_t min_val, uint16_t range);
void getScaledBrakeValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range);
void getScaledAccelValue(uint16_t* scaledValue, uint16_t min_val, uint16_t range);
void getGearForward(bool* gearForward);

// FUNCTIONS FOR COMPUTING THE PHASES
void initPhases(bool* Phases);
void initHalls(bool* Halls);
bool checkHallSensorMalfunction(uint8_t hallPosition);
void readHallSensors(bool* Halls);
void getHallPosition(bool Halls[3], uint8_t* hallPosition);
void getPhasesForward(bool* Phases, uint8_t hallPosition);
void getPhasesReverse(bool* Phases, uint8_t hallPosition);

// FUNCTIONS FOR CONTROLLING THE PWM
void stopTimerPWM(void);
void startTimerPWM(void);
void setNullDutyCiclePWM(void);
void setDutyCiclePWM(bool Phases[6], int dutyValue);
void setBrakingDutyCiclePWM(int dutyValue);
void LED_stateMachine (uint8_t systemState, bool Halls[3], uint32_t globalHeartbeat_50us, 
	uint32_t hallLED_state);

// PID FUNCTIONS
void computeHallSpeed(int* measuredSpeed, uint32_t globalHeartbeat_50us, uint8_t hallPosition, 
			uint32_t* hallEffectTick, uint8_t* lastHallPosition);
void getDamandedSpeed(int* demandedSpeed, int accelPedalValue_scaled, int maxMotorSpeed);
void scaleSaturationInt(int* val, int min_val, int max_val);
void getControlOutput(int* controlOutput, int demandedSpeed, int measuredSpeed, float actuatorSaturationPoint, 
	float* speedErrorSum, float Kp, float Ki);
void getDemandedPWM(int* demandedPWMdutyCicle, int controlOutput, float motorSpeedConstant, uint8_t supplyVoltage);
void getActuatorSaturationPoint(float* actuatorSaturationPoint, int supplyVoltage, float motorSpeedConstant);

#endif

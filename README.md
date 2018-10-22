# Motor controller - STM32F4

Software for an STM32F407GT Discovery Board acting as a motor controller for a brushless DC motor. The motor controller takes inputs from the acceleration and break pedals and outputs the corresponding PWM signals to drive the motor.

- [Inputs and outputs](#inputs-and-outputs)
- [Functionality](#functionality)
- [Improvements required](#improvements-required)

Code devloped by Sheffield Eco Motorsports.

# Inputs and outputs

Inputs:
 * Acceleration and break pedals (2 analog inputs)
 * Hall effect sensors (3 digital inputs)

Outputs:
 * 6 PWM signals going into motor circuitry (will drive MOSFETs which power motor)

# Functionality

When the break pedal is pressed, the PWM signals are set to energise those phases which allow regenerative braking, with duty cycle proportional to how much the pedal is being pressed. When the acceleration pedal is being pressed, the setpoint of the PID velocity controller is modulated proportionally from 0 to `maximum velocity` according to how much the pedal is being pressed. The PID controller takes as its input the velocity of the stator as measured by the hall effects (time between ticks), and outputs the duty cycle of the PWMs. Some of the PWM duty cycles will be set to 0, as different phases must be energised according to the position of the rotor (measured by the hall effect sensors). This is done using a standard look-up table.

The program flow is structured by polling three sections of code at different frequencies:
 * Every `10ms` the brake and acceleration pedals are sampled, and the pid setpoint is set if appropriate.
 * Every `1ms` the PID controller is sampled, computting ``demandedPWM``.
 * Every `100us`
   * If safety conditions are being violated or the brake pedal was being pressed the phases are set to braking mode.
   * Otherwise the hall effects are sampled and the duty cycle is set to ``demandedPWM``.

# Improvements required

Short-term improvements:
 * PID controller modifications:
  * Tune PID to ensure relatively fast responses while not overshooting.
  * Test if integral anti-windup measures work as expected.
  * Sample PID at the same frequency as the speed measurement.
 * Test that motor can start from all positions, and determine if fault is at a hardware level due to faulty wire conexions or the phase look-up table is partially incorrect.

Long-term improvements:
 * Use interrupts to change PWM phases as fast as possible when any given hall effect changes its state.
 * Use hardware timers or possible and RTOS to avoid constantly polling.


# Sheffield Eco Motorsports Control Team Repository

This version builds on top of the barebones code (similar to Chatura's but with a header file containing the necessary functions) by adding PI velocity control.

Additions:
  * Function to compute the shaft velocity from the hall sensor data. Called every 100us.
  * PI control function called every 1ms.
  * Function to turn the acceleration pedal value into demanded speed for the PI.

Code developed by Sheffield Eco Motorsport Controls Team.

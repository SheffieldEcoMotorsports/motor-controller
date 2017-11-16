# Sheffield Eco Motorsports Control Team Repository

This version builds on top of the pid code by adding anti integral windup measures.

Problems fixed:
  * demandedSpeed - when pressed there is no smooth transition, either zero values (even when significantly pressed) or almost maximum value. There was a problem with typecasting from int to float. Now works smoothly, similar to the acceleration pedal analog input.
  * speedMeasured - fixed initial readings. Now the hall sensor properly initializes.

Problems remaining:
  * speedMeasurement - very jumpy signal. Amplitude of readings too high (12000RPM). Does not read completely 0.
  * controlOutput - does not work due to faulty demandedSpeed
  * demandedPWM - motor brake constant/motor accel constant need to be adjusted.

  
Code developed by Sheffield Eco Motorsport Controls Team.

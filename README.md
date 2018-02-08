# Sheffield Eco Motorsports Control Team Repository

This version builds on top of the pid code by adding anti integral windup measures.

What to work on:
  * speedMeasurement - very jumpy signal. Amplitude of readings too high (12000RPM). Does not read completely 0
  * controlOutput - does not work due to faulty demandedSpeed
  * demandedPWM - motor brake constant/motor accel constant need to be adjusted
  
Code developed by Sheffield Eco Motorsport Controls Team.

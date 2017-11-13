# Sheffield Eco Motorsports Control Team Repository

This version builds on top of the pid code by adding anti integral windup measures.

Problems identified with the code:
  * demandedSpeed - when pressed there is no smooth transition, either zero values (even when significantly pressed) or almost maximum value
  * controlOutput - does not work due to faulty demandedSpeed
  * demandedPWM - does not work due to faulty demandedPWM
  * speedMeasured - does not start at 0. When motor is used and then brough back to 0 the velocity does not go back to 0 (hall effects not updating). The velocity measured is too large (arround 10,000 in RPMS). Velocity fluctuates heavily even when pressed to maximum speed.
  
Changed:
  * Added float typecasting to getDemandedSpeed
  * Added float typecasting to computeHallSpeed
  
Code developed by Sheffield Eco Motorsport Controls Team.

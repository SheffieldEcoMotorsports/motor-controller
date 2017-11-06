# Sheffield Eco Motorsports Control Team Repository

This version builds on top of the pid code by adding anti integral windup measures. Does not use global variables (except the hearbeats).

Additions:
  * Function to calculate the PI output that causes actuator saturation.
  * Modified the PI function to account for anti integral windup.

Code developed by Sheffield Eco Motorsport Controls Team.

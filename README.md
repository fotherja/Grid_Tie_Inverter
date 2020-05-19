# Grid_Tie_Inverter
An attempt to make my own GTI with acceptable output harmonics.
<br><br>
This is our setup: <br>
 - An STM32F407<br>
 - A DRV8301 Boost board for it's H-Bridge<br>
 - An LC output filter<br>
 - Step down transformer<br>
<br>
<br>
We use feedforward control in addition to a PI controller to obtain a given current setpoint.

The system outputs 40Watts with <5% THD 

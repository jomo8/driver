# TODO
1) Clean the code, add some comments.

2) Develop uart write and read fubctions using ring buffer.

3) Configure i2c communication read and write functions. use a second ring buffer.

4) Develop gcode interpreter for both uart and i2c.

5) Develop a function to save and read bytes to flash memory.

6) Test the code for high speed motor, check if mcu misses any encoder pulses.

7) Add gcode commands to set pid coefficients and to save them into flash memory.

8) Improve and optimize pid controller.

9) Figure out how to enable brownout detector. Write code to autosave last shaft position at brownout.

10) Configure ADC's to measure hbridge currents and send current data over uart or i2c to the master device.

11) Add votage measurement unit to the board. use an ADC pin to measure power supply voltage and send the readings to master over uart and i2c.

Stretch goal:
A) Build a fuzzy logic controller.



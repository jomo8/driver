# TODO
1) Clean the code, add some comments 

2) Develop uart write and read fubctions using ring buffer.

3) Configure i2c communication read and write functions. use a second ring buffer.

4) Develop gcode interpreter for both uart and i2c.

5) Develop a function to save and read bytes to flash memory.

6) Test the code for high speed motor, check if mcu misses any encoder pulses.

7) Add gcode commands to set pid coefficients and to save them into flash memory.

8) Improve and optimize pid controller.

9) figure out how to enable brownout detector. Write code to autosave last shaft position at brownout.

stretch goals:
10) build a fuzzy logic controller.



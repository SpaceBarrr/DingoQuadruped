# Dingo Servo Board
## Intro
The Dingo servoboard is a PWM based servo driver, with integrated current monitoring.

## Usage
- One board per side of the quadruped (left and right).
- An I2C line has to be connected to the Pi (either directly or via another peripheral). 
- Add the bridge at RXX to change the I2C address depending on whether it is the L or R board.
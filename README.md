# Rover
Controller software written for the iRobot platform in conjunction with an ATMega128 microcontroller

This is a bundle of software created for use with the iRobot Create. Rover contains basic function libraries for use with the on-board ATMega128 microcontroller, allowing for user control over movement, environment detection, audio output. Rover allows for bluetooth control via a PuTTY terminal, and serial communication via the Atmel Studio development tool. 

Rover was submitted in April 2015 as a final project for my CprE 288 course (Into to Embedded Systems) with Collin Farrell and Nicholas Boos.

## Use
Use of the Rover software requires an iRobot Create platform, Atmel Studios, PuTTY a bluetooth module, and a serial connection. Using Atmel Studios, build and upload all code to the iRobot using a serial connection. Then, by modifying the BAUD rate found in the Rover.c file, enable a bluetooth connection via a PuTTY terminal. Running Rover.c on the robot will allow commands to be sent via the terminal. Available commands can be found in RemoteControl.c file.

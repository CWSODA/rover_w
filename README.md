# EXPERIMENT BRANCH
This version was used for the individual experiment that tested the rotation speed and angular resolution of the LiDAR module. All settings have been preserved for reproducibility. This branch will NOT be updated.

# ROVER DESIGN CODE
Contains code for an autonomous robot built for the 2nd year rover design module at University of Southampton.

# Specifications
- Able to navigate indoor and outdoor terrain
- Autonomous obstacle avoidance

# Build Instructions
Install the Raspberry Pi Pico extension in VSCode (installs the required C SDK). \
Choose an appropriate Pico board (developed on Pico W but should work on other boards with correct pin changes)\
Simply compile with the compile/run button. \
When flashing the pico, hold the bootsel button down when plugging the pico into the computer to put it into boot mode.

# Notes
Pinout connections are in header files
Defines are used to toggle debug messaging over uart0

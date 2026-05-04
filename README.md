# ROVER DESIGN CODE
Contains code for an autonomous robot built for the 2nd year rover design module at University of Southampton.

# Specifications
- Able to navigate indoor and outdoor terrain
- Autonomous obstacle avoidance
- Can be controlled with a dedicated webapp at https://github.com/CWSODA/rover_webapp

# Build Instructions
Install the Raspberry Pi Pico extension in VSCode (installs the required C SDK). \
Choose an appropriate Pico board (developed on Pico W but should work on other boards with correct pin changes)\
Simply compile with the compile/run button. \
When flashing the pico, hold the bootsel button down when plugging the pico into the computer to put it into boot mode.

# Notes
All settings are found in the "settings.hpp" file, including PINOUTs and DEBUG selection settings
Can be debuged with uart0, USB, or through Wifi (TCP)

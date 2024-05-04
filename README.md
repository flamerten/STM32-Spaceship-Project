# STM32 Spaceship Project

This is the repository for the firmware of Assignment 2 of a module I took in NUS, [EE2028 - Microcontroller Interfacing and Programming](https://nusmods.com/courses/EE2028/microcontroller-programming-and-interfacing) in AY2023/2024 semester 1.

For this assignment the STM32 acts as a spaceship that has to display various outputs on UART when certain condtions are met. For example, pushing a button, or turning the board upside down. 

# Core Features
The core features refer to implementations that are compulsory for all students of the module.

1. Double push button press detection using interrupts
2. Telementary data sending using the UART protocol to the computer, read via PuTTY.
3. Interfacing the STM32 board with onboard sensors and reading their data. The sensors are a magnometer, 6-DOF inertial measuremnet unit, a pressure and temperature sensor.

# Additional Features
1. Interfacing the STM32 with an OLED module, driven by the SSD1306 driver.
2. Implementation of the complementary filter to calulcate tilt angle from inertial measurement unit readings.
3. Graphical Display of the tilt on the OLED screen, which is a line that tilts clockwise or anticlockwise depending on the current tilt.
4. Triple push button press detection to activate or deactive the OLED screen.
5. Interfacing the STM32 with an onboard buzzer.
6. Implementing interrupts on the intertial measurement unit such that the buzzer gives of a sound when the sensor tilts more than 30 degrees suddenly.
# STM32-F103C8T6 Project 

A blinking LED with a fixed on/off interval should be implemented using a timer. The initial value should be 100ms, and the time interval should be adjustable.

Every 500ms, a fixed-length HEX protocol with the following "Protocol Information" will be sent over UART.
If a "Time Data" is received over UART in a format specified by you, the LED flash time will be set to this value. (For example, if UART INCOMING DATA = 497, flash_time will be 497ms.)
The entered time value should be stored in "Flash Memory" to prevent it from being lost when the device is rebooted.
If an "External Interrupt" occurs through a button, the LED should turn off and data transfer over UART should be stopped.
The process should be able to continue if the button is pressed again.
If the button is held down for 3 seconds, the system will be reset to its default values.

Protocol Information:

LED Status
Total flash time of the LED
Time elapsed since the LED's last state change
Remaining time until the LED changes to its next state
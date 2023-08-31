# Sensirion Differential Pressure Sensor SDP8x Digital Driver

## Getting Started
Create an sdp8x_handler_t instance in main. Implement the *sdp8x_transmit, *sdp8x_receive, and *delay functions in main. Call sdp8x_init().

## Communicating With the Sensor
The SDP8x Digital sensor uses an I2C interface, the address is 0x25. The functions for retrieving data are defined in the sdp8x_digital.h file. 

## Debug Output
A precompiler directive is used to turn debug output on and off. Currently all of the outputs are using NRF_LOG_INFO which is a nordic semi SDK specific function, change these to printf or whatever your micro environment uses. 

# Sensirion Differential Pressure Sensor SDP8x Digital Driver

## Getting Started
Create an `sdp8x_handler_t` instance in main. Implement the `*sdp8x_transmit`, `*sdp8x_receive`, and `*delay` functions in main. Call `sdp8x_init()`.

Put this in your super loop in main. `i2c_xfer_done` should be a flag set to true in your I2C event handler.
```
if(i2c_xfer_done) {
    sdp8x_read_data(&sdp8x); // sdp8x is your sdp8x_handler_t instance
    i2c_xfer_done = false;
}
```

## Communicating With The Sensor
The SDP8x Digital sensor uses an I2C interface, the address is `0x25`. The `*sdp8x_transmit` and `*sdp8x_receive` functions should use I2C. The functions for retrieving data from the sensor are defined in the sdp8x_digital.h file. Use pullups on the SCL and SDA lines.

## Debug Output
A precompiler directive is used to turn debug output on and off. Currently all of the outputs are using `NRF_LOG_INFO` which is a Nordic nRF5 SDK specific function, change these to printf or whatever your micro environment uses. 

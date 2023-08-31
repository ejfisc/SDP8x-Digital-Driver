/* ************************************************************************** */
/** Sensirion SDP8x Digital Differential Pressure Sensor Function Library

  @Company
    Myriad Sensors

  @File Name
    sdp8x_digital.c

  @Summary
    Driver Library for Sensirion SDP8x Differential Pressure Sensor

  @Description
    Implements functions that allow the user to interact with the sensor
 **************************************************************************** */

#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include "sdp8x_digital.h"

static bool init_done = false; // used so that during the initialization process, the sdp8x_data_handler() does not process data in sdp8x_data that does not exist yet

/*******************************[ High-Level Sensor Functions For General Use ]****************************************/

/*
    @brief Function for initializing the SDP8x Differential Pressure Sensor

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_init(sdp8x_handler_t * sdp8x) {
#ifdef DEBUG_OUTPUT
    NRF_LOG_INFO("Sensirion SDP8x Differential Pressure Sensor Initialization:");
    NRF_LOG_FLUSH();
#endif

    // reset measurement values
    sdp8x->current_diff_pressure = 0;
    sdp8x->current_temperature = 0;

    sdp8x_stop_continuous_measurement(sdp8x); // stop continuous measurement

    sdp8x_read_product_info(sdp8x); // request and print product info

    sdp8x_start_continuous_measurement(SDP8X_CONT_MODE0, sdp8x); // start continuous measurement mode

    init_done = true;
}

/*
    @brief Function for starting continuous measurement mode with the given mode
    
    @param[in] mode Desired mode for continuous measurement

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_start_continuous_measurement(sdp8x_cont_mode_t mode, sdp8x_handler_t * sdp8x) {
    uint8_t reg[2] = {0};
    reg[0] = 0x36;
#ifdef DEBUG_OUTPUT
    NRF_LOG_INFO("Continuous Measurement Mode: ");
    NRF_LOG_FLUSH();
#endif
    switch(mode) {
	case SDP8X_CONT_MODE0:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Mass Flow, Averaging - Average till read");
	    NRF_LOG_FLUSH();
#endif	
	    reg[1] = 0x03;
	    break;
	case SDP8X_CONT_MODE1:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Mass Flow, Averaging - None, update rate 0.5ms");
	    NRF_LOG_FLUSH();
#endif	
	    reg[1] = 0x08;
	    break;
	case SDP8X_CONT_MODE2:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Differential Pressure, Averaging - Average till read");
	    NRF_LOG_FLUSH();
#endif	
	    reg[1] = 0x15;
	    break;
	case SDP8X_CONT_MODE3:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Differential Pressure, Averaging - None, update rate 0.5ms");
	    NRF_LOG_FLUSH();
#endif	
	    reg[1] = 0x1E;
	    break;
    }

    sdp8x->sdp8x_transmit(reg, 2, SDP8X_I2C_ADDR);
    sdp8x->delay(8);
}

/*
    @brief Function for stopping continuous measurement

    @note This command stops the continuous measurement and puts the sensor in idle mode.
	  It powers off the heater and makes the sensor receptive for another command after 500us. 
	  The Stop command is also required when switching between different continuous measurement commands

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_stop_continuous_measurement(sdp8x_handler_t * sdp8x) {
    uint8_t reg[2] = {0x3F, 0xF9};
    sdp8x->sdp8x_transmit(reg, 2, SDP8X_I2C_ADDR);
    sdp8x->delay(25);
#ifdef DEBUG_OUTPUT
    NRF_LOG_INFO("Stop Continuous Measurement");
    NRF_LOG_FLUSH();
#endif	
}

/*
    @brief Function for triggered measurement with the given mode

    @param[in] mode Desired mode for triggered measurement

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_triggered_measurement(sdp8x_triggered_mode_t mode, sdp8x_handler_t * sdp8x) {
    uint8_t reg[2] = {0};
    reg[0] = 0x36;
#ifdef DEBUG_OUTPUT
    NRF_LOG_INFO("Triggered Measurement Mode: ");
    NRF_LOG_FLUSH();
#endif
    switch(mode) {
	case SDP8X_TRIGGERED_MODE0:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Mass Flow, Clock stretching? - No");
	    NRF_LOG_FLUSH();
#endif	
	    reg[1] = 0x24;
	    break;
	case SDP8X_TRIGGERED_MODE1:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Mass Flow, Clock stretching? - Yes");
	    NRF_LOG_FLUSH();
#endif	
	    reg[0]++; // 0x37
	    reg[1] = 0x26;
	    break;
	case SDP8X_TRIGGERED_MODE2:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Differential Pressure, Clock stretching? - No");
	    NRF_LOG_FLUSH();
#endif	
	    reg[1] = 0x2F;
	    break;
	case SDP8X_TRIGGERED_MODE3:
#ifdef DEBUG_OUTPUT
	    NRF_LOG_INFO("Temperature compensation - Differential Pressure, Clock stretching? - Yes");
	    NRF_LOG_FLUSH();
#endif	
	    reg[0]++; // 0x37
	    reg[1] = 0x2D;
	    break;
    }

    sdp8x->sdp8x_transmit(reg, 2, SDP8X_I2C_ADDR); // transmit command
    sdp8x->delay(45); // wait 45ms
}

/*
    @brief Function for entering sleep mode

    @note Sleep mode can only be entered from idle mode, i.e. when the sensor is not measuring.
	  If in triggered mode, the sleep command can be sent after the result has been read and the sensor is idle.
	  If in continuous mode, the sleep command can be sent after the stop continous measurement 
	  command has been issued and the sensor is idle. In sleep mode the sensor cannot be reset.

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_enter_sleep_mode(sdp8x_handler_t * sdp8x) {
    uint8_t reg[2] = {0x36, 0x77};
    sdp8x->sdp8x_transmit(reg, 2, SDP8X_I2C_ADDR); // transmit command
}

/*
    @brief Function for exiting sleep mode
    
    @note The sensor exits the sleep mode and enters the idle mode when it receives the valid I2C address and a write bit (‘0’).
	  note that the I2C address is not acknowledged. The sensor should wake up within 20 ms. Polling with a write header
	  (I2C address and a write bit) can be used to check whether the sensor has woken up.

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_exit_sleep_mode(sdp8x_handler_t * sdp8x) {
    uint8_t reg[1] = {0x00};
    sdp8x->sdp8x_transmit(reg, 1, SDP8X_I2C_ADDR); // transmit command
    sdp8x->delay(20);
}

/*
    @brief Function for resetting the sensor.
    
    @note After the reset command the sensor will take maximum 20 ms to reset. 
	  During this time the sensor will not acknowledge its address nor accept commands.

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_soft_reset(sdp8x_handler_t * sdp8x) {
    uint8_t reg[1] = {0x06};
    uint8_t addr = 0x00;
    sdp8x->sdp8x_transmit(reg, 1, addr); // transmit command
    sdp8x->delay(20);
}

/*
    @brief Function for getting the product info

    @note The product identifier and serial number can be read out after sending a sequence of two commands, both preceded by a start condition.
	  Note that both commands need to be preceded with an I2C write header (I2C address + W).
	  The second command returns: - 32 bit unique product and revision
					number. Note that the last 8 bits are the revision
					number and can be subject to change.
				      - 64 bit unique serial number
    
    @note prints product number and serial number if debug output is on

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_read_product_info(sdp8x_handler_t * sdp8x) {
    uint8_t reg[2] = {0x36, 0x7C};
    sdp8x->sdp8x_transmit(reg, 2, SDP8X_I2C_ADDR); // transmit first command
    sdp8x->delay(10);

    reg[0] = 0xE1;
    reg[1] = 0x02;
    sdp8x->sdp8x_transmit(reg, 2, SDP8X_I2C_ADDR); // transmit second command
    sdp8x->delay(45);

    sdp8x->sdp8x_receive(sdp8x->sdp8x_product_info, 18, SDP8X_I2C_ADDR); // read product info
    sdp8x->delay(10);
    
    uint8_t product_num[4];
    sdp8x_get_bytes(sdp8x->sdp8x_product_info, 4, 0, product_num);
    union ByteArrayTo32BitInteger convert32 = {product_num[3], product_num[2], product_num[1], product_num[0]}; // these are backwards because of endianness
    sdp8x->product_number = convert32.int32;

#ifdef DEBUG_OUTPUT
    NRF_LOG_INFO("Product Number: 0x%x", sdp8x->product_number);
    NRF_LOG_FLUSH();
#endif
    
    uint8_t serial_num[8];
    sdp8x_get_bytes(sdp8x->sdp8x_product_info, 8, 6, serial_num);
    union ByteArrayTo64BitInteger convert64 = {serial_num[7], serial_num[6], serial_num[5], serial_num[4], serial_num[3], serial_num[2], serial_num[1], serial_num[0]}; // these are backwards because of endianness
    sdp8x->serial_number = convert64.int64;

#ifdef DEBUG_OUTPUT
    NRF_LOG_INFO("Serial Number: 0x%016llx\n", sdp8x->serial_number);
    NRF_LOG_FLUSH();
#endif
}

/*
    @brief Function for getting the product number

    @note the last 8 bits are the revision number and subject to change

    @return 32-bit product number

    @param[in] sdp8x Pointer of library handler
*/
uint32_t sdp8x_get_product_num(sdp8x_handler_t * sdp8x) {
    return sdp8x->product_number;
}

/*
    @brief Function for getting the serial number

    @return 64-bit unique Serial Number

    @param[in] sdp8x Pointer of library handler
*/
uint64_t sdp8x_get_serial_num(sdp8x_handler_t * sdp8x) {
    return sdp8x->serial_number;
}

/*
    @brief Function for reading measurement data

    @note Call this in your super loop

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_read_data(sdp8x_handler_t * sdp8x) {
    sdp8x->sdp8x_receive(sdp8x->sdp8x_data, 9, SDP8X_I2C_ADDR); // measurement data is sent in 9 bytes
}

/*
    @brief Function for getting x bytes of data out of the given sensor response buffer

    @param[in] buf Either sdp8x_data or sdp8x_product_info

    @param[in] num_bytes How many bytes to read from the buffer

    @param[in] start_idx Index to start at in the buffer

    @return byte array of the given size
*/
void sdp8x_get_bytes(uint8_t * buf, uint8_t num_bytes, uint8_t start_idx, uint8_t * bytes) {
    uint8_t i = start_idx;
    uint8_t cnt = 0;
    while(cnt < num_bytes) {
	bytes[cnt] = buf[i];
	cnt++;
	i++;
	if((i+1)%3 == 0) // skip every 3rd byte (CRC byte)
	    i++;
    }
}

/*
    @brief Function for processing data measurements

    @note prints data if debug output is on

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_data_handler(sdp8x_handler_t * sdp8x) {
    if(init_done) {
	uint8_t bytes[2];
        // get differential pressure scaling factor
        uint16_t scaling_factor = 0;
        sdp8x_get_bytes(sdp8x->sdp8x_data, 2, 6, bytes);
        union ByteArrayTo16BitInteger convert16 = {bytes[1], bytes[0]};
        scaling_factor = convert16.int16;

        // get differential pressure
        sdp8x_get_bytes(sdp8x->sdp8x_data, 2, 0, bytes);
        convert16.arr[0] = bytes[1];
        convert16.arr[1] = bytes[0];
	if(scaling_factor != 0) {
	    sdp8x->current_diff_pressure = (float)convert16.int16 / scaling_factor;
	}
        
#ifdef DEBUG_OUTPUT
        NRF_LOG_INFO("Differential Pressure: " NRF_LOG_FLOAT_MARKER " Pa", sdp8x->current_diff_pressure);
        NRF_LOG_FLUSH();
#endif

        // get temperature
        sdp8x_get_bytes(sdp8x->sdp8x_data, 2, 3, bytes);
        convert16.arr[0] = bytes[1];
        convert16.arr[1] = bytes[0];
        sdp8x->current_temperature = (float)convert16.int16 / 200.0;
#ifdef DEBUG_OUTPUT
        NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER " C", sdp8x->current_temperature);
        NRF_LOG_FLUSH();
#endif
    }
#ifdef DEBUG_OUTPUT
        NRF_LOG_INFO("");
        NRF_LOG_FLUSH();
#endif

}

/*
    @brief Function for getting the temperature

    @note returns what's in current_temperature, which contains the most recent temperature measurement

    @return float temperature measurement

    @param[in] sdp8x Pointer of library handler
*/
float sdp8x_get_temperature(sdp8x_handler_t * sdp8x) {
    return sdp8x->current_temperature;
}

/*
    @brief Function for getting the differential pressure

    @note returns what's in current_diff_pressure, which contains the most recent differential pressure measurement

    @return float differential pressure measurement

    @param[in] sdp8x Pointer of library handler
*/
float sdp8x_get_differential_pressure(sdp8x_handler_t * sdp8x) {
    return sdp8x->current_diff_pressure;
}


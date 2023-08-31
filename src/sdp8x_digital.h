/* ************************************************************************** */
/** Sensirion SDP8x Digital Differential Pressure Sensor Function Library

  @File Name
    sdp8x_digital.h

  @Summary
    Driver Library for Sensirion SDP8x Differential Pressure Sensor

  @Description
    Defines functions that allow the user to interact with the sensor
 **************************************************************************** */

#ifndef SDP8X_DIGITAL_H
#define SDP8X_DIGITAL_H

#include <stdint.h>

//#define DEBUG_OUTPUT // comment this line to turn off printf statements
#ifdef DEBUG_OUTPUT
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#endif

#define SDP8X_I2C_ADDR 0x25

// continous measurement modes
typedef enum {
    /*
        Temperature compensation - Mass Flow
        Averaging - Average till read
	Command Code - 0x3603
    */
    SDP8X_CONT_MODE0 = 0,
    /*
	Temperature compensation - Mass Flow
	Averaging - None, Update rate 0.5ms
	Command Code - 0x3608
    */
    SDP8X_CONT_MODE1,
    /*
	Temperature compensation - Differential Pressure
	Averaging - Average till read
	Command Code - 0x3615
    */
    SDP8X_CONT_MODE2,
    /*
	Temperature compensation - Differential Pressure
	Averaging - None, Update rate 0.5ms
	Command Code - 0x361E
    */
    SDP8X_CONT_MODE3,
    SDP8X_CONT_MODE_DEFAULT = SDP8X_CONT_MODE0
} sdp8x_cont_mode_t;

// triggered measurement modes
typedef enum {
    /*
	Temperature compensation - Mass Flow
	Clock stretching? - No
	Command Code - 0x3624
    */
    SDP8X_TRIGGERED_MODE0 = 0,
    /*
        Temperature compensation - Mass Flow
        Clock stretching? - Yes
	Command Code - 0x3726
    */
    SDP8X_TRIGGERED_MODE1,
    /*
        Temperature compensation - Differential Pressure
        Clock stretching? - No
	Command Code - 0x362F
    */
    SDP8X_TRIGGERED_MODE2,
    /*
        Temperature compensation - Differential Pressure
        Clock stretching? - Yes
	Command Code - 0x372D
    */
    SDP8X_TRIGGERED_MODE3,
} sdp8x_triggered_mode_t;

/*
    Union used to convert a byte array into a uint16_t
*/
union ByteArrayTo16BitInteger {
    uint8_t arr[2];
    int16_t int16;
};

/*
    Union used to convert a byte array into a uint32_t
*/
union ByteArrayTo32BitInteger {
    uint8_t arr[4];
    uint32_t int32;
};

/*
    Union used to convert a byte array into a uint64_t
*/
union ByteArrayTo64BitInteger {
    uint8_t arr[8];
    uint64_t int64;
};

typedef struct {
    uint8_t sdp8x_data[9];
    uint8_t sdp8x_product_info[18];
    uint32_t product_number;
    uint64_t serial_number;
    float current_diff_pressure;
    float current_temperature;
    void(*sdp8x_transmit)(uint8_t *tx, uint8_t size, uint8_t addr);
    void(*sdp8x_receive)(uint8_t *rx, uint8_t size, uint8_t addr);
    void(*delay)(uint32_t ms);
} sdp8x_handler_t;

/*******************************[ High-Level Sensor Functions For General Use ]****************************************/

/*
    @brief Function for initializing the SDP8x Differential Pressure Sensor

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_init(sdp8x_handler_t * sdp8x);

/*
    @brief Function for starting continuous measurement mode with the given mode
    
    @param[in] mode Desired mode for continuous measurement

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_start_continuous_measurement(sdp8x_cont_mode_t mode, sdp8x_handler_t * sdp8x);

/*
    @brief Function for stopping continuous measurement

    @note This command stops the continuous measurement and puts the sensor in idle mode.
	  It powers off the heater and makes the sensor receptive for another command after 500us. 
	  The Stop command is also required when switching between different continuous measurement commands

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_stop_continuous_measurement(sdp8x_handler_t * sdp8x);

/*
    @brief Function for triggered measurement with the given mode

    @param[in] mode Desired mode for triggered measurement

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_triggered_measurement(sdp8x_triggered_mode_t mode, sdp8x_handler_t * sdp8x);

/*
    @brief Function for entering sleep mode

    @note Sleep mode can only be entered from idle mode, i.e. when the sensor is not measuring.
	  If in triggered mode, the sleep command can be sent after the result has been read and the sensor is idle.
	  If in continuous mode, the sleep command can be sent after the stop continous measurement 
	  command has been issued and the sensor is idle. In sleep mode the sensor cannot be reset.

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_enter_sleep_mode(sdp8x_handler_t * sdp8x);

/*
    @brief Function for exiting sleep mode
    
    @note The sensor exits the sleep mode and enters the idle mode when it receives the valid I2C address and a write bit (‘0’).
	  note that the I2C address is not acknowledged. The sensor should wake up within 20 ms. Polling with a write header
	  (I2C address and a write bit) can be used to check whether the sensor has woken up.

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_exit_sleep_mode(sdp8x_handler_t * sdp8x);

/*
    @brief Function for resetting the sensor.
    
    @note After the reset command the sensor will take maximum 20 ms to reset. 
	  During this time the sensor will not acknowledge its address nor accept commands.

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_soft_reset(sdp8x_handler_t * sdp8x);

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
void sdp8x_read_product_info(sdp8x_handler_t * sdp8x);

/*
    @brief Function for getting the serial number

    @return 64-bit unique Serial Number

    @param[in] sdp8x Pointer of library handler
*/
uint64_t sdp8x_get_serial_num(sdp8x_handler_t * sdp8x);

/*
    @brief Function for getting the product number

    @note the last 8 bits are the revision number and subject to change

    @return 32-bit product number

    @param[in] sdp8x Pointer of library handler
*/
uint32_t sdp8x_get_product_num(sdp8x_handler_t * sdp8x);

/*
    @brief Function for reading measurement data

    @note Call this in your super loop

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_read_data(sdp8x_handler_t * sdp8x);

/*
    @brief Function for getting x bytes of data out of the given sensor response buffer

    @param[in] buf Either sdp8x_data or sdp8x_product_info

    @param[in] num_bytes How many bytes to read from the buffer

    @param[in] start_idx Index to start at in the buffer

    @param[in] byte array
*/
void sdp8x_get_bytes(uint8_t * buf, uint8_t num_bytes, uint8_t start_idx, uint8_t * bytes);

/*
    @brief Function for processing data measurements

    @note prints data if debug output is on

    @param[in] sdp8x Pointer of library handler
*/
void sdp8x_data_handler(sdp8x_handler_t * sdp8x);

/*
    @brief Function for getting the differential pressure

    @note returns what's in current_diff_pressure, which contains the most recent differential pressure measurement

    @return float differential pressure measurement

    @param[in] sdp8x Pointer of library handler
*/
float sdp8x_get_differential_pressure(sdp8x_handler_t * sdp8x);

/*
    @brief Function for getting the temperature

    @note returns what's in current_temperature, which contains the most recent temperature measurement

    @return float temperature measurement

    @param[in] sdp8x Pointer of library handler
*/
float sdp8x_get_temperature(sdp8x_handler_t * sdp8x);

#endif // SDP8x_DIGITAL_H


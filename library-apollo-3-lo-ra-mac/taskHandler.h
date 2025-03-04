/***************************************************************************//**
 *  \file       taskHandler.h
 *
 *  \brief      Provides multiple functions for tasks used in the main.
 *
 *  \date       06.03.2024
 *
 *  \author     Uttunga Shinde ( Uni Freiburg IMTEK )
 ******************************************************************************/

#ifndef TASKS_TASKHANDLER_H_
#define TASKS_TASKHANDLER_H_

/******************************************************************************
 * INCLUDES
 *****************************************************************************/
#include "apollo3.h"
#include "am_mcu_apollo.h"
#include "device.h"
#include "mb85rc64ta.h"
#include "i2c.h"
#include "spi.h"
#include "dac63002.h"
#include "am1805.h"
#include "shtc3.h"
#include "as7341.h"
#include "timing.h"
#include "am_util_delay.h"
#include "adc.h"
#include "am_util_stdio.h"
//#include "sht4x_i2c.h"


#define NUMBER_SEND 40
#define CHANNEL_PAR 0
#define CHANNEL_SHUNT_V 1
#define CHANNEL_BUS_V 2
#define CHANNEL_CURRENT_REG 3
#define CHANNEL_SUPERCAP_V 4





 typedef union SendData_u
{
    uint8_t SendBytes[NUMBER_SEND];
    struct
    {
        //uint16_t sendType;

        // Spectral Sensor
		union
		{
			float PAR;
			uint8_t PARbytes[4];
		};
//        uint16_t spec_ch0;
//        uint16_t spec_ch1;
//        uint16_t spec_ch2;
//        uint16_t spec_ch3;
//        uint16_t spec_ch4;
//        uint16_t spec_ch5;
//        uint16_t spec_ch6;
//        uint16_t spec_ch7;
//        uint16_t spec_ch8;
//        uint16_t spec_ch9;
//        uint16_t spec_ch10;
//        uint16_t spec_ch11;
//        uint16_t spec_gain;

//Changed for forestTrials	
		 // SHT4x
//       union
//        {
//            int32_t temperature;
//            uint8_t temp_bytes[4];
//        };

//        union
//        {
//            int32_t humidity;
//            uint8_t hum_bytes[4];
//        };

//Changed for forestTrials				
        // SHTC3
        union
        {
            float temperature;
            uint8_t temp_bytes[4];
        };

       union
       {
           float humidity;
            uint8_t hum_bytes[4];
        };

        // Datetime
        union
        {
            //Calendar datetime;	//had to adopt this to different RTC
			time_struct datetime;
            uint8_t datetime_bytes[9];	//in memory this is 12 because of byte padding
        };
		
		union
		{
			float shuntVoltage;
			uint8_t shuntVoltage_bytes[4];
		};
		
		union
		{
			float busVoltage;
			uint8_t busVoltage_bytes[4];
		};
		
		union 
		{
			float currentRegister;
			uint8_t currentRegister_bytes[4];
		};
		union 
		{
			float superCapVoltage;
			uint8_t superCapVoltage_bytes[4];
		};

    }Measurement;

}SendData_t;
 
typedef union MeasurementData_u
{
    uint8_t SendBytes[26];
    struct
    {
        //uint16_t sendType;

        // Spectral Sensor

        uint16_t spec_ch0;
        uint16_t spec_ch1;
        uint16_t spec_ch2;
        uint16_t spec_ch3;
        uint16_t spec_ch4;
        uint16_t spec_ch5;
        uint16_t spec_ch6;
        uint16_t spec_ch7;
        uint16_t spec_ch8;
        uint16_t spec_ch9;
        uint16_t spec_ch10;
        uint16_t spec_ch11;
        uint16_t spec_gain;

    }Measurement;

}MeasurementData_t;


/*
 *	@brief Initializes the I/O for measurements using the AS7341
 *
 *	@return 0 on success
 */
uint32_t initSpecMeasurement(void);


/*
 *	@brief Function that can be called to execute a measurement using the AS7341
 *
 *	@return 0 on success
 */
uint32_t executeSpecMeasurement(MeasurementData_t *data);

//changed for ForestTrails
/*
 *	@brief Function that can be called to execute a measurement using the SHT4x
 * 
 *	@return 0 on success
*/
//uint32_t executeTempMeasurement(SendData_t *data);

//changed for ForestTrails
/*
 *	@brief Function that can be called to execute a measurement using the SHTC3
 * 
 *	@return 0 on success
*/
uint32_t executeTempMeasurement(SendData_t *data);

	
//*****************************************************************************
//
//! @brief Executes the PAR calculation
//!
//! @return PAR value
//*****************************************************************************
float PARCalc(MeasurementData_t *data);


//*****************************************************************************
//
//! @brief Executes the sensor measurements
//!
//*****************************************************************************
void measurementProcess(SendData_t *data);
 
 #endif /* TASKS_TASKHANDLER_H_ */
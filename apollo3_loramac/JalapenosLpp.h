/*!
 * \file      JalapenosLpp.h
 *
 * \brief     Extends the Cayenne Low Power Protocol to support float 32
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 */
#ifndef __JALAPENOS_LPP_H__
#define __JALAPENOS_LPP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LPP_DIGITAL_INPUT       0       // 1 byte
#define LPP_DIGITAL_OUTPUT      1       // 1 byte
#define LPP_ANALOG_INPUT        2       // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT       3       // 2 bytes, 0.01 signed
#define LPP_LUMINOSITY          101     // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE            102     // 1 byte, 1
#define LPP_TEMPERATURE         103     // 4 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY   104     // 4 byte, 0.5% unsigned
#define LPP_PAR					105		// 4 bytes, signed value
#define LPP_TIMESTAMP			106		// 4 bytes, unsigned date
#define LPP_SHUNTVOLTAGE		107		// 4 bytes, signed value
#define LPP_BUSVOLTAGE			108		// 4 bytes, signed value
#define LPP_CURRENT				109		// 4 bytes, signed value
#define LPP_CAPVOLTAGE			110		// 4 bytes, signed value
#define LPP_ACCELEROMETER       113     // 2 bytes per axis, 0.001G
#define LPP_BAROMETRIC_PRESSURE 115     // 2 bytes 0.1 hPa Unsigned
#define LPP_GYROMETER           134     // 2 bytes per axis, 0.01 °/s
#define LPP_GPS                 136     // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01m


// Data ID + Data Type + Data Size
#define LPP_DIGITAL_INPUT_SIZE       	3
#define LPP_DIGITAL_OUTPUT_SIZE      	3
#define LPP_ANALOG_INPUT_SIZE        	6
#define LPP_ANALOG_OUTPUT_SIZE       	6
#define LPP_LUMINOSITY_SIZE          	4
#define LPP_PRESENCE_SIZE            	3
#define LPP_TEMPERATURE_SIZE         	14
#define LPP_RELATIVE_HUMIDITY_SIZE   	14
#define LPP_ACCELEROMETER_SIZE       	8
#define LPP_BAROMETRIC_PRESSURE_SIZE 	4
#define LPP_GYROMETER_SIZE           	8
#define LPP_GPS_SIZE                 	11
#define LPP_PAR_SIZE					6
#define LPP_TIMESTAMP_SIZE				6
#define LPP_SHUNTVOLTAGE_SIZE			6
#define LPP_BUSVOLTAGE_SIZE				6
#define LPP_CURRENT_SIZE				6
#define LPP_CAPVOLTAGE_SIZE				6


union analogVal
{
	float analog;
	uint8_t bytes[sizeof(float)];
};

void JalapenosLppInit( void );

void JalapenosLppReset( void );
uint8_t JalapenosLppGetSize( void );
uint8_t* JalapenosLppGetBuffer( void );
uint8_t JalapenosLppCopy( uint8_t* buffer );

uint8_t JalapenosLppAddDigitalInput( uint8_t channel, uint8_t value );
uint8_t JalapenosLppAddDigitalOutput( uint8_t channel, uint8_t value );

uint8_t JalapenosLppAddAnalogInput( uint8_t channel, float value );
uint8_t JalapenosLppAddAnalogOutput( uint8_t channel, float value );

uint8_t JalapenosLppAddLuminosity( uint8_t channel, uint16_t lux );
uint8_t JalapenosLppAddPresence( uint8_t channel, uint8_t value );
uint8_t JalapenosLppAddTemperature( uint8_t channel, float celsius1,float celsius2, float celsius3 );
uint8_t JalapenosLppAddRelativeHumidity( uint8_t channel, float rh1, float rh2, float rh3 );
uint8_t JalapenosLppAddAccelerometer( uint8_t channel, float x, float y, float z );
uint8_t JalapenosLppAddBarometricPressure( uint8_t channel, float hpa );
uint8_t JalapenosLppAddGyrometer( uint8_t channel, float x, float y, float z );
uint8_t JalapenosLppAddGps( uint8_t channel, float latitude, float longitude, float meters );

uint8_t JalapenosLppAddPAR ( uint8_t channel, float PAR);
uint8_t JalapenosLppAddTimestamp ( uint8_t channel, uint32_t time);
uint8_t JalapenosLppAddShuntVoltage ( uint8_t channel, float shuntVoltage);
uint8_t JalapenosLppAddBusVoltage ( uint8_t channel, float busVoltage);
uint8_t JalapenosLppAddCurrent ( uint8_t channel, float current);
uint8_t JalapenosLppAddCapVoltage ( uint8_t channel, float CapVoltage);

#ifdef __cplusplus
}
#endif

#endif // __JALAPENOS_LPP_H__

/*
 *  @file Adafruit_SHT4x.h
 *
 *  This is a library for the SHT4x Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT4x Humidity & Temp Sensor
 *  -----> https://www.adafruit.com/product/4885
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef ADAFRUIT_SHT4X_H
#define ADAFRUIT_SHT4X_H

#include <stdint.h>
#include <stdbool.h>
#include <i2c.h>
//#include <sensor.h>

#define SHT4x_DEFAULT_ADDR 0x44 /**< SHT4x I2C Address */
#define SHT4x_NOHEAT_HIGHPRECISION 0xFD /**< High precision measurement, no heater */
#define SHT4x_NOHEAT_MEDPRECISION 0xF6 /**< Medium precision measurement, no heater */
#define SHT4x_NOHEAT_LOWPRECISION 0xE0 /**< Low precision measurement, no heater */

#define SHT4x_HIGHHEAT_1S 0x39 /**< High precision measurement, high heat for 1 sec */
#define SHT4x_HIGHHEAT_100MS 0x32 /**< High precision measurement, high heat for 0.1 sec */
#define SHT4x_MEDHEAT_1S 0x2F /**< High precision measurement, med heat for 1 sec */
#define SHT4x_MEDHEAT_100MS 0x24 /**< High precision measurement, med heat for 0.1 sec */
#define SHT4x_LOWHEAT_1S 0x1E /**< High precision measurement, low heat for 1 sec */
#define SHT4x_LOWHEAT_100MS 0x15 /**< High precision measurement, low heat for 0.1 sec */

#define SHT4x_READSERIAL 0x89 /**< Read Out of Serial Register */
#define SHT4x_SOFTRESET 0x94  /**< Soft Reset */

/** How precise (repeatable) the measurement will be */
typedef enum {
  SHT4X_HIGH_PRECISION,
  SHT4X_MED_PRECISION,
  SHT4X_LOW_PRECISION,
} sht4x_precision_t;

/** Optional pre-heater configuration setting */
typedef enum {
  SHT4X_NO_HEATER,
  SHT4X_HIGH_HEATER_1S,
  SHT4X_HIGH_HEATER_100MS,
  SHT4X_MED_HEATER_1S,
  SHT4X_MED_HEATER_100MS,
  SHT4X_LOW_HEATER_1S,
  SHT4X_LOW_HEATER_100MS,
} sht4x_heater_t;

typedef struct {
  float temperature;
  float humidity;
  uint16_t sensorid_humidity;
  uint16_t sensorid_temp;
  i2c_dev_t *i2c_dev;
  sht4x_precision_t precision;
  sht4x_heater_t heater;
} Adafruit_SHT4x;

bool Adafruit_SHT4x_begin(Adafruit_SHT4x *sht4x, i2c_dev_t *i2c_dev);
uint32_t Adafruit_SHT4x_readSerial(Adafruit_SHT4x *sht4x);
bool Adafruit_SHT4x_reset(Adafruit_SHT4x *sht4x);

void Adafruit_SHT4x_setPrecision(Adafruit_SHT4x *sht4x, sht4x_precision_t prec);
sht4x_precision_t Adafruit_SHT4x_getPrecision(Adafruit_SHT4x *sht4x);
void Adafruit_SHT4x_setHeater(Adafruit_SHT4x *sht4x, sht4x_heater_t heat);
sht4x_heater_t Adafruit_SHT4x_getHeater(Adafruit_SHT4x *sht4x);

bool Adafruit_SHT4x_getEvent(Adafruit_SHT4x *sht4x, sensors_event_t *humidity, sensors_event_t *temp);

bool Adafruit_SHT4x_writeCommand(Adafruit_SHT4x *sht4x, uint16_t cmd);
bool Adafruit_SHT4x_readCommand(Adafruit_SHT4x *sht4x, uint16_t command, uint8_t *buffer, uint8_t num_bytes);

void Adafruit_SHT4x_fillTempEvent(Adafruit_SHT4x *sht4x, sensors_event_t *temp, uint32_t timestamp);
void Adafruit_SHT4x_fillHumidityEvent(Adafruit_SHT4x *sht4x, sensors_event_t *humidity, uint32_t timestamp);

#endif
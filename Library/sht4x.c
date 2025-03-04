/*!
 *  @file Adafruit_SHT4x.c
 *
 *  @mainpage Adafruit SHT4x Digital Humidity & Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the SHT4x Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT4x Digital sensor from Adafruit
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/4885
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "Adafruit_SHT4x.h"

static uint8_t crc8(const uint8_t *data, int len);

/*!
 * @brief  SHT4x constructor
 */
void Adafruit_SHT4x_init(Adafruit_SHT4x *sensor) {
}

/*!
 * @brief  SHT4x destructor
 */
void Adafruit_SHT4x_deinit(Adafruit_SHT4x *sensor) {
  if (sensor->temp_sensor) {
    free(sensor->temp_sensor);
  }
  if (sensor->humidity_sensor) {
    free(sensor->humidity_sensor);
  }
}

/**
 * Initialises the I2C bus, and assigns the I2C address to us.
 *
 * @param theWire   The I2C bus to use, defaults to &Wire
 *
 * @return True if initialisation was successful, otherwise False.
 */
bool Adafruit_SHT4x_begin(Adafruit_SHT4x *sensor, TwoWire *theWire) {
  if (sensor->i2c_dev) {
    free(sensor->i2c_dev); // remove old interface
  }

  sensor->i2c_dev = (Adafruit_I2CDevice *)malloc(sizeof(Adafruit_I2CDevice));
  Adafruit_I2CDevice_init(sensor->i2c_dev, SHT4x_DEFAULT_ADDR, theWire);

  if (!Adafruit_I2CDevice_begin(sensor->i2c_dev)) {
    return false;
  }

  if (!Adafruit_SHT4x_reset(sensor)) {
    return false;
  }

  sensor->humidity_sensor = (Adafruit_SHT4x_Humidity *)malloc(sizeof(Adafruit_SHT4x_Humidity));
  sensor->temp_sensor = (Adafruit_SHT4x_Temp *)malloc(sizeof(Adafruit_SHT4x_Temp));
  return true;
}

/**
 * Gets the ID register contents.
 *
 * @return The 32-bit ID register.
 */
uint32_t Adafruit_SHT4x_readSerial(Adafruit_SHT4x *sensor) {
  uint8_t cmd = SHT4x_READSERIAL;
  uint8_t reply[6];

  if (!Adafruit_I2CDevice_write(sensor->i2c_dev, &cmd, 1)) {
    return false;
  }
  delay(10);
  if (!Adafruit_I2CDevice_read(sensor->i2c_dev, reply, 6)) {
    return false;
  }

  if ((crc8(reply, 2) != reply[2]) || (crc8(reply + 3, 2) != reply[5])) {
    return false;
  }

  uint32_t serial = 0;
  serial = reply[0];
  serial <<= 8;
  serial |= reply[1];
  serial <<= 8;
  serial |= reply[3];
  serial <<= 8;
  serial |= reply[4];

  return serial;
}

/**
 * Performs a soft reset of the sensor to put it into a known state.
 * @returns True on success, false if could not communicate with chip
 */
bool Adafruit_SHT4x_reset(Adafruit_SHT4x *sensor) {
  uint8_t cmd = SHT4x_SOFTRESET;
  if (!Adafruit_I2CDevice_write(sensor->i2c_dev, &cmd, 1)) {
    return false;
  }
  delay(1);
  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the precision rating - more precise takes longer!
    @param  prec The desired precision setting, will be used during reads
*/
/**************************************************************************/
void Adafruit_SHT4x_setPrecision(Adafruit_SHT4x *sensor, sht4x_precision_t prec) {
  sensor->_precision = prec;
}

/**************************************************************************/
/*!
    @brief  Gets the precision rating - more precise takes longer!
    @returns  The current precision setting, will be used during reads
*/
/**************************************************************************/
sht4x_precision_t Adafruit_SHT4x_getPrecision(Adafruit_SHT4x *sensor) {
  return sensor->_precision;
}

/**************************************************************************/
/*!
    @brief  Sets the heating setting - more heating uses more power and takes
   longer
    @param  heat The desired heater setting, will be used during reads
*/
/**************************************************************************/
void Adafruit_SHT4x_setHeater(Adafruit_SHT4x *sensor, sht4x_heater_t heat) {
  sensor->_heater = heat;
}

/**************************************************************************/
/*!
    @brief  Gets the heating setting - more heating uses more power and takes
   longer
    @returns  The current heater setting, will be used during reads
*/
/**************************************************************************/
sht4x_heater_t Adafruit_SHT4x_getHeater(Adafruit_SHT4x *sensor) {
  return sensor->_heater;
}

/**************************************************************************/
/*!
    @brief  Gets the humidity sensor and temperature values as sensor events
    @param  humidity Sensor event object that will be populated with humidity
   data
    @param  temp Sensor event object that will be populated with temp data
    @returns true if the event data was read successfully
*/
/**************************************************************************/
bool Adafruit_SHT4x_getEvent(Adafruit_SHT4x *sensor, sensors_event_t *humidity,
                              sensors_event_t *temp) {
  uint32_t t = millis();

  uint8_t readbuffer[6];
  uint8_t cmd = SHT4x_NOHEAT_HIGHPRECISION;
  uint16_t duration = 10;

  if (sensor->_heater == SHT4X_NO_HEATER) {
    if (sensor->_precision == SHT4X_HIGH_PRECISION) {
      cmd = SHT4x_NOHEAT_HIGHPRECISION;
      duration = 10;
    }
    if (sensor->_precision == SHT4X_MED_PRECISION) {
      cmd = SHT4x_NOHEAT_MEDPRECISION;
      duration = 5;
    }
    if (sensor->_precision == SHT4X_LOW_PRECISION) {
      cmd = SHT4x_NOHEAT_LOWPRECISION;
      duration = 2;
    }
  }

  if (sensor->_heater == SHT4X_HIGH_HEATER_1S) {
    cmd = SHT4x_HIGHHEAT_1S;
    duration = 1100;
  }
  if (sensor->_heater == SHT4X_HIGH_HEATER_100MS) {
    cmd = SHT4x_HIGHHEAT_100MS;
    duration = 110;
  }

  if (sensor->_heater == SHT4X_MED_HEATER_1S) {
    cmd = SHT4x_MEDHEAT_1S;
    duration = 1100;
  }
  if (sensor->_heater == SHT4X_MED_HEATER_100MS) {
    cmd = SHT4x_MEDHEAT_100MS;
    duration = 110;
  }

  if (sensor->_heater == SHT4X_LOW_HEATER_1S) {
    cmd = SHT4x_LOWHEAT_1S;
    duration = 1100;
  }
  if (sensor->_heater == SHT4X_LOW_HEATER_100MS) {
    cmd = SHT4x_LOWHEAT_100MS;
    duration = 110;
  }

  if (!Adafruit_I2CDevice_write(sensor->i2c_dev, &cmd, 1)) {
    return false;
  }
  delay(duration);
  if (!Adafruit_I2CDevice_read(sensor->i2c_dev, readbuffer, 6)) {
    return false;
  }

  if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  float t_ticks = (uint16_t)readbuffer[0] * 256 + (uint16_t)readbuffer[1];
  float rh_ticks = (uint16_t)readbuffer[3] * 256 + (uint16_t)readbuffer[4];
  sensor->_temperature = -45 + 175 * t_ticks / 65535;
  sensor->_humidity = -6 + 125 * rh_ticks / 65535;

  sensor->_humidity = fmin(fmax(sensor->_humidity, (float)0.0), (float)100.0);

  // use helpers to fill in the events
  if (temp)
    Adafruit_SHT4x_fillTempEvent(sensor, temp, t);
  if (humidity)
    Adafruit_SHT4x_fillHumidityEvent(sensor, humidity, t);
  return true;
}

void Adafruit_SHT4x_fillTempEvent(Adafruit_SHT4x *sensor, sensors_event_t *temp, uint32_t timestamp) {
  memset(temp, 0, sizeof(sensors_event_t));
  temp->version = sizeof(sensors_event_t);
  temp->sensor_id = sensor->_sensorid_temp;
  temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  temp->timestamp = timestamp;
  temp->temperature = sensor->_temperature;
}

void Adafruit_SHT4x_fillHumidityEvent(Adafruit_SHT4x *sensor, sensors_event_t *humidity,
                                       uint32_t timestamp) {
  memset(humidity, 0, sizeof(sensors_event_t));
  humidity->version = sizeof(sensors_event_t);
  humidity->sensor_id = sensor->_sensorid_humidity;
  humidity->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  humidity->timestamp = timestamp;
  humidity->relative_humidity = sensor->_humidity;
}

/**
 * @brief Gets the Adafruit_Sensor object for the SHT4x's humidity sensor
 *
 * @return Adafruit_Sensor*
 */
Adafruit_Sensor *Adafruit_SHT4x_getHumiditySensor(Adafruit_SHT4x *sensor) {
  return sensor->humidity_sensor;
}

/**
 * @brief Gets the Adafruit_Sensor object for the SHT4x's temperature sensor
 *
 * @return Adafruit_Sensor*
 */
Adafruit_Sensor *Adafruit_SHT4x_getTemperatureSensor(Adafruit_SHT4x *sensor) {
  return sensor->temp_sensor;
}

/**
 * @brief  Gets the sensor_t object describing the SHT4x's humidity sensor
 *
 * @param sensor The sensor_t object to be populated
 */
void Adafruit_SHT4x_Humidity_getSensor(Adafruit_SHT4x_Humidity *humidity_sensor, sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "SHT4x_H", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = humidity_sensor->_sensorID;
  sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->min_delay = 0;
  sensor->min_value = 0;
  sensor->max_value = 100;
  sensor->resolution = 2;
}

/**
    @brief  Gets the humidity as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
 */
bool Adafruit_SHT4x_Humidity_getEvent(Adafruit_SHT4x_Humidity *humidity_sensor, sensors_event_t *event) {
  Adafruit_SHT4x_getEvent(humidity_sensor->_theSHT4x, event, NULL);

  return true;
}

/**
 * @brief  Gets the sensor_t object describing the SHT4x's temperature sensor
 *
 * @param sensor The sensor_t object to be populated
 */
void Adafruit_SHT4x_Temp_getSensor(Adafruit_SHT4x_Temp *temp_sensor, sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "SHT4x_T", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = temp_sensor->_sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40;
  sensor->max_value = 85;
  sensor->resolution = 0.3; // depends on calibration data?
}

/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns true
*/
bool Adafruit_SHT4x_Temp_getEvent(Adafruit_SHT4x_Temp *temp_sensor, sensors_event_t *event) {
  Adafruit_SHT4x_getEvent(temp_sensor->_theSHT4x, NULL, event);

  return true;
}

/**
 * Internal function to perform an I2C write.
 *
 * @param cmd   The 16-bit command ID to send.
 */
bool Adafruit_SHT4x_writeCommand(Adafruit_SHT4x *sensor, uint16_t command) {
  uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  return Adafruit_I2CDevice_write(sensor->i2c_dev, cmd, 2);
}

/**
 * Internal function to perform an I2C read.
 *
 * @param cmd   The 16-bit command ID to send.
 */
bool Adafruit_SHT4x_readCommand(Adafruit_SHT4x *sensor, uint16_t command, uint8_t *buffer,
                                 uint8_t num_bytes) {
  uint8_t cmd[2];

  cmd[0] = command >> 8;
  cmd[1] = command & 0xFF;

  return Adafruit_I2CDevice_write_then_read(sensor->i2c_dev, cmd, 2, buffer, num_bytes);
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}


/***************************************************************************//**
 *  \file       taskHandler.c
 *
 *  \brief      Provides multiple functions for tasks used in the main.
 *
 *  \date       06.03.2024
 *
 *  \author     Timm Luhmann ( Uni Freiburg IMTEK )
 *
 *  \author     Johannes Klueppel ( Uni Freiburg IMTEK )
 ******************************************************************************/
 
 #include "taskHandler.h"
 
 /******************************************************************************
 * VARIABLES
 *****************************************************************************/

static bool parInitialized = false;
static as7341_gain_t new_gain = AS7341_AGAIN_512;

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 *****************************************************************************/
uint32_t SpectrometerMeasurement(uint16_t *data);

/******************************************************************************
 * LOCAL FUNCTION IMPLEMENTATION
 *****************************************************************************/
 
 /***************************************************************************//**
 *
 * \brief Initializes I2C
 *
 ******************************************************************************/
uint32_t initSpecMeasurement(void){
	uint32_t error = 0;
	error += device_I2C_init();		//this is known to cause an error if called a second time without de-init
	parInitialized = true;
	return error;
}

/***************************************************************************//**
 * \brief Executes the whole measurement of the spectral sensor. 
 * 				Starts the autogain function.
 *				Moves the data to the measurements struct.
 *
 * \param [in] MeasurementData_t struct
 *
 * \ret		typedef enum as7341_status: SUCCESS, ERROR, ACKNO_ERROR
 ******************************************************************************/
uint32_t executeSpecMeasurement(MeasurementData_t *data){
	uint32_t error = 0;
	uint16_t spectralData[12];
	
	// Initialize I2C if not already done
	if(parInitialized==false){
		error += initSpecMeasurement();
	}
	
	// Test connection to sensor
	error += as7341_testCommunication();
	if (error)
	{
		// If no sensors answers, transfer error values in the measurement struct
		data->Measurement.spec_ch0 = 0xFFFF;
		data->Measurement.spec_ch1 = 0xFFFF;
		data->Measurement.spec_ch2 = 0xFFFF;
		data->Measurement.spec_ch3 = 0xFFFF;
		data->Measurement.spec_ch4 = 0xFFFF;
		data->Measurement.spec_ch5 = 0xFFFF;
		data->Measurement.spec_ch6 = 0xFFFF;
		data->Measurement.spec_ch7 = 0xFFFF;
		data->Measurement.spec_clear = 0xFFFF;
		data->Measurement.spec_nir = 0xFFFF;
		data->Measurement.spec_gain = 0xFFFF;
		data->Measurement.spec_tint = 0xFFFF;
		
		return error;
	}
	else
	{
		// Configure measurement parameter
		error += as7341_config(29, 5000, new_gain);
		uint16_t gain = 0;
		
		// Check if clear value is inside the borders
		// If yes, continue with transfering the measurements
		// If no, set new gain, repeat measurement
		for (uint8_t i = 0; i < MAX_AUTOGAIN_ITERATIONS; i++)
    {
			error += SpectrometerMeasurement(spectralData);
			error += as7341_getGain(&gain);
		  uint16_t target_value = spectralData[8];

      if (target_value < TARGET_MAX_VALUE && target_value > TARGET_MIN_VALUE)
      {
        break;
      }
      else if(target_value < TARGET_MIN_VALUE && gain == AS7341_AGAIN_512)
      {
        break;
      }
      else if(target_value > TARGET_MAX_VALUE && gain == AS7341_AGAIN_05)
      {
        break;
      }
      else
      {
				autogainSpec(gain, &new_gain, target_value);
        as7341_setGain(new_gain);
      }
    }
			// Transfer data to Measurement struct
			data->Measurement.spec_ch0 = spectralData[0];
			data->Measurement.spec_ch1 = spectralData[1];
			data->Measurement.spec_ch2 = spectralData[2];
			data->Measurement.spec_ch3 = spectralData[3];
			data->Measurement.spec_ch4 = spectralData[4];
			data->Measurement.spec_ch5 = spectralData[5];
			data->Measurement.spec_ch6 = spectralData[6];
			data->Measurement.spec_ch7 = spectralData[7];
			data->Measurement.spec_clear = spectralData[8];
			data->Measurement.spec_nir = spectralData[10];
		
		
			error += as7341_getGain(&gain);
			error += as7341_convertGain(&gain);
			data->Measurement.spec_gain = (uint16_t) gain;
			double tint = 0;
			error += as7341_getTINT(&tint);
			data->Measurement.spec_tint = (uint16_t) tint;
			
			return error;
	}
 }

 /***************************************************************************//**
 * \brief Functions that triggers the measurement of the spectral sensor.
 *
 * \param [in] uint16_t *data of at least size 12
 *
 * \ret		typedef enum as7341_status: SUCCESS, ERROR, ACKNO_ERROR
 ******************************************************************************/
uint32_t SpectrometerMeasurement(uint16_t *data)
{
	uint32_t error = 0;	// typedef enum as7341_status: SUCCESS, ERROR, ACKNO_ERROR
	
	// Configure Channel F1 to F6 to the ADC 
  error += as7341_writeSMUXmapping(AS7341_SMUX_F1_F6);
	// Wait for the task to be done
	error += as7341_delayForSMUX();
	// Start measuring configured channels
	error += as7341_startMeasurement(); 
	// Wait for the task to be done
	error += as7341_delayForData();
	// Read measurements from sensor
	error += as7341_transmitMeasurements(data);
	// Stop measurement
	error += as7341_stopMeasuring();

	// Configure Channel F7 and F8 as well as Clear, Dark, NIR, Flicker to the ADC 
	error += as7341_writeSMUXmapping(AS7341_SMUX_F7F8CDNF); 
	// Wait for the task to be done
	error += as7341_delayForSMUX();
	// Start measuring configured channels
	error += as7341_startMeasurement(); 
	// Wait for the task to be done
	error += as7341_delayForData();
	// Read measurements from sensor
	error += as7341_transmitMeasurements(data);
	// Stop measurement
	error += as7341_stopMeasuring();
		
	return error;
}

void autogainSpec(as7341_gain_t last_gain, as7341_gain_t *new_gain, uint16_t last_clear) {

    *new_gain = last_gain;

    // Clear > target value and gain > 1: reduce gain
    if ((last_clear > TARGET_MAX_VALUE) && (last_gain > AS7341_AGAIN_05)) {
        *new_gain = (as7341_gain_t)(last_gain - 1);
    }

    // Clear < target value and gain < 512: increase gain
    if ((last_clear < TARGET_MIN_VALUE) && (last_gain < AS7341_AGAIN_512)) {
        *new_gain = (as7341_gain_t)(last_gain + 1);
    }
}




	
//*****************************************************************************
//
//! @brief Executes the PAR calculation
//!
//! @return PAR value
//*****************************************************************************
float PARCalc(MeasurementData_t *data){
	MeasurementBasic_t basic_values;
	
	uint16_t conv_gain = 0;
	as7341_convertGain(&data->Measurement.spec_gain);
	
	// Divide raw value with ADC gain and integration time
	float div = (conv_gain * data->Measurement.spec_tint);
	basic_values.basic_ch0 = data->Measurement.spec_ch0 / div;
	basic_values.basic_ch1 = data->Measurement.spec_ch1 / div;
	basic_values.basic_ch2 = data->Measurement.spec_ch2 / div;
	basic_values.basic_ch3 = data->Measurement.spec_ch3 / div;
	basic_values.basic_ch4 = data->Measurement.spec_ch4 / div;
	basic_values.basic_ch5 = data->Measurement.spec_ch5 / div;
	basic_values.basic_ch6 = data->Measurement.spec_ch6 / div;
	basic_values.basic_ch7 = data->Measurement.spec_ch7 / div;
	basic_values.basic_clear = data->Measurement.spec_clear / div;
	basic_values.basic_nir = data->Measurement.spec_nir / div;
	
	float PAR = 0;
	float calibration_coefficients[8] = {28.82343311, 10.92971318, 12.36721112, 10.32584625, 7.72788242, 6.90065222, 8.71694163, 7.75490868*0.75}; //FlexPCB 1, Calib. 17.01.2025, not correct, too high
	PAR += basic_values.basic_ch0 * calibration_coefficients[0];
	PAR += basic_values.basic_ch1 * calibration_coefficients[1];
	PAR += basic_values.basic_ch2 * calibration_coefficients[2];
	PAR += basic_values.basic_ch3 * calibration_coefficients[3];
	PAR += basic_values.basic_ch4 * calibration_coefficients[4];
	PAR += basic_values.basic_ch5 * calibration_coefficients[5];
	PAR += basic_values.basic_ch6 * calibration_coefficients[6];
	PAR += basic_values.basic_ch7 * calibration_coefficients[7] * 0.73; // care for responsiveness above 700nm
	return PAR;
}

uint32_t executeTempMeasurement(SendData_t *data){
	uint32_t error = 0;
	float temps[3], hums[3];
	timerDelay(1);	
	error += SHT4x_GetTempAndHumi(temps, hums);
	data->Measurement.humidity1 = hums[0];
	data->Measurement.temperature1 = temps[0];
	data->Measurement.humidity2 = hums[1];
	data->Measurement.temperature2 = temps[1];
	data->Measurement.humidity3 = hums[2];
	data->Measurement.temperature3 = temps[2];
	error += turnOffTemp();
	return error;
}


	
void measurementProcess(SendData_t *data){
	
	prepareSensing();
	SendData_t test_data;
	executeTempMeasurement(&test_data);
	MeasurementData_t measData;		//for the PAR measurement
	//ina232_readBusVoltage(&data->Measurement.busVoltage);
	//ina232_readShuntVoltage(&data->Measurement.shuntVoltage);
	//ina232_readCurrent(&data->Measurement.currentRegister);
	executeSpecMeasurement(&measData);
	//data->Measurement.PAR = PARCalc(&measData);
	//executeTempMeasurement(data);
	//data->Measurement.superCapVoltage = 1.5 * 3.21 * getAdcSample() / 16384;		//1.5V is the ADC reference, 3.21 originates from the resistors, 2^14 resolution
	
}	
	
	
	
	
	
	
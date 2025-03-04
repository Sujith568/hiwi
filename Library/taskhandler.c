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

static uint8_t as_gain = AS7341_AGAIN_4;
static bool parInitialized = false;
uint16_t spectralDataFIFO[18];

/******************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 *****************************************************************************/
static void SpectrometerMeasurement(uint8_t selectgain);



/******************************************************************************
 * LOCAL FUNCTION IMPLEMENTATION
 *****************************************************************************/
/***************************************************************************//**
 * \brief Functions that triggers the measurement of the spectral Sensor
 *
 * \param [in] selectgain If selectgain=-1 --> gain is according to our own specification: Ch1 - 8: Gain 4, rest: Gain1
 *
 * \todo    return value
 ******************************************************************************/
 //#define DEBUG_AS
static void SpectrometerMeasurement(uint8_t selectgain)
{
    //First we init the array with something unreasonable to be sure that all values are overwritten:
//    if (selectgain == 0)
//        as7341_setGain(AS7341_AGAIN_4);
//    else
	
    as7341_setGain(selectgain);

    as7341_writeSMUXmapping(AS7341_SMUX_F1_F6);
	am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
	uint32_t reg = 0;
#ifdef DEBUG_AS
	as7341_readRegister(&reg);
	as7341_clearInterrupt(reg);
#endif
    as7341_startMeasurement(false, false); //first 6 channels flicker off, wait=false
	am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
    as7341_transmitMeasurementsFIFO(&spectralDataFIFO[0]);
#ifdef DEBUG_AS
	as7341_readRegister(&reg);
	as7341_clearInterrupt(reg);
#endif
    as7341_stopMeasuring();
	
//    if (selectgain == 0)
//        as7341_setGain(AS7341_AGAIN_4);
//    else
    as7341_setGain(selectgain);

    as7341_writeSMUXmapping(AS7341_SMUX_F7F8);

	am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
#ifdef DEBUG_AS
	as7341_readRegister(&reg);
	as7341_clearInterrupt(reg);
#endif
    as7341_startMeasurement(false, false); //first 6 channels flicker off, wait=false
	am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
    as7341_transmitMeasurementsFIFO(&spectralDataFIFO[6]);
#ifdef DEBUG_AS
	as7341_readRegister(&reg);
	as7341_clearInterrupt(reg);
#endif
    as7341_stopMeasuring();
	
//    if (selectgain == 0)
//        as7341_setGain(AS7341_AGAIN_1);
//    else
    as7341_setGain(selectgain);

    as7341_writeSMUXmapping(AS7341_SMUX_CDNF);
	am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
#ifdef DEBUG_AS
	as7341_readRegister(&reg);
	as7341_clearInterrupt(reg);
#endif
    as7341_startMeasurement(false, false); //first 6 channels flicker off, wait=false
	am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
    as7341_transmitMeasurementsFIFO(&spectralDataFIFO[12]);
#ifdef DEBUG_AS
	as7341_readRegister(&reg);
	as7341_clearInterrupt(reg);
#endif
    as7341_stopMeasuring();
	
}
 
uint32_t initSpecMeasurement(void){
	uint32_t error = 0;
	error += device_I2C_init();													//this is known to cause an error if called a second time without de-init
	error += am_hal_gpio_pinconfig(AS7341_INT_PIN,gpioPullIntPAR);			  //Initialize GPIO Intterupt used for PAR measurement
	AM_HAL_GPIO_MASKCREATE(GpioIntMask);
	error += am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AS7341_INT_PIN));
	error += am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, AS7341_INT_PIN));
	error += am_hal_gpio_interrupt_register(AS7341_INT_PIN,*(am_hal_gpio_handler_t)as7341IRQ);
	am_hal_interrupt_master_enable();
	NVIC_EnableIRQ(GPIO_IRQn);
	parInitialized = true;
	return error;
}
 
 
uint32_t executeSpecMeasurement(MeasurementData_t *data){
	uint32_t error = 0;
	error += turnOnPARLAI();
	timerDelay(1);
	if(parInitialized==false){
		error += initSpecMeasurement();
	}
	as7341_config();
	if (!as7341_testCommunication())
	{
		data->Measurement.spec_ch0 = 0xFFFF;
		data->Measurement.spec_ch1 = 0xFFFF;
		data->Measurement.spec_ch2 = 0xFFFF;
		data->Measurement.spec_ch3 = 0xFFFF;
		data->Measurement.spec_ch4 = 0xFFFF;
		data->Measurement.spec_ch5 = 0xFFFF;
		data->Measurement.spec_ch6 = 0xFFFF;
		data->Measurement.spec_ch7 = 0xFFFF;
		data->Measurement.spec_ch8 = 0xFFFF;
		data->Measurement.spec_ch9 = 0xFFFF;
		data->Measurement.spec_ch10 = 0xFFFF;
		data->Measurement.spec_ch11 = 0xFFFF;
		data->Measurement.spec_gain = 0xFFFF;
		return error+1;
	}
	else
	{
		SpectrometerMeasurement(as_gain);
		data->Measurement.spec_ch0 = spectralDataFIFO[0];
		data->Measurement.spec_ch1 = spectralDataFIFO[1];
		data->Measurement.spec_ch2 = spectralDataFIFO[2];
		data->Measurement.spec_ch3 = spectralDataFIFO[3];
		data->Measurement.spec_ch4 = spectralDataFIFO[4];
		data->Measurement.spec_ch5 = spectralDataFIFO[5];
		data->Measurement.spec_ch6 = spectralDataFIFO[6];
		data->Measurement.spec_ch7 = spectralDataFIFO[7];
		data->Measurement.spec_ch8 = spectralDataFIFO[14];
		data->Measurement.spec_ch9 = spectralDataFIFO[15];
		data->Measurement.spec_ch10 = spectralDataFIFO[16];
		data->Measurement.spec_ch11 = spectralDataFIFO[17];
		data->Measurement.spec_gain = (uint16_t) as7341_getSpectralGain();
		error += turnOffPARLAI();
		return error;
	}
	
	return error;
 }


uint32_t executeTempMeasurement(SendData_t *data){
	uint32_t error = 0;
	float temperature1;
	float humidity1;
	float temperature2;
	float humidity2;
	float temperature3;
	float humidity3;
	error += turnOnTemp();
	timerDelay(1);	
	error += SHTC3_GetTempAndHumi(&temperature1,&humidity1,&temperature2,&humidity2,&temperature3,&humidity3);
	data->Measurement.humidity1 = humidity1;
	data->Measurement.temperature1 = temperature1;
	data->Measurement.humidity2 = humidity2;
	data->Measurement.temperature2 = temperature2;
	data->Measurement.humidity3 = humidity3;
	data->Measurement.temperature3 = temperature3;
	error += turnOffTemp();
	return error;
}
	
	
//*****************************************************************************
//
//! @brief Executes the PAR calculation
//!
//! @return PAR value
//*****************************************************************************
float PARCalc(MeasurementData_t *data){
	float PAR = B0;
	PAR += data->Measurement.spec_ch0 * B1;	//assuming F1 corresponds to ch0
	PAR += data->Measurement.spec_ch1 * B2;
	PAR += data->Measurement.spec_ch2 * B3;
	PAR += data->Measurement.spec_ch3 * B4;
	PAR += data->Measurement.spec_ch4 * B5;
	PAR += data->Measurement.spec_ch5 * B6;
	PAR += data->Measurement.spec_ch6 * B7;
	PAR += data->Measurement.spec_ch7 * B8;
	return PAR;
}
	
	
void measurementProcess(SendData_t *data){
	
	prepareSensing();
	MeasurementData_t measData;		//for the PAR measurement
	ina232_readBusVoltage(&data->Measurement.busVoltage);
	ina232_readShuntVoltage(&data->Measurement.shuntVoltage);
	ina232_readCurrent(&data->Measurement.currentRegister);
	executeSpecMeasurement(&measData);
	data->Measurement.PAR = PARCalc(&measData);
	executeTempMeasurement(data);
	data->Measurement.superCapVoltage = 1.5 * 3.21 * getAdcSample() / 16384;		//1.5V is the ADC reference, 3.21 originates from the resistors, 2^14 resolution
	
}	
	
	
	
	
	
	
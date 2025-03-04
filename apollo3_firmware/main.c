/*!
 *  \file       main.c
 *
 *  \brief      Sample Firmware for Apollo 3 based applications
 *
 *  \date       06.03.2024
 *
 *  \author     Timm Luhmann (IMTEK)
 */
 
#include "RTE_Components.h" // Component selection
#include CMSIS_device_header // Device header
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
#include "arm_math.h"
#include "rtc.h"
#include "taskHandler.h"

//the current code will be configured to go to sleep for 1 minute, then execute the PAR & Temp/Humidity measurement & then go to sleep again

// The processor clock is initialized by CMSIS startup + system file
int main (void) { // User application starts here
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX,0);
	am_hal_pwrctrl_low_power_init();
	am_hal_rtc_osc_disable();
	
	am_devices_am1805_config_init();
	
	SendData_t data;
	initSpecMeasurement();
	while(1){
		executeSpecMeasurement(&data);
		executeTempMeasurement(&data);
		am_util_delay_ms(100);
	}
}

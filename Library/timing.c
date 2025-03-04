/*!
 *  \file       timing.c
 *
 *  \brief      Provides functions to configure the timer
 *
 *  \date       01.03.2023
 *
 *  \author     Timm Luhmann (IMTEK)
 */
//***** Header Files **********************************************************
//#include <stdint.h>
//#include <stdbool.h>
#include "math.h"
#include "am_mcu_apollo.h"
#include "pins.h"
#include "timing.h"
#include "device.h"

#include "am_util_stdio.h"

//***** Defines ***************************************************************

#define WAKE_INTERVAL_IN_MS 1
#define XT_PERIOD 32768
#define HFRC_PERIOD 3000000
//#define WAKE_INTERVAL XT_PERIOD * WAKE_INTERVAL_IN_MS * 1e-3

//#define WAKE_INTERVAL_IN_US 350
//#define WAKE_INTERVAL XT_PERIOD * WAKE_INTERVAL_IN_US * 1e-6

//***** Functions *************************************************************
int pin = 0;
int wait = 0;
extern uint8_t PAM_phase;
uint32_t interval;
uint16_t dacdata = 0xFFA0;	//if not other defined generate around 800mV
/*
* 
* @brief Function to initialize the system timer
*		 Currently using 3 MHz clock
*
*/
void myTimerInit(uint8_t state){
	interval = 16000;
	am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
	am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREF);
	am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
	NVIC_SetPriority(ADC_IRQn,1);	//set lower priority for adc
	NVIC_SetPriority(STIMER_CMPR0_IRQn,0);	//set the highest priority level
	NVIC_EnableIRQ(STIMER_CMPR0_IRQn);
	NVIC_EnableIRQ(STIMER_CMPR5_IRQn);
	am_hal_stimer_compare_delta_set(0,interval);
	if(state == 0){
		am_hal_stimer_compare_delta_set(5,600000);
	}
	else if(state == 1){
		am_hal_stimer_compare_delta_set(5,1800000);
	}
	else{
		am_hal_stimer_compare_delta_set(5,3000000);
	}
	////am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
	am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE | AM_HAL_STIMER_CFG_COMPARE_F_ENABLE);
	//am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE | AM_HAL_STIMER_CFG_COMPARE_F_ENABLE);
	PAM_phase = 1;	//PAM started
	pin = 1;
}

void timerDelay(uint32_t ms){
	wait = 0;
	uint32_t timerval = ms*3000;		//1ms equals 3000 timer ticks
	//uint32_t timerval = ms*32;		//1ms equals 32 timer ticks
	//am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA1);
	am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREE);
	am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);
	//am_hal_ctimer_config_single(1,AM_HAL_CTIMER_BOTH,AM_HAL_CTIMER_FN_ONCE | AM_HAL_CTIMER_HFRC_3MHZ);
	//NVIC_EnableIRQ(CTIMER_IRQn);
	NVIC_EnableIRQ(STIMER_CMPR4_IRQn);
	am_hal_stimer_compare_delta_set(4,timerval);
	//am_hal_ctimer_compare_set(1,AM_HAL_CTIMER_BOTH,0,timerval);
//	am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_E_ENABLE);
	am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ | AM_HAL_STIMER_CFG_COMPARE_E_ENABLE);
	//am_hal_ctimer_start(1,AM_HAL_CTIMER_BOTH);
	while(wait == 0){
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_NORMALSLEEP);
	}
	am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREE);
	NVIC_DisableIRQ(STIMER_CMPR4_IRQn);	
	//NVIC_DisableIRQ(CTIMER_IRQn);
}	

void myTimerDeInit(){
	am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR |AM_HAL_STIMER_CFG_FREEZE);
	am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREA);
	am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREF);
	NVIC_DisableIRQ(STIMER_CMPR0_IRQn);
	NVIC_DisableIRQ(STIMER_CMPR5_IRQn);
}

/*
*
* @brief system timer 0 interrupt routine: depending on the previous state the DAC is called to switch between high and low value
*
*/
void am_stimer_cmpr0_isr(){
	//
    // Check/clear the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, interval);
	if(pin){
		pin = 0;
		am_devices_dac63002_1_set_output(0x0000);
		//am_util_stdio_printf("Turn off");
	}
	else{
		pin = 1;
		am_devices_dac63002_1_set_output(dacdata);
		//am_util_stdio_printf("Turn on");
	}
	
}


void am_stimer_cmpr4_isr(){
	am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREE);
	wait = 1;
}

void am_stimer_cmpr5_isr(){
	//
    // Check/clear the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREF);
	NVIC_DisableIRQ(STIMER_CMPR5_IRQn);	// disable interrupt till it gets reconfigured
	NVIC_DisableIRQ(STIMER_CMPR0_IRQn);
	am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREA);
	am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREF);
	am_devices_dac63002_1_set_output(0x0000);
	dacdata = 0;
	PAM_phase = 2;
}

void am_ctimer_isr(){
	
	am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA1);
	wait = 1;
}
/*
* @param frequency: The frequency for the pwm generation using the external DAC in combination with the system timer
*
* @brief Function to set the timer interval to meet the desired frequency on the pwm output
*/
void setTimerFreq(uint32_t frequency){
	
	//interval = (uint32_t)round((32678/(frequency*2)));	//currently for 50% duty cycle
	interval = (uint32_t)round((3000000/(frequency*2)));	//currently for 50% duty cycle
	//am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);
	//am_hal_stimer_compare_delta_set(0,interval);
	//am_hal_stimer_config(AM_HAL_STIMER_CFG_THAW);
	interval -= 22;	//hand calibration for 1000 Hz currently
}

void setTimerVoltageOut(float voltage){
	dacdata = (uint16_t) round((voltage*4096)/(1.8));
	dacdata = dacdata<<4;
}
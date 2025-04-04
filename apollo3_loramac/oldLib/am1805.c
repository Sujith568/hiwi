/*!
 *  \file       am1805.c
 *
 *  \brief      Provides functions to use the external AM1805 RTC
 *
 *  \date       03.02.2023
 *
 *  \author     Timm Luhmann (IMTEK)
 */
//***** Header Files **********************************************************
#include <stdint.h>
//#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "pins.h"
#include "i2c.h"
#include "am1805.h"
#include "gpio.h"
#include "pins.h"
#include "am_util_stdio.h"


//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

typedef struct
{
    uint32_t                    ui32Module;
    void                        *pIomHandle;
    bool                        bOccupied;
} am_devices_iom_am1805_t;

am_devices_iom_am1805_t gAm1805[AM_DEVICES_AM1805_MAX_DEVICE_NUM];	//one device maximum

am_hal_iom_config_t     g_sIomAm1805Cfg =
{
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_100KHZ,
    .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
    .ui32NBTxnBufLength   = 0,
    .pNBTxnBuf = NULL,
};



//*****************************************************************************
//
//! @brief Reads the ID of the RTC and returns the value.
//!
//! @param pDeviceID - Pointer to the return buffer for the Device ID.
//!
//! This function reads the device ID register of the external rtc, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_am1805_read_id(uint32_t *pDeviceID)
{
    //am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)pHandle;
    am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)my_IomdevHdl;
    //
    // Send the command sequence to read the Device ID.
    //
    if (am_device_command_read(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           0x28,
                           false, pDeviceID, 2))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_AM1805_STATUS_SUCCESS;
}


//*****************************************************************************
//
//! @brief Configure the RTC initially
//!
//!
//! This function peforms a initial configuration: Set to 24 hour mode, autoreset after interrupts & configure the powerswitch, enable alarm interrupt
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t am_devices_am1805_config_init()
{
	//am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)pHandle;
    am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)my_IomdevHdl;
	uint32_t dataByte = 0;
	//
    // Send control 1 bits
    //
	dataByte = OPERATION24 + AUTORESET + POWERSWITCH;
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_CONTROL1,
                           false, &dataByte , 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	//
    // Send control 2 bits
    //
	dataByte = PSW_SLEEP;
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_CONTROL2,
                           false, &dataByte , 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	//
    // Send interrupt mask bits
    //
	dataByte = ALARM_INTERRUPT_ENABLE + INTERRUPT_MODE_RESET;
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_INTMASK,
                           false, &dataByte , 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	//
    // Send RPT to enable alarm for once per week configuration
    //	hex values maintain the default of the other register entries
	dataByte = RPT_ONCE_PER_DAY + 0x03 + 0x20;
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_TIMER_CONTROL,
                           false, &dataByte , 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	am_devices_am1805_read_register(&dataByte, ADDRESS_OSC_STATUS);
//	am_hal_gpio_pinconfig(PWRSAVERTC,g_AM_HAL_GPIO_OUTPUT);          //check if this is the correct implementation
//	gpioWrite(PWRSAVERTC,true);
	return AM_DEVICES_AM1805_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Read the current RTC time
//!
//! @param time_current - Struct that contains the current time to be set
//!
//! This function read the current time from the external RTC
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t am_devices_am1805_get_time(time_struct *time_current)
{
	//am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)pHandle;
    am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)my_IomdevHdl;
	uint8_t buffer[9];
	
	uint32_t dataByte = 0;
	//
    // Read current time
    //
    if (am_device_command_read(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_HUNDREDTHS,
                           false,(uint32_t *) &buffer , 8))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	time_current->hundredth = buffer[0];
	time_current->second = buffer[1];
	time_current->minute = buffer[2];
	time_current->hour = buffer[3];
	time_current->date = buffer[4];
	time_current->month = buffer[5];
	time_current->year = buffer[6];
	time_current->weekday = buffer[7];
	
	return AM_DEVICES_AM1805_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Set a alarm time
//!
//! @param alarm_time - Struct contains alarm time
//!
//! This function sets the alarm time 
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t am_devices_am1805_set_alarmTime(time_struct *alarm_time)
{
	//am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)pHandle;
    am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)my_IomdevHdl;
	uint8_t buffer[7];
	buffer[0] = alarm_time->hundredth;
	buffer[1] = alarm_time->second;
	buffer[2] = alarm_time->minute;
	buffer[3] = alarm_time->hour;
	buffer[4] = alarm_time->date;
	buffer[5] = alarm_time->month;
	buffer[6] = alarm_time->weekday;
	
	uint32_t dataByte = 0;
	//
    // write the alarm registers
    //
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_ALARM_HUNDREDTHS,
                           false,(uint32_t *) &buffer , 7))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }

	
	return AM_DEVICES_AM1805_STATUS_SUCCESS;
}


//*****************************************************************************
//
//! @brief Configure the RTC for sleep operation & initiate sleep
//!
//! @param sleepDays - Amount of Days the device should sleep
//!
//! @param sleepHours - Amount of hours the device should sleep
//!
//! @param sleepMinutes - Amount of minutes the device should sleep
//!
//! @param sleepSeconds - Amount of seconds the device should sleep
//!
//! This function configures the RTC to start a power down sleep phase
//! Idea: first read the current time, then add the desired sleeping time on top, write it into the alarm register & then go sleep
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t am_devices_am1805_config_sleep(uint8_t sleepDays, uint8_t sleepHours, uint8_t sleepMinutes, uint8_t sleepSeconds)
{
	//am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)pHandle;
    am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)my_IomdevHdl;

	time_struct time_current;
	uint8_t buffer[9];
	
	uint32_t dataByte = 0;
	am_hal_gpio_pinconfig(PWRSAVERTC,g_AM_HAL_GPIO_OUTPUT);          //check if this is the correct implementation
	gpioWrite(PWRSAVERTC,true);
	//
    // Read current time
    //
    if (am_devices_am1805_get_time(&time_current))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	/*
	#todo: need to catch second/minute/hour/day transitions that could occur (eg. 59minute + 25minute = +1 Hour + 24 minute 
	*/
	time_current.date = time_current.date + sleepDays;
	time_current.hour = time_current.hour + sleepHours;
	time_current.minute = time_current.minute + sleepMinutes;
	time_current.second = time_current.second + sleepSeconds;
	
	//
    // Set current time + sleep time as alarm
    //
    if (am_devices_am1805_set_alarmTime(&time_current))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }	
//	while(1){
//		am_devices_am1805_read_status(&dataByte);
//		am_util_stdio_printf("%d \n",dataByte);
//	}
	dataByte = CFG_KEY;		//Unlock writing to OSC_Control register
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_CONFIG_KEY,
                           false, &dataByte , 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	dataByte = PWGT;		//configure I/O interface to be disabled in power down mode, to reduce leakage
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_OSC_CONTROL,
                           false, &dataByte , 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	am_devices_am1805_read_register(&dataByte, ADDRESS_OSC_CONTROL);
//	am_util_stdio_printf("%d \n",dataByte);
//	gpioWrite(CS_LORA,false);
	//
    // Set current time + sleep time as alarm
    //
	dataByte = SLP;		//go to sleep without delay
    if (am_device_command_write(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           ADDRESS_SLEEP_CONTROL,
                           false, &dataByte , 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }
	return AM_DEVICES_AM1805_STATUS_SUCCESS;	
}


//*****************************************************************************
//
//! @brief Reads status register of the RTC
//!
//! @param pStatus - Pointer to the return buffer for the status register.
//!
//! This function reads the status register of the external rtc, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_am1805_read_status(uint32_t *pStatus)
{
    //am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)pHandle;
    am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)my_IomdevHdl;
    //
    // Send the command sequence to read the Device ID.
    //
    if (am_device_command_read(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           0x0F,
                           false, pStatus, 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_AM1805_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Reads given register of the RTC
//!
//! @param pStatus - Pointer to the return buffer for the register.
//!
//! @param register - Register number
//!
//! This function reads the given register of the external rtc, and returns
//! the result as an 32-bit unsigned integer value.
//!
//! @return 32-bit status
//
//*****************************************************************************
uint32_t
am_devices_am1805_read_register(uint32_t *pStatus, uint32_t reg)
{
    //am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)pHandle;
    am_devices_iom_am1805_t *pIom = (am_devices_iom_am1805_t *)my_IomdevHdl;
    //
    // Send the command sequence to read the Device ID.
    //
    if (am_device_command_read(pIom->pIomHandle, AM_DEVICES_AM1805_SLAVE_ID, 1,
                           reg,
                           false, pStatus, 1))
    {
        return AM_DEVICES_AM1805_STATUS_ERROR;
    }

    //
    // Return the status.
    //
    return AM_DEVICES_AM1805_STATUS_SUCCESS;
}
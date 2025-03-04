/*!
 *  \file       pins.h
 *
 *  \brief      This file acts as configuration file for different defines needed in the firmware.
 *
 *  \date       26.01.2023
 *
 *  \author     Timm Luhmann (IMTEK)
 */

#ifndef PINASSIGNMENT_H
#define PINASSIGNMENT_H

//---------------GPIOs-----------------
#define WP_FRAM 1
#define GPIO_LEFTBOARD_4 2
#define GPIO_RIGHTBOARD_4 4
#define SPIMASTER0CLK_TOPBOARD 5
#define SPIMASTER0MISO_TOPBOARD 6
#define SPIMASTER0MOSI_TOPBOARD 7
#define I2CSCL_LEFT_RIGHT_BOARD 8
#define I2CSDA_LEFT_RIGHT_BOARD 9
#define GPIO_RIGHTBOARD_3 10
#define GPIO_RIGHTBOARD_3A 12		//ADC capable
#define GPIO_RIGHTBOARD_4A 13		//ADC capable
#define GPIO_TOPBOARD_6 15
#define GPIO_TOPBOARD_2 16			//ADC capable
#define GPIO_TOPBOARD_1 17				
#define GPIO_TOPBOARD_3 18
#define GPIO_TOPBOARD_4 19
#define SWO_PIN 22              //used to generate ITM output to the printf terminal
#define ON_OFF_I2C 23			//if external pull-resistors for the I2C bus are used
#define GPIO_RIGHTBOARD_5 24
#define CHIP_SELECT_TOPBOARD 25
#define GPIO_TOPBOARD_7 26
#define GPIO_TOPBOARD_5 29			//ADC capable
#define GPIO_LEFTBOARD_1A 31		//ADC capable
#define GPIO_RIGHTBOARD_1A 32		//ADC capable
#define GPIO_LEFTBOARD_2A 34		//ADC capable
#define GPIO_RIGHTBOARD_2A 35		//ADC capable
#define GPIO_LEFTBOARD_3 36			
#define GPIO_LEFTBOARD_1 37
#define GPIO_LEFTBOARD_2 38
#define GPIO_RIGHTBOARD_1 39
#define GPIO_RIGHTBOARD_2 40
#define ON_OFF_FRAM 41
#define ON_OFF_DAC 42
#define PWRSAVERTC 44
#define UART0TX 48
#define UART0RX 49

/*
 * These are the switches on the Lora Board PE4259, PAY ATTENTION: The Logic in LAMBDA62-2 Datasheet is vice versa!
 */
#define ANT_SW_PIN      29         /*!< TX_SW_PIN */
#define RX_SW_PIN       19         // RX_SW_PIN


#endif
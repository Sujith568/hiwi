/*!
 *  \file       timing.h
 *
 *  \brief      Provides functions to configure the system Timer
 *
 *  \date       02.02.2023
 *
 *  \author     Uttunga Shinde (IMTEK)
 */
#ifndef TIMING_H
#define TIMING_H

#include "am_mcu_apollo.h"


void myTimerInit(uint8_t state);

void am_stimer_cmpr0_isr();

void setTimerFreq(uint32_t frequency);

void setTimerVoltageOut(float voltage);

void myTimerDeInit(void);

void timerDelay(uint32_t ms);
#endif

// TI File $Revision: /main/1 $
// Checkin $Date: August 14, 2008   16:57:50 $
//###########################################################################
//
// FILE:   DSP28x_Project.h
//
// TITLE:  DSP28x Project Headerfile and Examples Include File
//
//###########################################################################
// $TI Release: LaunchPad f2802x Support Library v100 $
// $Release Date: Wed Jul 25 10:45:39 CDT 2012 $
//###########################################################################

#ifndef DSP28x_PROJECT_H
#define DSP28x_PROJECT_H

#include "F2802x_Device.h"     // DSP2802x Headerfile Include File
#include "F2802x_Examples.h"   // DSP2802x Examples Include File


#define   USE_F28027_ADC      1
#define   USE_F28027_CAP      0
#define   USE_F28027_CLK      1
#define   USE_F28027_COMP     0
#define   USE_F28027_CPU      1
#define   USE_F28027_FLASH    1
#define   USE_F28027_GPIO     1
#define   USE_F28027_I2C      0
#define   USE_F28027_OSC      1
#define   USE_F28027_PIE      1
#define   USE_F28027_PLL      1
#define   USE_F28027_PWM      1
#define   USE_F28027_PWR      1
#define   USE_F28027_SCI_IO   0
#define   USE_F28027_SCI      0
#define   USE_F28027_SPI      0
#define   USE_F28027_TIMER    1
#define   USE_F28027_WDOG     0
#define   USE_F28027_NMI      0
#define   USE_F28027_PARTID   0
#define   USE_F28027_SYSCTRL  1
#define   USE_F28027_EMU      0
#define   USE_F28027_XINTS    0
#define   USE_F28027_DEVEMU   0
#define   USE_F28027_CSM      0


//! \brief External reference to the interrupt flag register (IFR) register
//cregister volatile unsigned int IFR;

//! \brief External reference to the interrupt enable register (IER) register
//cregister volatile unsigned int IER;


#endif  // end of DSP28x_PROJECT_H definition


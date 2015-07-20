/* ************************************************************************** */
/* ************************************************************************** */
#ifndef __INIT_H__
#define __INIT_H__

/* ************************************************************************** */

#include "./include/DSP28x_Project.h"
#include "./include/F2802x_Examples.h"
#include "./include/F2802x_GlobalPrototypes.h"

#include "./include/cpu.h"
#include "./include/clk.h"
#include "./include/flash.h"
#include "./include/gpio.h"
#include "./include/pll.h"
#include "./include/pwm.h"
#include "./include/wdog.h"
#include "./include/pie.h"
//#include "./include/pie_init.h"
#include "./include/timer.h"
#include "./include/adc.h"
#include "./include/sci.h"

//#include "./F28027_SCI.h"
//#include "TM1638.h"

//#include "n5110.h"
//#include "5110.h"

#include "types.h"
#include "init.h"
//#include "interrupts.h"


/* ************************************************************************** */
//..............................................................................



/* ************************************************************************** */
#define __AVR__           (0)
#define __TMS320__        (1)

#define __USE_TM1638__    (0)
#define __USE_LCD_5110__  (1)
#define __USE_DS1307__    (0)

/* ************************************************************************** */

/* ************************************************************************** */
#define gpio_mux  0x0000
#define gpio_dir  0x001f

/*
	uint16_t RamfuncsLoadStart;
	uint16_t RamfuncsLoadSize;
	uint16_t RamfuncsRunStart;
*/

//..............................................................................
// Used for running BackGround in flash, and ISR in RAM
///*extern*/ Uint32 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
///*extern*/ Uint32 *IQfuncsLoadStart, *IQfuncsLoadEnd, *IQfuncsRunStart;
extern uint16_t RamfuncsLoadStart, RamfuncsLoadEnd, RamfuncsRunStart;
//extern uint16_t IQfuncsLoadStart, IQfuncsLoadEnd, IQfuncsRunStart;


/* ========================================================================== */
// Prototype statements for functions found within this file.
void     InitFlash (void);
void     MemCopy (Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

void     Init_All ( void );
void     init_Cfg_EPwmTimers (void);
t_error  Init_Sys    (void);
t_error  Init_PWM    (void);
t_error  Init_GPIO   (void);
t_error  Init_Timer0 (void);
t_error  Init_UART_IRQ (void);
t_error  Init_TM1638 (void);
t_error  Init_PLL    (void);
t_error  Init_FLASH  (void);
t_error  Init_ADC    (void);
void     scia_fifo_init(void);

void     wrapper_Main ( void );
void     Error( t_error err );
void     error(void);
//t_error  wrapper_Init_UART_pooling (void);
//t_error  wrapper_Init_UART_IRQ (void);
/* ========================================================================== */

void DELAY(char s);

/* ************************************************************************** */

#endif //__INIT_H__
/* ************************************************************************** */

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

/* ========================================================================== */

#if (CPU_FRQ_40MHZ)
#define CPU_FREQ  (40000000) //Default=40 MHz Change to 50E6 for 50 MHz devices
#endif

#if (CPU_FRQ_50MHZ)
#define CPU_FREQ  (50000000) //Default=40 MHz Change to 50E6 for 50 MHz devices
#endif

#if (CPU_FRQ_60MHZ)
#define CPU_FREQ  (60000000) //Default=40 MHz Change to 50E6 for 50 MHz devices
#endif

#define LSPCLK_FREQ   (CPU_FREQ/4)

//#define SCI_FREQ          100E3
#define SCI_FREQ          115200
//#define SCI_PRD           (LSPCLK_FREQ/(SCI_FREQ*8))-1
#define SCI_BRR           (LSPCLK_FREQ/(SCI_FREQ*8))-1
/* ========================================================================== */

/* ========================================================================== */
// Maximum Dead Band values
#define EPWM1_MAX_DB   0x03FF
#define EPWM2_MAX_DB   0x03FF
#define EPWM3_MAX_DB   0x03FF

#define EPWM1_MIN_DB   0
#define EPWM2_MIN_DB   0
#define EPWM3_MIN_DB   0

// To keep track of which way the Dead Band is moving
#define DB_UP   1
#define DB_DOWN 0

/* ========================================================================== */
#define PWM_UP_MODE        0
#define PWM_DOWN_MODE      1
#define PWM_UP_DOWN_MODE   2
#define PWM_FREQ_SINUS     ( 50 )  // HZ
//#define STEPS            ( 360 ) // degree in sinus
#define STEPS              ( 90 )  // degree in sinus

#define PWM1_FREQ_SWITCH   ( PWM_FREQ_SINUS * STEPS ) / (2+10)
#define PWM1_FREQ_PERIOD   ( CPU_FREQ/PWM1_FREQ_SWITCH ) / PWM_UP_DOWN_MODE

#define PWM2_FREQ_SWITCH   ( PWM_FREQ_SINUS * STEPS ) / (2+10)
#define PWM2_FREQ_PERIOD   ( CPU_FREQ/PWM2_FREQ_SWITCH ) / PWM_UP_DOWN_MODE

#define TIME_PERIOD_1MS    (60 * 1000)
#define TIME_PERIOD_10MS   (60 * 10000)
#define TIME_PERIOD_20MS   (60 * 20000)
#define TIME_PERIOD_30MS   (60 * 30000)
#define TIME_PERIOD_40MS   (60 * 40000)
#define TIME_PERIOD_50MS   (60 * 50000)
#define TIME_PERIOD_100MS  (60 * 100000)
#define TIME_PERIOD_500MS  (60 * 500000)
#define TIME_PERIOD_1S     (60 * 1000000)
//#define TIME_PERIOD_MIN    (0xFFFF) // OK

//#define TIME_PERIOD_MIN    11000 //(0x2FFF) // TEST
//#define TIME_PERIOD_MIN    30000 //(0x2FFF) // OK
#define TIME_PERIOD_MIN    65000 //(0x2FFF) // TEST

#define TMR0__TIME_PERIOD  (TIME_PERIOD_MIN)
/* ========================================================================== */
// Configure which ePWM timer interrupts are enabled at the PIE level:
// 1 = enabled,  0 = disabled
#define  PWM1_INT_ENABLE  1
#define  PWM2_INT_ENABLE  1
#define  PWM3_INT_ENABLE  1

// Configure the period for each timer
#define PWM1_TIMER_TBPRD  0xFFFF
//#define PWM1_TIMER_TBPRD  0x1FFF

#define PWM2_TIMER_TBPRD  0x1FFF

//#define PWM3_TIMER_TBPRD  0x1FFF
#define PWM3_TIMER_TBPRD  0xFFFF


/* ========================================================================== */
//#define  USE_UART_IRQ_1   (0)
//#define  USE_UART_IRQ_2   (1)

/* ************************************************************************** */


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
t_error  ReInit_PWM_adc_on_VarResistor (void);
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

/* ========================================================================== */
/* ========================================================================== */
//  Group:          C2000
//  Target Device:  TMS320F2802x
/* ========================================================================== */
/* ========================================================================== */

#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>

#include "types.h"
#include "init.h"
#include "interrupts.h"

/*
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
*/

#include "n5110.h"
#include "5110.h"



/* ========================================================================== */
/* ========================================================================== */

/* ========================================================================== */
/* Global variables */
/* ========================================================================== */
//..............................................................................
int          i=0, i_tx=0, i_rx=0;
int          i_pwm3=0;

//..............................................................................
#define     RX_LEN  (8)

char        *p_sci_msg;
uint16_t     LoopCount;
uint16_t     ErrorCount;
uint16_t     ReceivedChar;
uint16_t     sdataA[RX_LEN];     // Send data for SCI-A
uint16_t     rdataA[RX_LEN];     // Received data for SCI-A
uint16_t     rdata_pointA;       // Used for checking the received data
uint16_t     RxTx;
char        lcd_buff[8] = "       \n";


//..............................................................................


//..............................................................................

/* ========================================================================== */



/* ==========================================================================
 * MAIN
 * ========================================================================== */
void main (void)
{
	Init_All();

#if (1==__USE_LCD_5110__)
	Lcd_clear();
	Lcd_init();
	//LCD_PrintToScreen();

	// Main code
    for(;;)
    {
    	//wrapper_Main();

    	//tm1638_prints("123456789");
    	//tm1638_printx("1", 1);

    	//Lcd_prints ( 0, 0, FONT_1X, "ADC0:     " );
    	//ltoa( adc_data[0], (char *)&lcd_buff );
    	//Lcd_prints ( 6, 0, FONT_1X, (byte *)lcd_buff );

    	//Lcd_pixel(0, 0, PIXEL_ON);
    	//Lcd_pixel(1, 1, PIXEL_ON);
    	//Lcd_pixel(2, 2, PIXEL_ON);
    	//Lcd_pixel(3, 3, PIXEL_ON);

    	//Lcd_update();
    }
#endif //(1==__USE__LCD_5110__)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Wrapper_Main
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void wrapper_Main ( void ) {
#if 0
    /* asm (" NOP");
    for ( i=1; i<=10; i++ ) {
        GPIO_setPortData (myGpio, GPIO_Port_A, i );
    }  */

	// Describe of functionality
	// - UART Terminal for control
	// -- Timer #0 as base CLI (Command Line Interface)
	// -- CLI (Command Line Interface)
	// -- PWM #0, #1, #2 : On-Off
	// -- UART_PWM_generator
	// -- LCD and LEDs
	// -- GPIO


	// -- GPIO
/*
	uint16_t  ui16_gpio_in;
	ui16_gpio_in = GPIO_getPortData (myGpio, GPIO_Port_A);
	if ( ui16_gpio_in & (1<<12) ) {
		//GPIO_setLow(myGpio, GPIO_Number_0 );
		GPIO_setPortData (myGpio, GPIO_Port_A, 0xF );
		//SCI_write(mySci, '3');
	} else {
		//GPIO_setHigh(myGpio, GPIO_Number_0 );
		GPIO_setPortData (myGpio, GPIO_Port_A, 0xE );
	}
*/
	/*if (0)
    {
    	t_terminal_state  terminal_state;
		static uint16_t   try = 0xffff;

    	switch ( terminal_state ){
    		case TERMINAL_PRINT:
    	    	p_sci_msg = "\rEnter a character: \0";
    	        scia_msg(p_sci_msg);
    			terminal_state = TERMINAL_READ;
    		break;

    		case TERMINAL_READ:
    	        if (SCI_getRxFifoStatus(mySci) >= SCI_FifoStatus_1_Word) {
    	            ReceivedChar = SCI_getData(mySci); // Get character
        			terminal_state = TERMINAL_SHOW;
    	        } else {
        			if (--try)  terminal_state = TERMINAL_READ;
        			else        terminal_state = TERMINAL_SHOW;
    	        }
    		break;

    		case TERMINAL_SHOW:
    			try=0xffff;
	            // Echo character back
	            p_sci_msg = "  You sent: \0";
	    		terminal_state = TERMINAL_ENTER;
    		break;

    		case TERMINAL_ENTER:
	            scia_msg(p_sci_msg);
	            scia_xmit(ReceivedChar);
	            //LoopCount++;
	    		terminal_state = TERMINAL_WAIT;
    		break;

    		case TERMINAL_WAIT:
    	        // Wait for inc character
    	        //while (SCI_getRxFifoStatus(mySci) < SCI_FifoStatus_4_Words) {}
	    		terminal_state = TERMINAL_PRINT;
	        break;

    		default:
	    		terminal_state = TERMINAL_PRINT;
    		break;
    	}
    }*/
#endif //0
}
/* ========================================================================== */



//===========================================================================
// No more.
//===========================================================================

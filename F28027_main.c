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
//Sinus
//Sin table values
int sinus360[] = {
	// http://www.meraman.com/htmls/en/sinTableOld.html
	// 360 values
	127, 129, 131, 134, 136, 138, 140, 142, 145, 147, 149, 151, 153, 156, 158, 160,
	162, 164, 166, 168, 170, 173, 175, 177, 179, 181, 183, 185, 187, 189, 191, 192,
	194, 196, 198, 200, 202, 203, 205, 207, 209, 210, 212, 214, 215, 217, 218, 220,
	221, 223, 224, 226, 227, 228, 230, 231, 232, 234, 235, 236, 237, 238, 239, 240,
	241, 242, 243, 244, 245, 246, 246, 247, 248, 248, 249, 250, 250, 251, 251, 252,
	252, 252, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254,
	253, 253, 253, 252, 252, 252, 251, 251, 250, 250, 249, 248, 248, 247, 246, 246,
	245, 244, 243, 242, 241, 240, 239, 238, 237, 236, 235, 234, 232, 231, 230, 228,
	227, 226, 224, 223, 221, 220, 218, 217, 215, 214, 212, 210, 209, 207, 205, 203,
	202, 200, 198, 196, 194, 192, 191, 189, 187, 185, 183, 181, 179, 177, 175, 173,
	170, 168, 166, 164, 162, 160, 158, 156, 153, 151, 149, 147, 145, 142, 140, 138,
	136, 134, 131, 129, 127, 125, 123, 120, 118, 116, 114, 112, 109, 107, 105, 103,
	101, 98, 96, 94, 92, 90, 88, 86, 84, 81, 79, 77, 75, 73, 71, 69, 67, 65, 63, 62,
	60, 58, 56, 54, 52, 51, 49, 47, 45, 44, 42, 40, 39, 37, 36, 34, 33, 31, 30, 28,
	27, 26, 24, 23, 22, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6,
	5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 2, 2,
	2, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
	22, 23, 24, 26, 27, 28, 30, 31, 33, 34, 36, 37, 39, 40, 42, 44, 45, 47, 49, 51,
	52, 54, 56, 58, 60, 62, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 84, 86, 88, 90,
	92, 94, 96, 98, 101, 103, 105, 107, 109, 112, 114, 116, 118, 120, 123, 125
};

int sinus360_1[] = {
	// http://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml
	// 360 values
	0x80,0x82,0x84,0x86,0x88,0x8b,0x8d,0x8f,
	0x91,0x93,0x96,0x98,0x9a,0x9c,0x9e,0xa0,
	0xa3,0xa5,0xa7,0xa9,0xab,0xad,0xaf,0xb1,
	0xb3,0xb5,0xb7,0xb9,0xbb,0xbd,0xbf,0xc1,
	0xc3,0xc5,0xc7,0xc9,0xca,0xcc,0xce,0xd0,
	0xd1,0xd3,0xd5,0xd6,0xd8,0xda,0xdb,0xdd,
	0xde,0xe0,0xe1,0xe3,0xe4,0xe5,0xe7,0xe8,
	0xe9,0xea,0xec,0xed,0xee,0xef,0xf0,0xf1,
	0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf7,0xf8,
	0xf9,0xf9,0xfa,0xfb,0xfb,0xfc,0xfc,0xfd,
	0xfd,0xfd,0xfe,0xfe,0xfe,0xff,0xff,0xff,
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
	0xfe,0xfe,0xfe,0xfd,0xfd,0xfd,0xfc,0xfc,
	0xfb,0xfb,0xfa,0xf9,0xf9,0xf8,0xf7,0xf7,
	0xf6,0xf5,0xf4,0xf3,0xf2,0xf1,0xf0,0xef,
	0xee,0xed,0xec,0xea,0xe9,0xe8,0xe7,0xe5,
	0xe4,0xe3,0xe1,0xe0,0xde,0xdd,0xdb,0xda,
	0xd8,0xd6,0xd5,0xd3,0xd1,0xd0,0xce,0xcc,
	0xca,0xc9,0xc7,0xc5,0xc3,0xc1,0xbf,0xbd,
	0xbb,0xb9,0xb7,0xb5,0xb3,0xb1,0xaf,0xad,
	0xab,0xa9,0xa7,0xa5,0xa3,0xa0,0x9e,0x9c,
	0x9a,0x98,0x96,0x93,0x91,0x8f,0x8d,0x8b,
	0x88,0x86,0x84,0x82,0x80,0x7d,0x7b,0x79,
	0x77,0x74,0x72,0x70,0x6e,0x6c,0x69,0x67,
	0x65,0x63,0x61,0x5f,0x5c,0x5a,0x58,0x56,
	0x54,0x52,0x50,0x4e,0x4c,0x4a,0x48,0x46,
	0x44,0x42,0x40,0x3e,0x3c,0x3a,0x38,0x36,
	0x35,0x33,0x31,0x2f,0x2e,0x2c,0x2a,0x29,
	0x27,0x25,0x24,0x22,0x21,0x1f,0x1e,0x1c,
	0x1b,0x1a,0x18,0x17,0x16,0x15,0x13,0x12,
	0x11,0x10,0xf,0xe,0xd,0xc,0xb,0xa,
	0x9,0x8,0x8,0x7,0x6,0x6,0x5,0x4,
	0x4,0x3,0x3,0x2,0x2,0x2,0x1,0x1,
	0x1,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x1,0x1,0x1,0x2,
	0x2,0x2,0x3,0x3,0x4,0x4,0x5,0x6,
	0x6,0x7,0x8,0x8,0x9,0xa,0xb,0xc,
	0xd,0xe,0xf,0x10,0x11,0x12,0x13,0x15,
	0x16,0x17,0x18,0x1a,0x1b,0x1c,0x1e,0x1f,
	0x21,0x22,0x24,0x25,0x27,0x29,0x2a,0x2c,
	0x2e,0x2f,0x31,0x33,0x35,0x36,0x38,0x3a,
	0x3c,0x3e,0x40,0x42,0x44,0x46,0x48,0x4a,
	0x4c,0x4e,0x50,0x52,0x54,0x56,0x58,0x5a,
	0x5c,0x5f,0x61,0x63,0x65,0x67,0x69,0x6c,
	0x6e,0x70,0x72,0x74,0x77,0x79,0x7b,0x7d
};

int sinus[] = {
	// http://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml
	// 84 values
	0x80,0x89,0x93,0x9c,0xa5,0xae,0xb7,0xbf,
	0xc7,0xcf,0xd6,0xdd,0xe3,0xe9,0xee,0xf2,
	0xf6,0xf9,0xfc,0xfe,0xff,0xff,0xff,0xfe,
	0xfc,0xf9,0xf6,0xf2,0xee,0xe9,0xe3,0xdd,
	0xd6,0xcf,0xc7,0xbf,0xb7,0xae,0xa5,0x9c,
	0x93,0x89,0x80,0x76,0x6c,0x63,0x5a,0x51,
	0x48,0x40,0x38,0x30,0x29,0x22,0x1c,0x16,
	0x11,0xd,0x9,0x6,0x3,0x1,0x0,0x0,
	0x0,0x1,0x3,0x6,0x9,0xd,0x11,0x16,
	0x1c,0x22,0x29,0x30,0x38,0x40,0x48,0x51,
	0x5a,0x63,0x6c,0x76,0x80
};

int sinus180[] = {
	// http://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml
	// 180 values
	0x80,0x84,0x89,0x8d,0x92,0x96,0x9b,0x9f,
	0xa3,0xa8,0xac,0xb0,0xb4,0xb8,0xbc,0xc0,
	0xc4,0xc8,0xcb,0xcf,0xd2,0xd6,0xd9,0xdc,
	0xdf,0xe2,0xe5,0xe8,0xea,0xed,0xef,0xf1,
	0xf3,0xf5,0xf7,0xf8,0xfa,0xfb,0xfc,0xfd,
	0xfe,0xff,0xff,0x100,0x100,0x100,0x100,0x100,
	0xff,0xff,0xfe,0xfd,0xfc,0xfb,0xfa,0xf8,
	0xf7,0xf5,0xf3,0xf1,0xef,0xed,0xea,0xe8,
	0xe5,0xe2,0xdf,0xdc,0xd9,0xd6,0xd2,0xcf,
	0xcb,0xc8,0xc4,0xc0,0xbc,0xb8,0xb4,0xb0,
	0xac,0xa8,0xa3,0x9f,0x9b,0x96,0x92,0x8d,
	0x89,0x84,0x80,0x7c,0x77,0x73,0x6e,0x6a,
	0x65,0x61,0x5d,0x58,0x54,0x50,0x4c,0x48,
	0x44,0x40,0x3c,0x38,0x35,0x31,0x2e,0x2a,
	0x27,0x24,0x21,0x1e,0x1b,0x18,0x16,0x13,
	0x11,0xf,0xd,0xb,0x9,0x8,0x6,0x5,
	0x4,0x3,0x2,0x1,0x1,0x0,0x0,0x0,
	0x0,0x0,0x1,0x1,0x2,0x3,0x4,0x5,
	0x6,0x8,0x9,0xb,0xd,0xf,0x11,0x13,
	0x16,0x18,0x1b,0x1e,0x21,0x24,0x27,0x2a,
	0x2e,0x31,0x35,0x38,0x3c,0x40,0x44,0x48,
	0x4c,0x50,0x54,0x58,0x5d,0x61,0x65,0x6a,
	0x6e,0x73,0x77,0x7c,0x80
};


int sinus90[] = {
	// http://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml
	// 90 values
#if 0
	/* 0*/	0x80, 0x89, 0x92, 0x9b, 0xa3,
	/* 5*/	0xac, 0xb4, 0xbc, 0xc4, 0xcb,
	/*10*/	0xd2, 0xd9, 0xdf, 0xe5, 0xea,
	/*15*/	0xef, 0xf3, 0xf7, 0xfa, 0xfc,
	/*20*/	0xfe, 0xff,0x100, 0x100,0xff,
	/*25*/	0xfe, 0xfc, 0xfa, 0xf7, 0xf3,
	/*30*/	0xef, 0xea, 0xe5, 0xdf, 0xd9,
	/*35*/	0xd2, 0xcb, 0xc4, 0xbc, 0xb4,
	/*40*/	0xac, 0xa3, 0x9b, 0x92, 0x89,
	/*45*/	0x80, 0x77, 0x6e, 0x65, 0x5d,
	/*50*/	0x54, 0x4c, 0x44, 0x3c, 0x35,
	/*55*/	0x2e, 0x27, 0x21, 0x1b, 0x16,
	/*60*/	0x11, 0x0d, 0x09, 0x06, 0x04,
	/*65*/	0x02, 0x01, 0x00, 0x00, 0x01,
	/*70*/	0x02, 0x04, 0x06, 0x09, 0x0d,
	/*75*/	0x11, 0x16, 0x1b, 0x21, 0x27,
	/*80*/	0x2e, 0x35, 0x3c, 0x44, 0x4c,
	/*85*/	0x54, 0x5d, 0x65, 0x6e, 0x77,
	/*90*/	0x80
#endif //0
	0x200,0x224,0x247,0x26a,
	0x28d,0x2af,0x2d0,0x2f0,
	0x30f,0x32d,0x349,0x364,
	0x37c,0x393,0x3a8,0x3bb,
	0x3cc,0x3db,0x3e7,0x3f1,
	0x3f8,0x3fd,0x400,0x400,
	0x3fd,0x3f8,0x3f1,0x3e7,
	0x3db,0x3cc,0x3bb,0x3a8,
	0x393,0x37c,0x364,0x349,
	0x32d,0x30f,0x2f0,0x2d0,
	0x2af,0x28d,0x26a,0x247,
	0x224,0x200,0x1dc,0x1b9,
	0x196,0x173,0x151,0x130,
	0x110,0xf1,0xd3,0xb7,
	0x9c,0x84,0x6d,0x58,
	0x45,0x34,0x25,0x19,
	0xf,0x8,0x3,0x0,
	0x0,0x3,0x8,0xf,
	0x19,0x25,0x34,0x45,
	0x58,0x6d,0x84,0x9c,
	0xb7,0xd3,0xf1,0x110,
	0x130,0x151,0x173,0x196,
	0x1b9,0x1dc,0x200,
};

int sinus45[] = {
	// http://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml
	0x400,0x48f,0x51a,0x5a0,
	0x61f,0x692,0x6f9,0x751,
	0x798,0x7ce,0x7f0,0x7ff,
	0x7fa,0x7e2,0x7b5,0x777,
	0x727,0x6c7,0x65a,0x5e1,
	0x55e,0x4d5,0x447,0x3b9,
	0x32b,0x2a2,0x21f,0x1a6,
	0x139,0xd9,0x89,0x4b,
	0x1e,0x6,0x1,0x10,
	0x32,0x68,0xaf,0x107,
	0x16e,0x1e1,0x260,0x2e6,
	0x371,0x400
};

/* ========================================================================== */

/* ========================================================================== */
/* Global variables */
/* ========================================================================== */
extern CPU_Handle   myCpu;
extern PLL_Handle   myPll;
extern CLK_Handle   myClk;
extern FLASH_Handle myFlash;
extern ADC_Handle   myAdc;
extern SCI_Handle   mySci;
extern GPIO_Handle  myGpio;
extern PIE_Handle   myPie;
extern PWM_Handle   myPwm1, myPwm2, myPwm3;
extern TIMER_Handle myTimer;
extern WDOG_Handle  myWDog;

extern t_error      rc;
extern t_status     sys_stat;

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
char         lcd_buff[8] = "       \n";

//..............................................................................
extern volatile uint16_t     update_LCD_flag;
extern volatile uint16_t     update_data_from_adc;

extern uint16_t   adc_data_array[84/*LCD_X_RES*/];
extern uint16_t   adc_data      [16];
extern uint16_t   adc_on_VarResistor;

extern uint16_t   pwm1_Period;
extern int        adc_T_sampling_ns;

extern uint32_t   EPwm1TimerIntCount;
extern uint32_t   EPwm2TimerIntCount;
extern uint32_t   EPwm3TimerIntCount;

//..............................................................................
typedef enum {
	LCD_UPD_PREPARE = 0,
	LCD_UPD_UPDATE
} lcd_update_seq_t;

lcd_update_seq_t lcd_update_seq = LCD_UPD_PREPARE;

/* ========================================================================== */


/* ==========================================================================
 * adc_find_offset_for_start
 * ========================================================================== */
uint16_t adc_find_offset_for_start (char mode)
{
	uint16_t  cnt, start_offfset=0;
	uint16_t  now, next, next2;

	if ( mode == 'q' || mode == 'Q' ) // find if square
	{
		for ( cnt=0; cnt<84; cnt++ )
		{
			now=adc_data_array[cnt];
			next=adc_data_array[cnt+1];
			next2=adc_data_array[cnt+2];
			if ( now < next && next < next2 )
			{
				start_offfset = cnt;

				return start_offfset;
			}
		}
	}

	return 0;
}


/* ==========================================================================
 * adc_find_center_for_sinus
 * ========================================================================== */
uint16_t adc_find_center_for_sinus (char mode)
{
	uint16_t  cnt, start_offfset=0;

	if ( mode == 's' || mode == 'S' ) // find if square
	{
		long SinusRMS=0;

		for ( cnt=0; cnt<84; cnt++ )
		{
			SinusRMS += sinus[cnt];
		}
		start_offfset = SinusRMS/84;

		return start_offfset;
	}

	return 0;
}


/* ==========================================================================
 * MAIN
 * ========================================================================== */
void main (void)
{
	//Init_All();
	if (E_OK==rc)  rc = Init_Sys();   else  Error(rc); // Init system and handles
    if (E_OK==rc)  rc = Init_PLL();   else  Error(rc); // Init PLL
    if (E_OK==rc)  rc = Init_FLASH(); else  Error(rc); // Init FLASH
    if (E_OK==rc)  rc = Init_GPIO();  else  Error(rc); // Init GPIO system
    if (E_OK==rc)  rc = Init_PWM();   else  Error(rc); // Init IRQs
    //if (E_OK==rc)  rc = Init_Timer0(); else  Error(rc); // Init Timer0
    //if (E_OK==rc)  rc = Init_ADC();   else  Error(rc); // Init ADC
    //if (E_OK==rc)  rc = Init_UART_IRQ(); else  Error(rc); // Init UART IRQ
    //rc = Init_UART_pooling (); // Init UART without IRQ
    //if (E_OK==rc)  rc = Init_TM1638();  else  Error(rc); // Init TM1638
    if (E_OK==rc) { sys_stat.sys.error = E_OK; }

#if (1==__USE_LCD_5110__)
	Lcd_clear();
	Lcd_init();
	//LCD_PrintToScreen();

	// Main code
    for(;;)
    {
#if (0)
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

    	if ( update_data_from_adc )
    	{
    		static uint16_t  cnt, cnt_Update;
    		static uint16_t  xx=0;
    		static uint16_t  x1=0, y1=0, x2=0, y2=0;
    		uint16_t         adc_lcd_offset , sinus_center;
    		volatile int     dbg=0;

    		adc_lcd_offset = adc_find_offset_for_start('q');

    		adc_lcd_offset = 0;

    		sinus_center = adc_find_center_for_sinus('s');

			dbg++;

    		//for ( cnt=0; cnt<84; cnt++ )
        	for ( cnt=adc_lcd_offset; cnt < 84+adc_lcd_offset; cnt++ )
        	{
    			/*if ( cnt == adc_lcd_offset )
    				dbg++;

    			if ( cnt == 84-adc_lcd_offset )
    				dbg++; */

    			//xx++;
    			if ( ++xx >= 84+adc_lcd_offset /*LCD_X_RES*/ ) {
    				xx=0;
    			}

    			x2 = xx;
    		    y2 = sinus[cnt-adc_lcd_offset]/7;    /*test*/
    			//y2 = adc_data_array[cnt]/35;
    			Lcd_line( x2, 0, x2, LCD_Y_RES-1, PIXEL_OFF );
                if ( x1 != 0 ) {
                	Lcd_line( x1, y1, x2, y2, PIXEL_ON );  // as lines - (Slow)
    			} else {
                	Lcd_pixel( x2, y2, PIXEL_ON );     // as dots  - (Fast)
        		}
    			//Lcd_pixel( x2, y2, PIXEL_ON );     // as dots  - (Fast)

    			if ( x2 < ( LCD_X_RES-1+adc_lcd_offset ) )
    			{
    				x1 = x2;
    				y1 = y2;
    			} else {
    				x1 = 0;
    				y1 = 0;
    			}

    			cnt_Update++;

    			if ( cnt_Update> 5 * 84 )
    			{
    				update_LCD_flag = 1;
    				cnt_Update = 0;
    				lcd_update_seq = LCD_UPD_PREPARE;
    			}

    			update_data_from_adc = 0;
    		}

        	//Lcd_update();
        	if ( update_LCD_flag )
        	{
        		//long tmp=0;

        		//switch (lcd_update_seq)
        		{
    				//case LCD_UPD_PREPARE:
    					//Lcd_clear();
    				    //ce_l;            // чип позволит 0xFB = 11111011
    					lcd_update_seq = LCD_UPD_UPDATE;

    					//Lcd_line(0, 0,  LCD_X_RES, 0,  PIXEL_ON);  // as lines - (Slow)
    					Lcd_line(0, (sinus_center/7)-1, LCD_X_RES, (sinus_center/7)-1, PIXEL_ON);  // as lines - (Slow)
    					//Lcd_line(0, 39, LCD_X_RES, 39, PIXEL_ON);  // as lines - (Slow)

    		        	Lcd_prints ( 0, 5, FONT_1X, "ADC Ts:     nS" );
    		        	//ltoa( adc_on_VarResistor, (char *)&lcd_buff[0] );
    		        	ltoa( adc_T_sampling_ns, (char *)&lcd_buff[0] );
    		        	Lcd_prints ( 8, 5, FONT_1X, (byte *)lcd_buff );
    		        //break;

    				//case LCD_UPD_UPDATE:
    		    		Lcd_update();
    		    		ReInit_PWM_adc_on_VarResistor();
    		    		update_LCD_flag = 0;
    		    	//break;
        		}
        	}

    	}
#endif //(0)
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

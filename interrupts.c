#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>

#include "types.h"
#include "init.h"
#include "interrupts.h"

#include "n5110.h"
//#include "5110.h"

//..............................................................................

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


#define ADC_NUMS  (2*84/*LCD_X_RES*/)

//..............................................................................
// ADC variables
//..............................................................................
//ADC_Obj  *adc;
typedef   enum { ADC_INIT=0, ADC_READY }  stat_adc_t;
uint16_t   adc_offset    [16];
uint16_t   adc_data      [16];
uint16_t   adc_data_array[ADC_NUMS];
uint16_t   adc_on_VarResistor;
uint16_t   ConvCount=0;
uint16_t   TempSensorVoltage[10];
stat_adc_t stat_adc=ADC_INIT;

uint16_t   PWM_Ramp = (STEPS/2)+10;
uint16_t   PWM_Ramp_TMP = 0;
uint16_t   PWM_StartUP=0;

//..............................................................................
uint32_t   EPwm1TimerIntCount;
uint32_t   EPwm2TimerIntCount;
uint32_t   EPwm3TimerIntCount;
uint32_t   Timer0IntCount;

uint16_t   pwm1_duty;
uint16_t   pwm1_duty_val;

extern t_status  sys_stat;
extern int sinus[];
extern int sinus45[];
extern int sinus90[];
extern int sinus180[];
extern int sinus360[];
extern int sinus360_1[];

volatile uint16_t     update_LCD_flag;
volatile uint16_t     update_data_from_adc=0;

//..............................................................................



/* ==========================================================================
 * NAME - epwm1_timer_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   http://www.daycounter.com/Calculators/Sine-Generator-Calculator2.phtml
   ========================================================================== */
// Interrupt routines uses in this example:
interrupt void epwm1_timer_isr (void) {
#if (1==USE_F28027_PWM)
#if (1==PWM1_INT_ENABLE)
	static uint16_t tetha=0;
	//static uint16_t safa_begin=0;

	//SinusRMS += sinus[cnt];
	//PWM_setPeriod(myPwm1, sinus[cnt] );
	//PWM_setCmpA(myPwm1, EPWM1_MIN_DB+(1*sinus90[cnt]) );
	//PWM_setCmpB(myPwm1, /*PWM2_FREQ_PERIOD-*/EPWM1_MIN_DB+(1*sinus90[cnt]) );

#if 0 // 360 steps
	if ( tetha >= STEPS ) tetha=0;

	pwm1_duty = EPWM1_MIN_DB+sinus360[tetha]/2;
	if ( tetha<STEPS/2 )
	{
		PWM_setCmpA(myPwm1, /*0x300*/0 );
		PWM_setCmpB(myPwm1, 1*(pwm1_duty) );
	}
	else
	{
		PWM_setCmpA(myPwm1, 1*(pwm1_duty) );
		PWM_setCmpB(myPwm1, /*0x300*/0x0 );
	}
	tetha += 1;
#endif //0

#if 1 // 90 steps - sinus OK

	pwm1_duty = ((sinus180[tetha]));

	if ( tetha > (STEPS/2) ) {
		// increase ramp if sinus in zerro-point
		if ( 0==(sinus180[tetha]) ) {
			if ( 1==PWM_StartUP ) {
				if ( PWM_Ramp < STEPS ) {
					if ( PWM_Ramp_TMP < 10 ) {
						PWM_Ramp_TMP++;
					} else {
						PWM_Ramp_TMP=0;
						PWM_Ramp++;
					}
				}
			}
		}
	}

	if ( tetha<(STEPS/2) ) {
		pwm1_duty_val = (PWM_Ramp * pwm1_duty)/STEPS;  // Ramp
		PWM_setCmpA(myPwm1, 0x00 );
		if ( tetha < 22 ) {
			PWM_setCmpB(myPwm1, 1+pwm1_duty_val ); // ok !!!
		}else{
			PWM_setCmpB(myPwm1, pwm1_duty_val ); // ok !!!
		}
	} else {
		pwm1_duty_val = (PWM_Ramp * (0x80-pwm1_duty) )/STEPS;  // Ramp
		PWM_setCmpA(myPwm1, pwm1_duty_val );
		PWM_setCmpB(myPwm1, 0x00 );
		/*if ( cnt == (STEPS)-1 ) {
			PWM_setCmpA(myPwm1, 0x80-pwm1_duty_val );
		}*/
	}
	tetha += 1;
	if ( tetha >= STEPS ) tetha=0;
#endif //0

#if 0
	if ( tetha >= STEPS ) tetha=0;

	pwm1_duty = EPWM1_MIN_DB+ (sinus45[tetha]/2);
	if ( tetha<STEPS/2 )
	//if ( (cnt>=22/*STEPS*/) && (cnt<67/*STEPS*/) )
	{
		PWM_setCmpA(myPwm1, /*0x300*/0 );
		//PWM_setCmpB(myPwm1, 1*(val-0x400) );
		PWM_setCmpB(myPwm1, (pwm1_duty) );
		/*if (cnt == ((STEPS/2)-1)) {
			PWM_setCmpA(myPwm1, 0 );
			PWM_setCmpB(myPwm1, 0 );
		}*/
	}
	else
	//if ( (cnt>=67/*STEPS*/) && (cnt<22/*STEPS*/) )
	{
		//PWM_setCmpA(myPwm1, 1*(0x400-val) );
		PWM_setCmpA(myPwm1, (0x400-pwm1_duty) );
		PWM_setCmpB(myPwm1, /*0x300*/0x0 );
		/*if (cnt == ((STEPS)-1)) {
			PWM_setCmpA(myPwm1, 0 );
			PWM_setCmpB(myPwm1, 0 );
		}*/
	}
	tetha += 1;
#endif //0

    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm1);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);

    //GPIO_setPortData (myGpio, GPIO_Port_A, EPwm1TimerIntCount & 0x1 ? 0xE : 0xF);
#endif //(1==PWM1_INT_ENABLE)
#endif //(1==USE_F28027_PWM)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - epwm2_timer_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void epwm2_timer_isr (void) {
#if (1==USE_F28027_PWM)
#if (1==PWM2_INT_ENABLE)
	static uint16_t cnt=0;

	//SinusRMS += sinus[cnt];
	//PWM_setPeriod(myPwm1, sinus[cnt] );
	PWM_setCmpA(myPwm2, /*EPWM2_MIN_DB+/ (1*sinus90[cnt]) );
	PWM_setCmpB(myPwm2, /*PWM2_FREQ_PERIOD-*/ /*EPWM2_MIN_DB+*/ (1*sinus90[cnt]) );

	if (++cnt >= STEPS ) cnt=0;

	// Clear INT flag for this timer
    PWM_clearIntFlag(myPwm2);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);
#endif //(1==PWM2_INT_ENABLE)
#endif //(1==USE_F28027_PWM)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - epwm3_timer_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void epwm3_timer_isr (void) {
#if (1==USE_F28027_PWM)
#if (1==PWM3_INT_ENABLE)
    EPwm3TimerIntCount += 10;
    //PWM_setPeriod (myPwm3, (uint16_t)EPwm3TimerIntCount);
/*
    //-------------------
    CLK_enablePwmClock   (myClk, PWM_Number_3);
    //PWM_setSyncMode      (myPwm3, PWM_SyncMode_EPWMxSYNC);    // Setup Sync
    PWM_setSyncMode      (myPwm3, PWM_SyncMode_CounterEqualZero); // Setup Sync
    PWM_enableCounterLoad(myPwm3);                              // Allow each timer to be sync'ed
    PWM_setPhase         (myPwm3, 0);
    PWM_setPeriod        (myPwm3, 0xffff);
    //PWM_setChoppingDutyCycle
    PWM_setCounterMode   (myPwm3, PWM_CounterMode_Up);          // Count up
    PWM_setIntMode       (myPwm3, PWM_IntMode_CounterEqualZeroOrPeriod);// Enable INT on Zero event
    PWM_enableInt        (myPwm3);                              // Enable INT
    PWM_setIntPeriod     (myPwm3, PWM_IntPeriod_ThirdEvent);    // Generate INT on 3rd event
    //-------------------

    //PWM_setActionQual_CntUp_CmpB_PwmB(myPwm3, PWM_ActionQual_Toggle);
    PWM_setActionQual_Period_PwmB(myPwm3, PWM_ActionQual_Toggle);
*/
    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm3);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);

    //GPIO_setPortData (myGpio, GPIO_Port_A, ++i_pwm3 & 0x1 ? 0xE : 0xF );
    //if ( ++i_pwm3 & 0x1)  GpioDataRegs.GPADAT.bit.GPIO0 = 1;
    //else                  GpioDataRegs.GPADAT.bit.GPIO0 = 0;
#endif //(1==PWM3_INT_ENABLE)
#endif //(1==USE_F28027_PWM)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - timer0_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
/*
interrupt void timer0_isr (void) {
#if (1==USE_F28027_TIMER)
    Timer0IntCount++;

    // Clear INT flag for this timer
    TIMER_clearFlag(myTimer);

    // Acknowledge this interrupt to receive more interrupts from group 3
    //TIMER_clearIntFlag(myTimer);
#endif //(1==USE_F28027_TIMER)
}
*/
/* ========================================================================== */



//* ========================================================================== */
interrupt void  adc_isr(void)
{
	uint16_t  cnt=0, nn=0;
	uint32_t  tmp=0;
	static uint16_t  yy=0;

	switch (stat_adc)
	{
		case ADC_INIT:
		    //adc->ADCRESULT[0];
		    //memcpy(adc_data, adc->ADCRESULT, 16);
			for (cnt=0; cnt<16; cnt++) {
				//adc_data[cnt]=adc->ADCRESULT[cnt];
				adc_offset[cnt]= ADC_readResult( (ADC_Handle)myAdc, cnt );
			}
			stat_adc = ADC_READY;
		break;

		case ADC_READY:
			//for (cnt=0; cnt<16; cnt++) {
			//	//adc_data[cnt]=adc->ADCRESULT[cnt];
			//	adc_data[cnt] = ADC_readResult( myAdc, (cnt+ADC_ResultNumber_0) );
			//}
	//	    //TempSensorVoltage[ConvCount] = ADC_readResult( myAdc, ADC_ResultNumber_0 );
	//	    tmp = adc_data[0];
	//	    TempSensorVoltage[0] = tmp;
	//		// If 20 conversions have been logged, start over
	//	    if ( ConvCount >= 9 )
	//	    	ConvCount = 0;
	//	    else
	//	    	ConvCount++;

	    	//Lcd_pixel( x, adc_data[0]/100, PIXEL_XOR );
	    	//yy++;
	    	if ( ++yy >= ADC_NUMS ) {
	    		update_data_from_adc = 1;
	    		yy=0;
	    	}
	    	//adc_data_array[yy] = adc_data[0] /*-adc_offset[0]*/;
	    	adc_data_array[yy] = ADC_readResult( myAdc, (ADC_ResultNumber_0) );
        	tmp = ADC_readResult( myAdc, (ADC_ResultNumber_14) );
        	//adc_on_VarResistor >>= 4;

        	if ( nn++ >= 16 ) {
        		tmp++;
        	} else {
        		adc_on_VarResistor = tmp/16;
        		nn = 0;
        		tmp = 0;
        	}

		break;
	}

    ADC_clearIntFlag(myAdc, ADC_IntNumber_1);
    PIE_clearInt(myPie, PIE_GroupNumber_10);

    return;
}
/* ========================================================================== */



/* ========================================================================== */
__interrupt void cpu_timer0_isr(void)
{
	//interruptCount++;

	if (sys_stat.sys.error != E_OK)
	{
		// Toggle GPIOs
		GPIO_toggle(myGpio, GPIO_Number_0); // 0=LED_ON, 1=LED_OFF
		GPIO_toggle(myGpio, GPIO_Number_1); // 0=LED_ON, 1=LED_OFF
		GPIO_toggle(myGpio, GPIO_Number_2); // 0=LED_ON, 1=LED_OFF
		GPIO_toggle(myGpio, GPIO_Number_3); // 0=LED_ON, 1=LED_OFF
	}

	// Acknowledge this interrupt to receive more interrupts from group 1
    PIE_clearInt(myPie, PIE_GroupNumber_1);
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - sciaTxFifoIsr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void sciaTxFifoIsr (void) {
#if (1==USE_F28027_SCI)
    switch (sys_stat.sci.tx){
    	case SCI_TX_READY:
    	break;

    	case SCI_TX_SENDING:
	        SCI_enableRxInt(mySci);
	        SCI_enableTxInt(mySci);
    		if (i_tx < RX_LEN) {
    	        SciaRegs.SCITXBUF=rdataA[i_tx];     // Send data
    	        //SciaRegs.SCITXBUF=RxTx;              // Send data
        		//SciaRegs.SCICTL2.bit.TXINTENA=1;
        		SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
        		PieCtrlRegs.PIEACK.all |= 0x100;    // Issue PIE ACK
    			sys_stat.sci.tx = SCI_TX_SENDING;
    			i_tx++;
    		    //EALLOW;	// This is needed to write to EALLOW protected registers
    			//DINT;
    			//IER = 0x100;	// Enable CPU INT
    			//IFR = 0x100;
    			//EINT;
 		        //EDIS;   // This is needed to disable write to EALLOW protected registers
    		} else {
    			sys_stat.sci.tx = SCI_TX_FINISH;
    			i_tx=0;

        		SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;	      // Clear SCI Interrupt flag
        		PieCtrlRegs.PIEACK.all |= 0x100;          // Issue PIE ACK
        	    //SCI_clearTxFifoInt(mySci);              // Clear SCI Interrupt flag
        	    //PIE_clearInt(myPie, PIE_GroupNumber_9); // Issue PIE ACK
        	    SCI_disableTxInt(mySci);
         		}
    	break;

    	case SCI_TX_FINISH:
    	break;
    }
#endif //(1==USE_F28027_SCI)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - sciaRxFifoIsr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void sciaRxFifoIsr (void) {
#if (1==USE_F28027_SCI)
    static Uint16 i=0;

	if (SciaRegs.SCIRXBUF.bit.RXDT=='_') {
		i=0;
		sys_stat.sci.tx = SCI_TX_READY;
	}

    switch (sys_stat.sci.tx){
    	case SCI_TX_READY:
     	    if ( i < RX_LEN ) {
    	    	rdataA[i++]=SciaRegs.SCIRXBUF.all;	 // Read data

    	    	SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    	    	SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    	    	PieCtrlRegs.PIEACK.all |= 0x100;     // Issue PIE ack
    	    } else {
    	    	i=0;

    			sys_stat.sci.tx = SCI_TX_SENDING;
    			i_tx=0;
    	        SCI_enableTxInt(mySci);
    	    	RxTx='>';
    	        SciaRegs.SCITXBUF=RxTx;              // Send data

        		SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
        		PieCtrlRegs.PIEACK.all |= 0x100;     // Issue PIE ACK
    	    }
    	break;

    	case SCI_TX_SENDING:
    	break;

    	case SCI_TX_FINISH:
    	break;
    }
#endif //(1==USE_F28027_SCI)
}
/* ========================================================================== */



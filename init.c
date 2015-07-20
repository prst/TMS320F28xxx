
#include "stddef.h"
#include "stdlib.h"

#include "./include/F2802x_Device.h"
#include "./include/DSP28x_Project.h"
#include "./include/F2802x_Examples.h"
#include "./include/F2802x_GlobalPrototypes.h"

#include "types.h"
#include "init.h"
#include "interrupts.h"


/* ========================================================================== */
/* ========================================================================== */


CPU_Handle   myCpu;
PLL_Handle   myPll;
CLK_Handle   myClk;
FLASH_Handle myFlash;
ADC_Handle   myAdc;
SCI_Handle   mySci;
GPIO_Handle  myGpio;
PIE_Handle   myPie;
PWM_Handle   myPwm1, myPwm2, myPwm3;
TIMER_Handle myTimer;
WDOG_Handle  myWDog;

/*
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
*/

/* ========================================================================== */
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
#define TIME_PERIOD_MIN    30000 //(0x2FFF) // TEST

#define TMR0__TIME_PERIOD  (TIME_PERIOD_MIN)
/* ========================================================================== */
// Configure which ePWM timer interrupts are enabled at the PIE level:
// 1 = enabled,  0 = disabled
#define  PWM1_INT_ENABLE  1
#define  PWM2_INT_ENABLE  0
#define  PWM3_INT_ENABLE  0

// Configure the period for each timer
#define PWM1_TIMER_TBPRD  0xFFFF
//#define PWM1_TIMER_TBPRD  0x1FFF

#define PWM2_TIMER_TBPRD  0x1FFF

//#define PWM3_TIMER_TBPRD  0x1FFF
#define PWM3_TIMER_TBPRD  0xFFFF

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

#define LSPCLK_FREQ       (CPU_FREQ/4)

//#define SCI_FREQ          100E3
#define SCI_FREQ          115200
//#define SCI_PRD           (LSPCLK_FREQ/(SCI_FREQ*8))-1
#define SCI_BRR           (LSPCLK_FREQ/(SCI_FREQ*8))-1
/* ========================================================================== */

//#define  USE_UART_IRQ_1   (0)
//#define  USE_UART_IRQ_2   (1)


/* ========================================================================== */
t_error      rc = E_OK;
t_status     sys_stat;

/* ========================================================================== */



/* ========================================================================== */
void InitGpio_Conf_HW (void) {
	EALLOW;
	GpioCtrlRegs.GPAMUX1.all |= gpio_mux;
	GpioCtrlRegs.GPADIR.all |= gpio_dir;
	//	GpioDataRegs.GPASET.all |= 0xff;
	EDIS;
}
/* ========================================================================== */



/* ========================================================================== */
void* memset(void *mem, register int ch, register size_t length)
{
     register char *m = (char *)mem;

     while (length--) *m++ = ch;
     return mem;
}
/* ========================================================================== */


/* ========================================================================== */
// This function initializes the Flash Control registers
/* ========================================================================== */
//                   CAUTION
// This function MUST be executed out of RAM. Executing it
// out of OTP/Flash will yield unpredictable results
void InitFlash(void)
{
   EALLOW;
   //Enable Flash Pipeline mode to improve performance
   //of code executed from Flash.
   FlashRegs.FOPT.bit.ENPIPE = 1;

   //                CAUTION
   //Minimum waitstates required for the flash operating
   //at a given CPU rate must be characterized by TI.
   //Refer to the datasheet for the latest information.

   //Set the Paged Waitstate for the Flash
   FlashRegs.FBANKWAIT.bit.PAGEWAIT = 3;

   //Set the Random Waitstate for the Flash
   FlashRegs.FBANKWAIT.bit.RANDWAIT = 3;

   //Set the Waitstate for the OTP
   FlashRegs.FOTPWAIT.bit.OTPWAIT = 5;

   //                CAUTION
   //ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED
   FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
   FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;
   EDIS;

   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.

   asm(" RPT #7 || NOP");
}
/* ========================================================================== */



/* ========================================================================== */
// This function will copy the specified memory contents from
// one location to another.
//
//	Uint16 *SourceAddr        Pointer to the first word to be moved
//                          SourceAddr < SourceEndAddr
//	Uint16* SourceEndAddr     Pointer to the last word to be moved
//	Uint16* DestAddr          Pointer to the first destination word
//
// No checks are made for invalid memory locations or that the
// end address is > then the first start address.
/* ========================================================================== */
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_All
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void Init_All ( void )
{
	if (E_OK==rc)  rc = Init_Sys();  else  Error(rc); // Init system and handles
    if (E_OK==rc)  rc = Init_PLL();  else  Error(rc); // Init PLL
    if (E_OK==rc)  rc = Init_ADC();  else  Error(rc); // Init ADC
    if (E_OK==rc)  rc = Init_PWM();  else  Error(rc); // Init IRQs
    if (E_OK==rc)  rc = Init_FLASH(); else  Error(rc); // Init FLASH
    if (E_OK==rc)  rc = Init_GPIO();  else  Error(rc); // Init GPIO system
    if (E_OK==rc)  rc = Init_Timer0(); else  Error(rc); // Init Timer0

    /*
    if (E_OK==rc)  rc = Init_UART_IRQ(); // Init UART IRQ
    	//err = Init_UART_pooling (); // Init UART without IRQ
    else  Error(rc);

    if (E_OK==rc)  rc = Init_TM1638();  // Init TM1638
    else  Error(rc);
    */

    if (E_OK==rc) {
    	sys_stat.sys.error = E_OK;
    }
}


/* ==========================================================================
 * NAME - Error
 * IN   - t_error err
 * OUT  - void
 * RET  - void
   ========================================================================== */
void Error (t_error err) {
	switch (err) {
		case E_OK:
			sys_stat.sys.error = E_OK;
			break;

		case E_FAIL:
		case E_BADPTR:
			sys_stat.sys.error = E_FAIL;
			break;

		default:
			sys_stat.sys.error = E_FAIL;
			error();
			break;
	}
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_Sys
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_Sys (void) {

	// PERIPHERAL CLOCK ENABLES
	//---------------------------------------------------
	// If you are not using a peripheral you may want to switch
	// the clock off to save power, i.e. set to =0
	// Note: not all peripherals are available on all 280x derivates.
	// Refer to the datasheet for your particular device.
	//------------------------------------------------
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK   = 0;  // ADC
	SysCtrlRegs.PCLKCR3.bit.COMP1ENCLK = 0;	 // COMP1
	SysCtrlRegs.PCLKCR3.bit.COMP2ENCLK = 0;	 // COMP2
	SysCtrlRegs.PCLKCR0.bit.I2CAENCLK  = 1;  // I2C
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK  = 0;	 // SPI-A
	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK  = 0;  // SCI-A
	SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 0;	 // eCAP1
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // ePWM1
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // ePWM2
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // ePWM3
	SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;  // ePWM4
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC  = 1;  // Enable TBCLK
	//------------------------------------------------


    // Initialize all the handles needed for this application
#if (1==USE_F28027_CLK)
    myClk  = CLK_init  ((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
#endif //(1==USE_F28027_CLK)

#if (1==USE_F28027_CPU)
    myCpu  = CPU_init  ((void *)NULL, sizeof(CPU_Obj));
#endif //(1==USE_F28027_CPU)

#if (1==USE_F28027_FLASH)
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
#endif //(1==USE_F28027_FLASH)

#if (1==USE_F28027_GPIO)
    myGpio = GPIO_init ((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
#endif //(1==USE_F28027_GPIO)

#if (1==USE_F28027_PIE)
    myPie  = PIE_init  ((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
#endif //(1==USE_F28027_PIE)

#if (1==USE_F28027_PLL)
    myPll  = PLL_init  ((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
#endif //(1==USE_F28027_PLL)

#if (1==USE_F28027_PWM)
	#if (1==PWM1_INT_ENABLE)
    myPwm1 = PWM_init  ((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
	#endif //(1==PWM1_INT_ENABLE)

	#if (1==PWM2_INT_ENABLE)
    myPwm2 = PWM_init  ((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
	#endif //(1==PWM2_INT_ENABLE)

	#if (1==PWM2_INT_ENABLE)
    myPwm3 = PWM_init  ((void *)PWM_ePWM3_BASE_ADDR, sizeof(PWM_Obj));
	#endif //(1==PWM2_INT_ENABLE)
#endif //(1==USE_F28027_PWM)

#if (1==USE_F28027_WDOG)
    myWDog = WDOG_init ((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
#endif //(1==USE_F28027_WDOG)

#if (1==USE_F28027_TIMER)
    myTimer = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
#endif //(1==USE_F28027_TIMER)

#if (1==USE_F28027_ADC)
    myAdc = ADC_init  ((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
#endif //(1==USE_F28027_ADC)

#if (1==USE_F28027_SCI)
    mySci   = SCI_init  ((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));
#endif //(1==USE_F28027_SCI)

    return (t_error) E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_PWM
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_PWM (void) {

    CLK_enablePwmClock(myClk, PWM_Number_1);

    PWM_enableSocAPulse(myPwm1);
    PWM_setSocAPulseSrc(myPwm1, PWM_SocPulseSrc_CounterEqualCmpAIncr);
    PWM_setSocAPeriod(myPwm1, PWM_SocPeriod_FirstEvent);
    ((PWM_Obj *)myPwm1)->CMPA = 0x0080;
    //PWM_setPeriod(myPwm1, 0xFFFF);
    PWM_setPeriod(myPwm1, 0x00FF);
    PWM_setCounterMode(myPwm1, PWM_CounterMode_Up);

    CLK_enableTbClockSync(myClk);

/*
#if (1==USE_F28027_PWM)
    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable (myPie);
    PIE_enable (myPie);

    // Register interrupt handlers in the PIE vector table

#if (1==PWM1_INT_ENABLE)
    PIE_registerPieIntHandler (myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_1,
                                 (intVec_t)&epwm1_timer_isr );
#endif //(1==PWM1_INT_ENABLE)

#if (1==PWM2_INT_ENABLE)
    PIE_registerPieIntHandler (myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_2,
                                 (intVec_t)&epwm2_timer_isr );
#endif //(1==PWM2_INT_ENABLE)

#if (1==PWM3_INT_ENABLE)
    PIE_registerPieIntHandler (myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_3,
                                 (intVec_t)&epwm3_timer_isr );
#endif //(1==PWM3_INT_ENABLE)

    // For this example, only initialize the ePWM Timers
    init_Cfg_EPwmTimers ();

	{
		// TODO ??? GROUP ???
		//PIE_registerPieIntHandler ( myTimer,
		                              PIE_GroupNumber_3,
		                              PIE_SubGroupNumber_3,
		                              (intVec_t)&timer0_isr );

		// Enable CPU INT3 which is connected to EPWM1-6 INT
		//Timer0IntCount     = 0;
		//CPU_enableInt (myCpu, CPU_IntNumber_3);
	}

    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
#if (1==PWM1_INT_ENABLE)
    EPwm1TimerIntCount = 0;    // Initalize counters:
    PIE_enablePwmInt (myPie, PWM_Number_1);
#endif //(1==PWM1_INT_ENABLE)

#if (1==PWM2_INT_ENABLE)
    EPwm2TimerIntCount = 0;    // Initalize counters:
    PIE_enablePwmInt (myPie, PWM_Number_2);
#endif //(1==PWM2_INT_ENABLE)

#if (1==PWM3_INT_ENABLE)
    EPwm3TimerIntCount = 0;    // Initalize counters:
    PIE_enablePwmInt (myPie, PWM_Number_3);
#endif //(1==PWM3_INT_ENABLE)

    //PIE_enableTimer0Int (myTimer);

    // Enable global Interrupts and higher priority real-time debug events
    CPU_enableGlobalInts (myCpu);
    CPU_enableDebugInt (myCpu);

#endif //(1==USE_F28027_PWM)
*/

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_GPIO
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_GPIO (void) {
#if (1==USE_F28027_GPIO)
    // Initalize GPIO

	// Version 1
    /*GPIO_setDirection (myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_1, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_2, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_3, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_4, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_5, GPIO_Direction_Output);
    GPIO_setPortData  (myGpio, GPIO_Port_A, 0x003F); */

    // Version 2
    GPIO_setDirection (myGpio, GPIO_Number_7,  GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_6,  GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_17, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_16, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_12, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_19, GPIO_Direction_Output);
    GPIO_setPortData  (myGpio, GPIO_Port_A,
    		                  ( 1<<7 | 1<<6 | 1<<17 | 1<<16 | 1<<12 | 1<<19 ) );

    GPIO_setDirection (myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_1, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_2, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_3, GPIO_Direction_Output);

    //GPIO_setPortData (myGpio, GPIO_Port_A, 0xF ); // 0=LED_ON, 1=LED_OFF
    GpioDataRegs.GPADAT.bit.GPIO0 = 0;  // 0=LED_ON, 1=LED_OFF
    GpioDataRegs.GPADAT.bit.GPIO1 = 0;  // 0=LED_ON, 1=LED_OFF
    GpioDataRegs.GPADAT.bit.GPIO2 = 0;  // 0=LED_ON, 1=LED_OFF
    GpioDataRegs.GPADAT.bit.GPIO3 = 0;  // 0=LED_ON, 1=LED_OFF


    //GPIO_setMode(myGpio, GPIO_Number_5, GPIO_5_Mode_EPWM3B);

	//InitGpio_Conf_HW();
#endif //(1==USE_F28027_GPIO)
    return E_OK;
}
/* ========================================================================== */


/* ==========================================================================
 * NAME - Init_Timer0
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_Timer0 (void) {
#if (1==USE_F28027_TIMER)

    // Register interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler ( myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_7,
    		                    (intVec_t)&cpu_timer0_isr );

    // Configure CPU-Timer 0 to interrupt every 500 milliseconds:
    // 60MHz CPU Freq, 50 millisecond Period (in uSeconds)
    //    ConfigCpuTimer(&CpuTimer0, 60, 500000);
    TIMER_stop(myTimer);
    //TIMER_setPeriod(myTimer, 50 * 500000);
    TIMER_setPeriod(myTimer, (uint32_t)TMR0__TIME_PERIOD);
    TIMER_setPreScaler(myTimer, 0);
    TIMER_reload(myTimer);
    TIMER_setEmulationMode(myTimer, TIMER_EmulationMode_StopAfterNextDecrement);
    TIMER_enableInt(myTimer);

    TIMER_start(myTimer);

    // Enable CPU INT1 which is connected to CPU-Timer 0:
    CPU_enableInt(myCpu, CPU_IntNumber_1);

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PIE_enableTimer0Int(myPie);

    // Enable global Interrupts and higher priority real-time debug events
    CPU_enableGlobalInts(myCpu);
    CPU_enableDebugInt(myCpu);

#endif //(1==USE_F28027_TIMER)
    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================

    return E_OK;
 * NAME - init_Cfg_EPwmTimers
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void init_Cfg_EPwmTimers (void) {
#if (1==USE_F28027_PWM)
    // Stop all the TB clocks
    CLK_disableTbClockSync(myClk);

#if (1==PWM1_INT_ENABLE)
    CLK_enablePwmClock   (myClk, PWM_Number_1);
    PWM_setSyncMode      (myPwm1, PWM_SyncMode_EPWMxSYNC);    // Setup Sync
    PWM_enableCounterLoad(myPwm1);     // Allow each timer to be sync'ed
    PWM_setPhase         (myPwm1, 100);
    PWM_setPeriod        (myPwm1, PWM1_TIMER_TBPRD);
    PWM_setCounterMode   (myPwm1, PWM_CounterMode_Up);        // Count up
    PWM_setIntMode       (myPwm1, PWM_IntMode_CounterEqualZero);// Select INT on Zero event
    PWM_enableInt        (myPwm1);                            // Enable INT
    PWM_setIntPeriod     (myPwm1, PWM_IntPeriod_FirstEvent);  // Generate INT on 1st event
#endif //(1==PWM1_INT_ENABLE)

#if (1==PWM2_INT_ENABLE)
    CLK_enablePwmClock   (myClk, PWM_Number_2);
    PWM_setSyncMode      (myPwm2, PWM_SyncMode_EPWMxSYNC);    // Setup Sync
    PWM_enableCounterLoad(myPwm2);     // Allow each timer to be sync'ed
    PWM_setPhase         (myPwm2, 200);
    PWM_setPeriod        (myPwm2, PWM2_TIMER_TBPRD);
    PWM_setCounterMode   (myPwm2, PWM_CounterMode_Up);        // Count up
    PWM_setIntMode       (myPwm2, PWM_IntMode_CounterEqualZero);// Enable INT on Zero event
    PWM_enableInt        (myPwm2);                            // Enable INT
    PWM_setIntPeriod     (myPwm2, PWM_IntPeriod_SecondEvent); // Generate INT on 2nd event
#endif //(1==PWM2_INT_ENABLE)

#if (1==PWM3_INT_ENABLE)
/*
    CLK_enablePwmClock   (myClk, PWM_Number_3);
    PWM_setSyncMode      (myPwm3, PWM_SyncMode_EPWMxSYNC);    // Setup Sync
    //PWM_setSyncMode      (myPwm3, PWM_SyncMode_CounterEqualZero); // Setup Sync
    PWM_enableCounterLoad(myPwm3);    // Allow each timer to be sync'ed
    PWM_setPhase         (myPwm3, 300);
    //PWM_setPhase         (myPwm3, 0);
    //PWM_setPeriod        (myPwm3, PWM3_TIMER_TBPRD);
    PWM_setPeriod        (myPwm3, 0x0fff);
    PWM_setCounterMode   (myPwm3, PWM_CounterMode_Up);        // Count up
    PWM_setIntMode       (myPwm3, PWM_IntMode_CounterEqualZero);// Enable INT on Zero event
    PWM_enableInt        (myPwm3);                            // Enable INT
    PWM_setIntPeriod     (myPwm3, PWM_IntPeriod_ThirdEvent);  // Generate INT on 3rd event
*/
    //=====================================================================
    // (Note: code for only 3 modules shown)
    // Initialization Time
    //========================
    /*
    // EPWM Module 1 config
    EPwm1Regs.TBPRD = 1200; // Period = 1201 TBCLK counts
    EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Asymmetrical mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.AQCTLA.bit.PRD = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
    // EPWM Module 2 config
    EPwm2Regs.TBPRD = 1400; // Period = 1401 TBCLK counts
    EPwm2Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Asymmetrical mode
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm2Regs.AQCTLA.bit.PRD = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
    */
    CLK_enablePwmClock   (myClk, PWM_Number_2);
    // EPWM Module 3 config
    EPwm3Regs.TBPRD = 800; // Period = 801 TBCLK counts
    EPwm3Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm3Regs.AQCTLB.bit.PRD = AQ_TOGGLE; //AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_TOGGLE; //AQ_SET;
    // Run Time (Note: Example execution of one run-time instant)
    //=========================================================
    //EPwm1Regs.CMPA.half.CMPA = 700; // adjust duty for output EPWM1A
    //EPwm2Regs.CMPA.half.CMPA = 700; // adjust duty for output EPWM2A
    EPwm3Regs.CMPB = 500; // adjust duty for output EPWM3A

    PWM_setIntMode   (myPwm3, PWM_IntMode_CounterEqualZero);// Enable INT on Zero event
    PWM_enableInt    (myPwm3);                              // Enable INT
    PWM_setIntPeriod (myPwm3, PWM_IntPeriod_SecondEvent);   // Generate INT on 2nd event
#endif //(1==PWM3_INT_ENABLE)

    // Start all the timers synced
    CLK_enableTbClockSync(myClk);
#endif //(1==USE_F28027_PWM)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - error
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void error(void) {
    asm(" ESTOP0"); // Test failed!! Stop!
    for (;;){ }
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_UART_IRQ
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_UART_IRQ (void) {

#if (1==USE_F28027_SCI)
	// Initalize GPIO
	GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
	GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
	GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);
	GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
	GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);

    // Step ... Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
       DINT;

    // Initialize PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2802x_PieCtrl.c file.
       InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
       IER = 0x0000;
       IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2802x_DefaultIsr.c.
    // This function is found in DSP2802x_PieVect.c.
       InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
       EALLOW;	// This is needed to write to EALLOW protected registers
       PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
       PieVectTable.SCITXINTA = &sciaTxFifoIsr;
       EDIS;   // This is needed to disable write to EALLOW protected registers

    // Step ... Initialize all the Device Peripherals:
    // This function is found in DSP2802x_InitPeripherals.c
    // InitPeripherals(); // Not required for this example
       scia_fifo_init();  // Init SCI-A

    // Step ... User specific code, enable interrupts:

    // Init send data.  After each transmission this data
    // will be updated for the next transmission
       /*for(i = 0; i<2; i++) {  sdataA[i] = i;  }
       rdata_pointA = sdataA[0];*/

    // Enable interrupts required for this example
       PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
       PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
       PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2
       IER = 0x100;	// Enable CPU INT
       EINT;
#endif //(1==USE_F28027_SCI)

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_TM1638
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_TM1638 (void) {

#if (1==__USE_TM1638__)
	tm1638_init();
#endif //(1==__USE_TM1638__)

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_PLL
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_PLL (void) {

#if (1==USE_F28027_PLL)

    // Perform basic system initialization
    WDOG_disable (myWDog);

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc (myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup (myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);

    // Disable the PIE and all interrupts
    PIE_disable (myPie);
    PIE_disableAllInts (myPie);
    CPU_disableGlobalInts (myCpu);
    CPU_clearIntFlags (myCpu);

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);

#endif //(1==USE_F28027_PLL)

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_FLASH
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_FLASH (void) {

#if (1==USE_F28027_FLASH)

#ifdef _FLASH
    /*// If running from flash copy RAM only functions to RAM
#ifdef _FLASH
    memcpy (&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif */

	// Copy time critical code and Flash setup code to RAM
	// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files.
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	//MemCopy(&IQfuncsLoadStart, &IQfuncsLoadEnd, &IQfuncsRunStart);

	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function

    /*
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);
    */

#endif //(FLASH)

#endif //(1==USE_F28027_FLASH)

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Init_ADC
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error Init_ADC (void) {

#if (1==USE_F28027_ADC)
	uint16_t cnt;

    //CLK_enableAdcClock (myClk);
    //(*Device_cal)();

    //CLK_disableAdcClock (myClk);

	//adc_init();
	{
	//Initialize the ADC:
	CLK_enableAdcClock(myClk);
	(*Device_cal)();

	// Setup a debug vector table and enable the PIE
	PIE_setDebugIntVectorTable (myPie);
	PIE_enable (myPie);

	// Register interrupt handlers in the PIE vector table
	PIE_registerPieIntHandler (
			myPie,
			PIE_GroupNumber_10,
			PIE_SubGroupNumber_1,
			(intVec_t)&adc_isr
			);

	//Initialize the ADC:
	ADC_enableBandGap(myAdc);
	ADC_enableRefBuffers(myAdc);
	ADC_powerUp(myAdc);
	ADC_enable(myAdc);
	ADC_setVoltRefSrc(myAdc, ADC_VoltageRefSrc_Int);

	ADC_enableTempSensor(myAdc);

	ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);

	ADC_enableInt         (myAdc, ADC_IntNumber_1);
	ADC_setIntMode        (myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);
	ADC_setIntSrc         (myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC1);

	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_0,  ADC_SocChanNumber_A0);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_1,  ADC_SocChanNumber_A1);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_2,  ADC_SocChanNumber_A2);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_3,  ADC_SocChanNumber_A3);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_4,  ADC_SocChanNumber_A4);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_5,  ADC_SocChanNumber_A5);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_6,  ADC_SocChanNumber_A6);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_7,  ADC_SocChanNumber_A7);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_8,  ADC_SocChanNumber_B0);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_9,  ADC_SocChanNumber_B1);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_10, ADC_SocChanNumber_B2);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_11, ADC_SocChanNumber_B3);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_12, ADC_SocChanNumber_B4);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_13, ADC_SocChanNumber_B5);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_14, ADC_SocChanNumber_B6);
	ADC_setSocChanNumber  (myAdc, ADC_SocNumber_15, ADC_SocChanNumber_B7);

	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_0,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_1,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_2,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_3,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_4,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_5,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_6,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_7,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_8,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_9,  ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_10, ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_11, ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_12, ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_13, ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_14, ADC_SocTrigSrc_EPWM1_ADCSOCA);
	ADC_setSocTrigSrc     (myAdc, ADC_SocNumber_15, ADC_SocTrigSrc_EPWM1_ADCSOCA);

	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_3,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_4,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_5,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_6,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_7,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_8,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_9,  ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_10, ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_11, ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_12, ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_13, ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_14, ADC_SocSampleWindow_37_cycles);
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_15, ADC_SocSampleWindow_37_cycles);

	PIE_enableAdcInt (myPie, ADC_IntNumber_1); // Enable ADCINT1 in PIE
	CPU_enableInt (myCpu, CPU_IntNumber_10);   // Enable CPU Interrupt 1
	CPU_enableGlobalInts (myCpu);              // Enable Global interrupt INTM
	CPU_enableDebugInt (myCpu);                // Enable Global realtime interrupt DBGM

/*
	// Configure ADC
	//Note: Channel ADCINA4  will be double sampled to workaround the ADC 1st sample issue for rev0 silicon errata
	//ADCINT1 trips after AdcResults latch
	ADC_setIntPulseGenMode(myAdc, ADC_IntPulseGenMode_Prior);

	//Enabled ADCINT1
	ADC_enableInt(myAdc, ADC_IntNumber_1);

	//Disable ADCINT1 Continuous mode
	ADC_setIntMode(myAdc, ADC_IntNumber_1, ADC_IntMode_ClearFlag);

	//setup EOC2 to trigger ADCINT1 to fire
	ADC_setIntSrc(myAdc, ADC_IntNumber_1, ADC_IntSrc_EOC2);

	//set SOC0 channel select to ADCINA4
	ADC_setSocChanNumber (myAdc, ADC_SocNumber_0, ADC_SocChanNumber_A4);

	//set SOC1 channel select to ADCINA4
	ADC_setSocChanNumber (myAdc, ADC_SocNumber_1, ADC_SocChanNumber_A4);

	//set SOC2 channel select to ADCINA2
	ADC_setSocChanNumber (myAdc, ADC_SocNumber_2, ADC_SocChanNumber_A2);

	//set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	ADC_setSocTrigSrc(myAdc, ADC_SocNumber_0, ADC_SocTrigSrc_EPWM1_ADCSOCA);

	//set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
	ADC_setSocTrigSrc(myAdc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM1_ADCSOCA);

	//set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1, then SOC2
	ADC_setSocTrigSrc(myAdc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM1_ADCSOCA);

	//set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_0, ADC_SocSampleWindow_7_cycles);

	//set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_1, ADC_SocSampleWindow_7_cycles);

	//set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	ADC_setSocSampleWindow(myAdc, ADC_SocNumber_2, ADC_SocSampleWindow_7_cycles);
*/
 	}
#endif //(1==USE_F28027_ADC)

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - scia_fifo_init
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void scia_fifo_init (void) {
#if (1==USE_F28027_SCI)

/*	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK  = 1;  // SCI-A
	SciaRegs.SCICCR.all =0x0007;  // 1-Stop bit, No loopback, No parity,
                                  // 8-char bits, async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003; // enable TX, RX, internal SCICLK,
                                  // Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.bit.TXINTENA   = 1;
	SciaRegs.SCICTL2.bit.RXBKINTENA = 1;
	SciaRegs.SCIHBAUD = 0x0000;
	SciaRegs.SCILBAUD = 15; //15=115200 //SCI_BAUD;
    //SciaRegs.SCICCR.bit.LOOPBKENA = 0; // Enable/Disable loop back
	SciaRegs.SCIFFTX.all = 0xC022;
	SciaRegs.SCIFFRX.all = 0x0022;
	SciaRegs.SCIFFCT.all = 0x00;
	SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset
	SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
	SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;  */

    CLK_enableSciaClock (myClk);

    // 1 stop bit, No loopback, No parity, 8 bits, Async-Mode, Idle-Line protocol
    SCI_disableParity(mySci);
    SCI_setNumStopBits(mySci, SCI_NumStopBits_One);
    SCI_setCharLength(mySci, SCI_CharLength_8_Bits);

    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    SCI_enableTx(mySci);
    SCI_enableRx(mySci);
    SCI_enableTxInt(mySci);
    SCI_enableRxInt(mySci);
    //SCI_enableLoopBack(mySci);
    SCI_setTxFifoIntLevel( mySci, SCI_FifoLevel_Empty );
    SCI_setRxFifoIntLevel( mySci, SCI_FifoLevel_1_Word );
    //SCI_disableFifoEnh(mySci);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
#if (CPU_FRQ_50MHZ)
    //SCI_setBaudRate(mySci, SCI_BaudRate_9_6_kBaud);
    SCI_setBaudRate(mySci, /*SCI_BaudRate_115_2_kBaud*/ (SCI_BaudRate_e)13);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)129);
#endif
    SCI_enable(mySci);

#endif //(1==USE_F28027_SCI)
}
/* ========================================================================== */






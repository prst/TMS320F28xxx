//#############################################################################
//  File:   f2802x_examples_ccsv4/epwm_timer_interrupts/Example_F2802xEPwmTimerInt.c
//  Title:  F2802x ePWM Timer Interrupt example.
//#############################################################################
//  Group:          C2000
//  Target Device:  TMS320F2802x
//#############################################################################
//! \addtogroup example_list
//!  <h1>PWM Timer Interrupt</h1>
//!
//!   This example configures the ePWM Timers and increments
//!   a counter each time an interrupt is taken.
//!
//!   As supplied:
//!   All ePWM's are initalized.
//!   All timers have the same period. \n
//!   The timers are started sync'ed. \n
//!   An interrupt is taken on a zero event for each ePWM timer.
//!      ePWM1: takes an interrupt every event \n
//!      ePWM2: takes an interrupt every 2nd event \n
//!      ePWM3: takes an interrupt every 3rd event 
//!
//!   Thus the Interrupt count for ePWM1 and ePWM4 should be equal.
//!   The interrupt count for ePWM2 should be about half that of ePWM1,
//!   and the interrupt count for ePWM3 should be about 1/3 that of ePWM1
//!   Watch Variables:
//!   - EPwm1TimerIntCount
//!   - EPwm2TimerIntCount
//!   - EPwm3TimerIntCount
//#############################################################################
//  (C) Copyright 2012, Texas Instruments, Inc.
//#############################################################################
// $TI Release: LaunchPad f2802x Support Library v100 $
// $Release Date: Wed Jul 25 10:45:39 CDT 2012 $
//#############################################################################

#include "./include/DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "./include/F2802x_Examples.h"
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


//#############################################################################

// Configure which ePWM timer interrupts are enabled at the PIE level:
// 1 = enabled,  0 = disabled
#define PWM1_INT_ENABLE   1
#define PWM2_INT_ENABLE   0
#define PWM3_INT_ENABLE   0

// Configure the period for each timer
#define PWM1_TIMER_TBPRD  0xFFFF
//#define PWM1_TIMER_TBPRD  0x1FFF
#define PWM2_TIMER_TBPRD  0x1FFF
#define PWM3_TIMER_TBPRD  0x1FFF

//#define CPU_FREQ          (40E6)   // Default = 40 MHz. Change to 50E6 for 50 MHz devices
#define CPU_FREQ          (50E6)   // Default = 40 MHz. Change to 50E6 for 50 MHz devices
#define LSPCLK_FREQ       CPU_FREQ/4

//#define CPU_FRQ_40MHZ
//#define CPU_FRQ_50MHZ
//#define CPU_FRQ_60MHZ

#define SCI_FREQ          100E3
#define SCI_PRD           (LSPCLK_FREQ/(SCI_FREQ*8))-1

typedef enum {
	E_OK,
	E_FAIL,
	E_BADPTR
} t_error;

// Prototype statements for functions found within this file.
interrupt void sciaTxFifoIsr(void);
interrupt void sciaRxFifoIsr(void);
//interrupt void scibTxFifoIsr(void);
//interrupt void scibRxFifoIsr(void);
void scia_init(void);
void scia_fifo_init(void);
void error(void);

// Prototype statements for functions found within this file.
interrupt void epwm1_timer_isr(void);
interrupt void epwm2_timer_isr(void);
interrupt void epwm3_timer_isr(void);
interrupt void timer0_isr(void);
void init_EPwmTimer(void);

t_error wrapper_Init_Sys (void);
t_error wrapper_Init_PWM_IRQs (void);
t_error wrapper_Init_GPIO (void);
void    wrapper_Main ( void );
void    wrapper_Error_Handle( t_error err );


// Global variables used in this example
uint32_t     EPwm1TimerIntCount;
uint32_t     EPwm2TimerIntCount;
uint32_t     EPwm3TimerIntCount;
uint32_t     Timer0IntCount;

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
t_error      err = E_OK;
int          i=0;
int          i_pwm0=0;

// Global variables
uint16_t     sdataA[2];     // Send data for SCI-A
uint16_t     rdataA[2];     // Received data for SCI-A
uint16_t     rdata_pointA;  // Used for checking the received data


/* ==========================================================================
 * MAIN
 * ========================================================================== */
void main(void) {
	if (E_OK==err) {
        err = wrapper_Init_Sys ();     	 // Init system and handles
    } else {
    	wrapper_Error_Handle (err);
    }
    
    /*if (E_OK==err) {
        err = wrapper_Init_PWM_IRQs ();  // Init IRQs
    } else {
    	wrapper_Error_Handle (err);
    }*/

    if (E_OK==err) {
    	err = wrapper_Init_GPIO ();   // GPIO system
    } else {
    	wrapper_Error_Handle (err);
    }

    if (E_OK==err) {
    	err = wrapper_Init_UART_IRQ ();   // GPIO system
    } else {
    	wrapper_Error_Handle (err);
    }
    sciaTxFifoIsr();

	// Main code
    for(;;) {
    	wrapper_Main();
    }
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Wrapper_Main
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void wrapper_Main ( void ) {
    /* asm (" NOP");
    for ( i=1; i<=10; i++ ) {
        GPIO_setPortData (myGpio, GPIO_Port_A, i );
    }  */

	// Describe of functionality
	// - UART Terminal for control
	// -- Timer #0 as base CLI (Command Line Interface)
	// -- CLI (Command Line Interface)
	// -- PWM #0, #1, #2 : On-Off
	// -- GPIO
	// -- UART_PWM_generator
	// -- LCD and LEDs
}
 /* ========================================================================== */



/* ==========================================================================
 * NAME - Wrapper_Error_Handle
 * IN   - t_error err
 * OUT  - void
 * RET  - void
   ========================================================================== */
void wrapper_Error_Handle (t_error err) {
	switch (err) {
		case E_OK:
			break;

		case E_FAIL:
		case E_BADPTR:
		default:
		    asm(" ESTOP0"); // Test failed!! Stop!
		    for (;;){}
			//break;
	}
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Wrapper_Init_Sys
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error wrapper_Init_Sys (void) {
    // Initialize all the handles needed for this application
    myClk   = CLK_init  ((void *)CLK_BASE_ADDR, sizeof(CLK_Obj));
    myCpu   = CPU_init  ((void *)NULL, sizeof(CPU_Obj));
    myFlash = FLASH_init((void *)FLASH_BASE_ADDR, sizeof(FLASH_Obj));
    myGpio  = GPIO_init ((void *)GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    myPie   = PIE_init  ((void *)PIE_BASE_ADDR, sizeof(PIE_Obj));
    myPll   = PLL_init  ((void *)PLL_BASE_ADDR, sizeof(PLL_Obj));
    myPwm1  = PWM_init  ((void *)PWM_ePWM1_BASE_ADDR, sizeof(PWM_Obj));
    myPwm2  = PWM_init  ((void *)PWM_ePWM2_BASE_ADDR, sizeof(PWM_Obj));
    myPwm3  = PWM_init  ((void *)PWM_ePWM3_BASE_ADDR, sizeof(PWM_Obj));
    myWDog  = WDOG_init ((void *)WDOG_BASE_ADDR, sizeof(WDOG_Obj));
    myTimer = TIMER_init((void *)TIMER0_BASE_ADDR, sizeof(TIMER_Obj));
    myAdc   = ADC_init  ((void *)ADC_BASE_ADDR, sizeof(ADC_Obj));
    mySci   = SCI_init  ((void *)SCIA_BASE_ADDR, sizeof(SCI_Obj));

    // Perform basic system initialization
    WDOG_disable (myWDog);
    CLK_enableAdcClock (myClk);
    (*Device_cal)();
    CLK_disableAdcClock (myClk);

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc (myClk, CLK_OscSrc_Internal);

    // Setup the PLL for x10 /2 which will yield 50Mhz = 10Mhz * 10 / 2
    PLL_setup (myPll, PLL_Multiplier_10, PLL_DivideSelect_ClkIn_by_2);

    // Disable the PIE and all interrupts
    PIE_disable (myPie);
    PIE_disableAllInts (myPie);
    CPU_disableGlobalInts (myCpu);
    CPU_clearIntFlags (myCpu);

    // If running from flash copy RAM only functions to RAM
#ifdef _FLASH
    memcpy (&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Wrapper_Init_Cfg_IRQs
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error wrapper_Init_PWM_IRQs (void) {
    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable (myPie);
    PIE_enable (myPie);

    // Register interrupt handlers in the PIE vector table

#if (1==PWM1_INT_ENABLE)
    PIE_registerPieIntHandler (myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_1, (intVec_t)&epwm1_timer_isr);
#endif //(1==PWM1_INT_ENABLE)

#if (1==PWM2_INT_ENABLE)
    PIE_registerPieIntHandler (myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_2, (intVec_t)&epwm2_timer_isr);
#endif //(1==PWM2_INT_ENABLE)

#if (1==PWM3_INT_ENABLE)
    PIE_registerPieIntHandler (myPie, PIE_GroupNumber_3, PIE_SubGroupNumber_3, (intVec_t)&epwm3_timer_isr);
#endif //(1==PWM3_INT_ENABLE)

    // TODO ??? GROUP ???
    PIE_registerPieIntHandler (myTimer, PIE_GroupNumber_3, PIE_SubGroupNumber_3, (intVec_t)&timer0_isr);

    init_EPwmTimer ();  // For this example, only initialize the ePWM Timers

    // Initalize counters:
    EPwm1TimerIntCount = 0;
    EPwm2TimerIntCount = 0;
    EPwm3TimerIntCount = 0;
    Timer0IntCount     = 0;

    // Enable CPU INT3 which is connected to EPWM1-6 INT
    CPU_enableInt (myCpu, CPU_IntNumber_3);

    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
#if (1==PWM1_INT_ENABLE)
    PIE_enablePwmInt (myPie, PWM_Number_1);
#endif //(1==PWM1_INT_ENABLE)

#if (1==PWM2_INT_ENABLE)
    PIE_enablePwmInt (myPie, PWM_Number_2);
#endif //(1==PWM2_INT_ENABLE)

#if (1==PWM3_INT_ENABLE)
    PIE_enablePwmInt (myPie, PWM_Number_3);
#endif //(1==PWM3_INT_ENABLE)

    PIE_enableTimer0Int (myTimer);

    // Enable global Interrupts and higher priority real-time debug events
    CPU_enableGlobalInts (myCpu);
    CPU_enableDebugInt (myCpu);

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Wrapper_Init_Cfg_Sys
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error wrapper_Init_GPIO (void) {
    // Initalize GPIO
    GPIO_setDirection (myGpio, GPIO_Port_A, 0x000F);
    GPIO_setPortData  (myGpio, GPIO_Port_A, 0x000F);

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - Wrapper_Init_Cfg_Sys
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error wrapper_Init_UART_IRQ (void) {
	// Initalize GPIO
	GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
	GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
	GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);
	GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
	GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);

	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
	EALLOW;    // This is needed to write to EALLOW protected registers
		   PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
		//((PIE_Obj *)myPie)->SCIRXINTA = &sciaRxFifoIsr;
		   PieVectTable.SCITXINTA = &sciaTxFifoIsr;
		//((PIE_Obj *)myPie)->SCITXINTA = &sciaTxFifoIsr;
	EDIS;   // This is needed to disable write to EALLOW protected registers

	// Register interrupt handlers in the PIE vector table
	PIE_registerPieIntHandler(myPie, PIE_GroupNumber_9, PIE_SubGroupNumber_1, (intVec_t)&sciaRxFifoIsr);
	PIE_registerPieIntHandler(myPie, PIE_GroupNumber_9, PIE_SubGroupNumber_2, (intVec_t)&sciaTxFifoIsr);

	scia_init();        // Init SCI-A
	scia_fifo_init();   // Init SCI-A Fifos
    SCI_enableTxInt(mySci);
    SCI_enableRxInt(mySci);


    // Init send data.  After each transmission this data
    // will be updated for the next transmission
    for (i=0; i<2; i++) {
        sdataA[i]=i;
    }
    rdata_pointA = sdataA[0];

    // Enable interrupts required for this example
    PIE_enableInt(myPie, PIE_GroupNumber_9, PIE_InterruptSource_SCIARX);
    PIE_enableInt(myPie, PIE_GroupNumber_9, PIE_InterruptSource_SCIATX);

    CPU_enableInt(myCpu, CPU_IntNumber_9);
    CPU_enableGlobalInts(myCpu);

    return E_OK;
}
/* ========================================================================== */



/* ========================================================================== */
void init_EPwmTimer() {
    // Stop all the TB clocks
    CLK_disableTbClockSync(myClk);
    
#if (1==PWM1_INT_ENABLE)
    CLK_enablePwmClock   (myClk, PWM_Number_1);
    PWM_setSyncMode      (myPwm1, PWM_SyncMode_EPWMxSYNC);      // Setup Sync
    PWM_enableCounterLoad(myPwm1);                              // Allow each timer to be sync'ed
    PWM_setPhase         (myPwm1, 100);
    PWM_setPeriod        (myPwm1, PWM1_TIMER_TBPRD);
    PWM_setCounterMode   (myPwm1, PWM_CounterMode_Up);          // Count up
    PWM_setIntMode       (myPwm1, PWM_IntMode_CounterEqualZero);// Select INT on Zero event
    PWM_enableInt        (myPwm1);                              // Enable INT
    PWM_setIntPeriod     (myPwm1, PWM_IntPeriod_FirstEvent);    // Generate INT on 1st event
#endif //(1==PWM1_INT_ENABLE)

#if (1==PWM2_INT_ENABLE)
    CLK_enablePwmClock   (myClk, PWM_Number_2);
    PWM_setSyncMode      (myPwm2, PWM_SyncMode_EPWMxSYNC);      // Setup Sync
    PWM_enableCounterLoad(myPwm2);                              // Allow each timer to be sync'ed
    PWM_setPhase         (myPwm2, 200);
    PWM_setPeriod        (myPwm2, PWM2_TIMER_TBPRD);
    PWM_setCounterMode   (myPwm2, PWM_CounterMode_Up);          // Count up
    PWM_setIntMode       (myPwm2, PWM_IntMode_CounterEqualZero);// Enable INT on Zero event
    PWM_enableInt        (myPwm2);                              // Enable INT
    PWM_setIntPeriod     (myPwm2, PWM_IntPeriod_SecondEvent);   // Generate INT on 2nd event
#endif //(1==PWM2_INT_ENABLE)

#if (1==PWM3_INT_ENABLE)
    CLK_enablePwmClock   (myClk, PWM_Number_3);
    PWM_setSyncMode      (myPwm3, PWM_SyncMode_EPWMxSYNC);      // Setup Sync
    PWM_enableCounterLoad(myPwm3);                              // Allow each timer to be sync'ed
    PWM_setPhase         (myPwm3, 300);
    PWM_setPeriod        (myPwm3, PWM3_TIMER_TBPRD);
    PWM_setCounterMode   (myPwm3, PWM_CounterMode_Up);          // Count up
    PWM_setIntMode       (myPwm3, PWM_IntMode_CounterEqualZero);// Enable INT on Zero event
    PWM_enableInt        (myPwm3);                              // Enable INT
    PWM_setIntPeriod     (myPwm3, PWM_IntPeriod_ThirdEvent);    // Generate INT on 3rd event
#endif //(1==PWM3_INT_ENABLE)

    // Start all the timers synced
    CLK_enableTbClockSync(myClk);
}
/* ========================================================================== */



/* ========================================================================== */
// Interrupt routines uses in this example:
interrupt void epwm1_timer_isr(void) {
#if (1==PWM1_INT_ENABLE)
    EPwm1TimerIntCount++;

    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm1);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);

    GPIO_setPortData (myGpio, GPIO_Port_A, ++i_pwm0&0x1 ? 0xE : 0xF );
#endif //(1==PWM1_INT_ENABLE)
}
/* ========================================================================== */



/* ========================================================================== */
interrupt void epwm2_timer_isr(void) {
#if (1==PWM2_INT_ENABLE)
    EPwm2TimerIntCount++;

    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm2);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);
#endif //(1==PWM2_INT_ENABLE)
}
/* ========================================================================== */



/* ========================================================================== */
interrupt void epwm3_timer_isr(void) {
#if (1==PWM3_INT_ENABLE)
    EPwm3TimerIntCount++;

    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm3);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);
#endif (1==PWM3_INT_ENABLE)
}
/* ========================================================================== */



/* ========================================================================== */
interrupt void timer0_isr(void) {
    Timer0IntCount++;

    // Clear INT flag for this timer
    TIMER_clearFlag(myTimer);

    // Acknowledge this interrupt to receive more interrupts from group 3
    //TIMER_clearIntFlag(myTimer);
}
/* ========================================================================== */



/* ========================================================================== */
/* ========================================================================== */
interrupt void sciaTxFifoIsr(void)
{
    uint16_t i;
    for(i=0; i< 2; i++) {
        // Send data
        SCI_write(mySci, sdataA[i]);
    }

    for(i=0; i< 2; i++) {
        //Increment send data for next cycle
        sdataA[i] = (sdataA[i]+1) & 0x00FF;
    }

    // Clear SCI Interrupt flag
    SCI_clearTxFifoInt(mySci);

    // Issue PIE ACK
    PIE_clearInt(myPie, PIE_GroupNumber_9);

    return;
}
/* ========================================================================== */



/* ========================================================================== */
/* ========================================================================== */
interrupt void sciaRxFifoIsr(void)
{
    uint16_t  i;
    for ( i=0; i<2; i++ ) {
        rdataA[i] = SCI_read(mySci);  // Read data
    }
    for ( i=0; i<2; i++ ) {
       if (rdataA[i] != ( (rdata_pointA+i) & 0x00FF) )  // Check received data
       error();
    }
    rdata_pointA = (rdata_pointA+1) & 0x00FF;

    SCI_clearRxFifoOvf(mySci);     // Clear Overflow  flag
    SCI_clearRxFifoInt(mySci);     // Clear Interrupt flag
    PIE_clearInt(myPie, PIE_GroupNumber_9);   // Issue PIE ack

    return;
}
/* ========================================================================== */



/* ========================================================================== */
/* ========================================================================== */
void scia_init()
{
    CLK_enableSciaClock(myClk);

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

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
#if (CPU_FRQ_50MHZ)
    //SCI_setBaudRate(mySci, SCI_BaudRate_9_6_kBaud);
    SCI_setBaudRate(mySci, SCI_BaudRate_115_2_kBaud);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)129);
#endif
    SCI_enable(mySci);

    return;
}
/* ========================================================================== */



/* ========================================================================== */
/* ========================================================================== */
void scia_fifo_init()
{
    SCI_enableFifoEnh(mySci);
    SCI_resetTxFifo(mySci);
    SCI_clearTxFifoInt(mySci);
    SCI_resetChannels(mySci);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_2_Words);
    SCI_enableTxFifoInt(mySci);

    SCI_resetRxFifo(mySci);
    SCI_clearRxFifoInt(mySci);
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_2_Words);
    SCI_enableRxFifoInt(mySci);

    return;
}
/* ========================================================================== */



/* ========================================================================== */
void error(void)
{
    asm(" ESTOP0"); // Test failed!! Stop!
    for (;;){ }
}
/* ========================================================================== */



//===========================================================================
// No more.
//===========================================================================

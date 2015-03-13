/* ========================================================================== */
/* ========================================================================== */
//  Group:          C2000
//  Target Device:  TMS320F2802x
/* ========================================================================== */
/* ========================================================================== */

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


/* ========================================================================== */
// Configure which ePWM timer interrupts are enabled at the PIE level:
// 1 = enabled,  0 = disabled
#define PWM1_INT_ENABLE   0
#define PWM2_INT_ENABLE   0
#define PWM3_INT_ENABLE   0

// Configure the period for each timer
#define PWM1_TIMER_TBPRD  0xFFFF
//#define PWM1_TIMER_TBPRD  0x1FFF
#define PWM2_TIMER_TBPRD  0x1FFF
#define PWM3_TIMER_TBPRD  0x1FFF

/* ========================================================================== */
#if (CPU_FRQ_40MHZ)
#define CPU_FREQ          (40000000) // Default=40 MHz Change to 50E6 for 50 MHz devices
#endif

#if (CPU_FRQ_50MHZ)
#define CPU_FREQ          (50000000) // Default=40 MHz Change to 50E6 for 50 MHz devices
#endif

#if (CPU_FRQ_60MHZ)
#define CPU_FREQ          (60000000) // Default=40 MHz Change to 50E6 for 50 MHz devices
#endif

#define LSPCLK_FREQ       (CPU_FREQ/4)

//#define SCI_FREQ          100E3
#define SCI_FREQ          115200
//#define SCI_PRD           (LSPCLK_FREQ/(SCI_FREQ*8))-1
#define SCI_BRR           (LSPCLK_FREQ/(SCI_FREQ*8))-1
/* ========================================================================== */

#define  USE_UART_IRQ_1   (0)
#define  USE_UART_IRQ_2   (1)

typedef enum {
	TERMINAL_PRINT=0,
	TERMINAL_READ,
	TERMINAL_SHOW,
	TERMINAL_ENTER,
	TERMINAL_WAIT,
} t_terminal_state;

typedef enum {
	E_OK=0,
	E_FAIL,
	E_BADPTR
} t_error;


// Prototype statements for functions found within this file.
interrupt void epwm1_timer_isr(void);
interrupt void epwm2_timer_isr(void);
interrupt void epwm3_timer_isr(void);
interrupt void timer0_isr(void);
void           init_EPwmTimer(void);

t_error wrapper_Init_Sys (void);
t_error wrapper_Init_PWM_IRQs (void);
t_error wrapper_Init_GPIO (void);
//t_error wrapper_Init_UART_pooling (void);
//t_error wrapper_Init_UART_IRQ (void);
void    wrapper_Main ( void );
void    wrapper_Error_Handle( t_error err );
void    error(void);

// Prototype statements for functions found within this file.
interrupt void  sciaTxFifoIsr (void);
interrupt void  sciaRxFifoIsr (void);
void  scia_init(void);
void  scia_fifo_init(void);
void  scia_echoback_init (void);
void  scia_xmit (int a);
void  scia_msg (char * msg);
void  scia_echoback_fifo_init (void);

t_error wrapper_Init_UART_pooling (void);
t_error wrapper_Init_UART_IRQ (void);


/* ========================================================================== */
/* Global variables */
/* ========================================================================== */

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


t_error      err    = E_OK;
int          i      = 0;
int          i_pwm0 = 0;

char         *p_sci_msg;
uint16_t     LoopCount;
uint16_t     ErrorCount;
uint16_t     ReceivedChar;
#define      RX_LEN   (8)
uint16_t     sdataA[RX_LEN];     // Send data for SCI-A
uint16_t     rdataA[RX_LEN];     // Received data for SCI-A
uint16_t     rdata_pointA;  // Used for checking the received data

uint16_t     RxTx;

/* ==========================================================================
 * MAIN
 * ========================================================================== */
void main (void) {
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
    	err = wrapper_Init_GPIO ();   // Init GPIO system
    } else {
    	wrapper_Error_Handle (err);
    }

    if (E_OK==err) {
    	err = wrapper_Init_UART_IRQ ();   // Init UART IRQ
    	//err = wrapper_Init_UART_pooling ();   // Init UART without IRQ
    } else {
    	wrapper_Error_Handle (err);
    }

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
		case E_OK:		break;
		case E_FAIL:	case E_BADPTR:		default:
			error();
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
	SysCtrlRegs.PCLKCR0.bit.I2CAENCLK  = 0;  // I2C
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK  = 0;	 // SPI-A
	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK  = 0;  // SCI-A
	SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 0;	 // eCAP1
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 0;  // ePWM1
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 0;  // ePWM2
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 0;  // ePWM3
	SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0;  // ePWM4
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC  = 0;  // Enable TBCLK
	//------------------------------------------------

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
    //PIE_registerPieIntHandler (myTimer, PIE_GroupNumber_3, PIE_SubGroupNumber_3, (intVec_t)&timer0_isr);

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

    //PIE_enableTimer0Int (myTimer);

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
    GPIO_setDirection (myGpio, GPIO_Number_0, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_1, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_2, GPIO_Direction_Output);
    GPIO_setDirection (myGpio, GPIO_Number_3, GPIO_Direction_Output);
    GPIO_setPortData  (myGpio, GPIO_Port_A, 0x000F);

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================

    return E_OK;
 * NAME - init_EPwmTimer
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void init_EPwmTimer (void) {
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



/* ==========================================================================
 * NAME - epwm1_timer_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
// Interrupt routines uses in this example:
interrupt void epwm1_timer_isr (void) {
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



/* ==========================================================================
 * NAME - epwm2_timer_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void epwm2_timer_isr (void) {
#if (1==PWM2_INT_ENABLE)
    EPwm2TimerIntCount++;

    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm2);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);
#endif //(1==PWM2_INT_ENABLE)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - epwm3_timer_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void epwm3_timer_isr (void) {
#if (1==PWM3_INT_ENABLE)
    EPwm3TimerIntCount++;

    // Clear INT flag for this timer
    PWM_clearIntFlag(myPwm3);

    // Acknowledge this interrupt to receive more interrupts from group 3
    PIE_clearInt(myPie, PIE_GroupNumber_3);
#endif //(1==PWM3_INT_ENABLE)
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - timer0_isr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void timer0_isr (void) {
    Timer0IntCount++;

    // Clear INT flag for this timer
    TIMER_clearFlag(myTimer);

    // Acknowledge this interrupt to receive more interrupts from group 3
    //TIMER_clearIntFlag(myTimer);
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
 * NAME - wrapper_Init_UART_pooling
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error wrapper_Init_UART_pooling (void) {
    // Initalize GPIO
    GPIO_setPullUp(myGpio, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(myGpio, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setQualification(myGpio, GPIO_Number_28, GPIO_Qual_ASync);
    GPIO_setMode(myGpio, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(myGpio, GPIO_Number_29, GPIO_29_Mode_SCITXDA);

    // Setup a debug vector table and enable the PIE
    PIE_setDebugIntVectorTable(myPie);
    PIE_enable(myPie);

    LoopCount = 0;
    ErrorCount = 0;

    scia_echoback_init();           // Initalize SCI for echoback
    scia_echoback_fifo_init();      // Initialize the SCI FIFO

    p_sci_msg = "\r\n\n\nHello World!\0";
    scia_msg(p_sci_msg);

    p_sci_msg = "\r\nYou will enter a character, and the DSP will echo it back! \n\0";
    scia_msg(p_sci_msg);

    return E_OK;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - wrapper_Init_UART_IRQ
 * IN   - void
 * OUT  - void
 * RET  - t_error err
   ========================================================================== */
t_error wrapper_Init_UART_IRQ (void) {
#if (USE_UART_IRQ_1==1)
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
/*
	// Register interrupt handlers in the PIE vector table
	PIE_registerPieIntHandler(myPie,
			PIE_GroupNumber_9, PIE_SubGroupNumber_1, (intVec_t)&sciaRxFifoIsr);
	PIE_registerPieIntHandler(myPie,
			PIE_GroupNumber_9, PIE_SubGroupNumber_2, (intVec_t)&sciaTxFifoIsr);
*/
	scia_init();            // Init SCI-A
	scia_fifo_init();       // Init SCI-A Fifos
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
#endif //USE_UART_IRQ_1

#if (USE_UART_IRQ_2==1)
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
#endif //USE_UART_IRQ_2

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
#if (USE_UART_IRQ_1==1)
/*    SCI_enableFifoEnh(mySci);
    SCI_resetTxFifo(mySci);
    SCI_clearTxFifoInt(mySci);
    SCI_resetChannels(mySci);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_2_Words);
    SCI_enableTxFifoInt(mySci);

    SCI_resetRxFifo(mySci);
    SCI_clearRxFifoInt(mySci);
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_2_Words);
    SCI_enableRxFifoInt(mySci);
*/
    return;
#endif //USE_UART_IRQ_1

#if (USE_UART_IRQ_2==1)
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
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_1_Word);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_Empty);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
#if (CPU_FRQ_50MHZ)
    //SCI_setBaudRate(mySci, SCI_BaudRate_9_6_kBaud);
    SCI_setBaudRate(mySci, /*SCI_BaudRate_115_2_kBaud*/ (SCI_BaudRate_e)13);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)129);
#endif
    SCI_enable(mySci);

#endif //USE_UART_IRQ_2
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - sciaTxFifoIsr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void sciaTxFifoIsr (void) {
#if (USE_UART_IRQ_1==1)
    uint16_t i=0;
    for(i=0; i< 2; i++) {
        // Send data
        //SCI_write(mySci, sdataA[i]);
  	    SciaRegs.SCITXBUF=rdataA[i];     // Send data
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
#endif //USE_UART_IRQ_1

#if (USE_UART_IRQ_2==1)
    //Uint16 i=0;
    //for(i=0; i< 2; i++) {
 	//   SciaRegs.SCITXBUF=rdataA[i];   // Send data
	//}
    SciaRegs.SCITXBUF=RxTx; //rdataA[0]; // Send data
    //for(i=0; i< 2; i++)               //Increment send data for next cycle
    //{
 	//   sdataA[i] = (sdataA[i]+1) & 0x00FF;
	//}

	SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;	// Clear SCI Interrupt flag
	//SciaRegs.SCIFFTX.bit.TXFFINT=0;	    // Clear SCI Interrupt flag
	PieCtrlRegs.PIEACK.all |= 0x100;    // Issue PIE ACK

    //SCI_clearTxFifoInt(mySci);               // Clear SCI Interrupt flag
    //PIE_clearInt(myPie, PIE_GroupNumber_9);  // Issue PIE ACK

    #endif //USE_UART_IRQ_2
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - sciaRxFifoIsr
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
interrupt void sciaRxFifoIsr (void) {
#if (USE_UART_IRQ_1==1)
    uint16_t  i=0;
    for ( i=0; i<2; i++ ) {
        //rdataA[i] = SCI_read(mySci);  // Read data
 	    rdataA[i]=SciaRegs.SCIRXBUF.all;	 // Read data
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
#endif //USE_UART_IRQ_1

#if (USE_UART_IRQ_2==1)
    static Uint16 i=0;
	//for (i=0; i<RX_LEN; i++) {
	//   rdataA[i]=SciaRegs.SCIRXBUF.all;	 // Read data
	//}
	//for(i=0;i<2;i++) {                  // Check received data
	//   if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) ) error();
	//}
	//rdata_pointA = (rdata_pointA+1) & 0x00FF;

    if (SciaRegs.SCIRXBUF.bit.RXDT==0) i=0;

    if (i<8) {
    	rdataA[i++]=SciaRegs.SCIRXBUF.all;	 // Read data

    	SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    	SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    	PieCtrlRegs.PIEACK.all |= 0x100;     // Issue PIE ack
    } else {
    	i=0;

    	RxTx='x';
        SciaRegs.SCITXBUF=RxTx;              // Send data
    	PieCtrlRegs.PIEACK.all |= 0x100;     // Issue PIE ack
    }

#endif //USE_UART_IRQ_2
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - scia_init
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
void scia_init (void) {
#if (USE_UART_IRQ_1==1)
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
    //SCI_disableLoopBack(mySci);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
#if (CPU_FRQ_50MHZ)
    //SCI_setBaudRate(mySci, SCI_BaudRate_9_6_kBaud);
    SCI_setBaudRate(mySci, /*SCI_BaudRate_115_2_kBaud*/(SCI_BaudRate_e)13);
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)129);
#endif
    SCI_enable(mySci);

    return;
#endif //USE_UART_IRQ_1

#if (USE_UART_IRQ_2==1)
#endif //USE_UART_IRQ_2
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - scia_echoback_init
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
// Test 1,SCIA  DLB, 8-bit word, baud rate 0x000F, default, 1 STOP bit, no parity
void scia_echoback_init (void) {
    CLK_enableSciaClock(myClk);

    // 1 stop bit, No loopback, No parity,8 char bits, async mode, idle-line protocol
    SCI_disableParity(mySci);
    SCI_setNumStopBits(mySci, SCI_NumStopBits_One);
    SCI_setCharLength(mySci, SCI_CharLength_8_Bits);

    SCI_enableTx(mySci);
    SCI_enableRx(mySci);
    SCI_enableTxInt(mySci);
    SCI_enableRxInt(mySci);

    // SCI BRR = LSPCLK/(SCI BAUDx8) - 1
#if (CPU_FRQ_60MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)194);
#elif (CPU_FRQ_50MHZ)
    //SCI_setBaudRate(mySci, (SCI_BaudRate_e)162);  // @ 9600
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)13);   // @ 115200
#elif (CPU_FRQ_40MHZ)
    SCI_setBaudRate(mySci, (SCI_BaudRate_e)129);
#endif
    SCI_enable(mySci);

    return;
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - scia_xmit
 * IN   - int
 * OUT  - void
 * RET  - void
   ========================================================================== */
// Transmit a character from the SCI
void scia_xmit (int a) {
//    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    while(SCI_getTxFifoStatus(mySci) != SCI_FifoStatus_Empty){
    }
//    SciaRegs.SCITXBUF=a;
    SCI_putDataBlocking(mySci, a);
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - scia_msg
 * IN   - char *
 * OUT  - void
 * RET  - void
   ========================================================================== */
void scia_msg (char * msg) {
    int i;
    i = 0;
    while(msg[i] != '\0')     {
        scia_xmit(msg[i]);
        i++;
    }
}
/* ========================================================================== */



/* ==========================================================================
 * NAME - scia_echoback_fifo_init
 * IN   - void
 * OUT  - void
 * RET  - void
   ========================================================================== */
// Initalize the SCI FIFO
void scia_echoback_fifo_init (void) {
    SCI_enableFifoEnh(mySci);
    SCI_resetTxFifo(mySci);
    SCI_clearTxFifoInt(mySci);
    SCI_resetChannels(mySci);
    SCI_setTxFifoIntLevel(mySci, SCI_FifoLevel_Empty);

    SCI_resetRxFifo(mySci);
    SCI_clearRxFifoInt(mySci);
    SCI_setRxFifoIntLevel(mySci, SCI_FifoLevel_4_Words);

    return;
}
/* ========================================================================== */


//===========================================================================
// No more.
//===========================================================================

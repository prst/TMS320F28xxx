
//###########################################################################
// $TI Release: f2802x Support Library v200 $
// $Release Date: Tue Jul 24 10:01:39 CDT 2012 $
//###########################################################################
//*/

MEMORY  {
 PAGE 0:    /* Program Memory */
   /* For this example, L0 is split between PAGE 0 and PAGE 1 */
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
   BEGIN      : origin = 0x000000, length = 0x000002
   //RAMM0      : origin = 0x000050, length = 0x0003B0
   //RAMM01     : origin = 0x000002, length = 0x0007B0
   //RAML0      : origin = 0x008000, length = 0x000800
   RAMM01     : origin = 0x000002, length = 0x000800
   //PRAMH0     : origin = 0x008000, length = 0x000500
   PRAMH0     : origin = 0x008000, length = 0x001000

   IQTABLES   : origin = 0x3FE000, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3  : origin = 0x3FEBDC, length = 0x0000AA     /* IQ Math Tables in Boot ROM */
   RESET      : origin = 0x3FFFC0, length = 0x000002
   BOOTROM    : origin = 0x3FF27C, length = 0x000D44


 PAGE 1:    /* Data Memory */
   //RAMM0    : origin = 0x000000, length = 0x000400
   //RAMM1    : origin = 0x000400, length = 0x000400
   DEV_EMU     : origin = 0x000880, length = 0x000105     /* device emulation registers */
   SYS_PWR_CTL : origin = 0x000985, length = 0x000003     /* System power control registers */
   FLASH_REGS  : origin = 0x000A80, length = 0x000060     /* FLASH registers */
   CSM         : origin = 0x000AE0, length = 0x000010     /* code security module registers */
   ADC_RESULT  : origin = 0x000B00, length = 0x000020     /* ADC Results register */
   CPU_TIMER0  : origin = 0x000C00, length = 0x000008     /* CPU Timer0 registers */
   CPU_TIMER1  : origin = 0x000C08, length = 0x000008     /* CPU Timer0 registers (CPU Timer1 & Timer2 reserved TI use)*/
   CPU_TIMER2  : origin = 0x000C10, length = 0x000008     /* CPU Timer0 registers (CPU Timer1 & Timer2 reserved TI use)*/
   PIE_CTRL    : origin = 0x000CE0, length = 0x000020     /* PIE control registers */
   PIE_VECT    : origin = 0x000D00, length = 0x000100     /* PIE Vector Table */
   COMP1       : origin = 0x006400, length = 0x000020     /* Comparator 1 registers */
   COMP2       : origin = 0x006420, length = 0x000020     /* Comparator 2 registers */
   EPWM1       : origin = 0x006800, length = 0x000040     /* Enhanced PWM 1 registers */
   EPWM2       : origin = 0x006840, length = 0x000040     /* Enhanced PWM 2 registers */
   EPWM3       : origin = 0x006880, length = 0x000040     /* Enhanced PWM 3 registers */
   EPWM4       : origin = 0x0068C0, length = 0x000040     /* Enhanced PWM 4 registers */
   ECAP1       : origin = 0x006A00, length = 0x000020     /* Enhanced Capture 1 registers */
   GPIOCTRL    : origin = 0x006F80, length = 0x000040     /* GPIO control registers */
   GPIODAT     : origin = 0x006FC0, length = 0x000020     /* GPIO data registers */
   GPIOINT     : origin = 0x006FE0, length = 0x000020     /* GPIO interrupt/LPM registers */
   SYSTEM      : origin = 0x007010, length = 0x000020     /* System control registers */
   SPIA        : origin = 0x007040, length = 0x000010     /* SPI-A registers */
   SCIA        : origin = 0x007050, length = 0x000010     /* SCI-A registers */
   NMIINTRUPT  : origin = 0x007060, length = 0x000010     /* NMI Watchdog Interrupt Registers */
   XINTRUPT    : origin = 0x007070, length = 0x000010     /* external interrupt registers */
   ADC         : origin = 0x007100, length = 0x000080     /* ADC registers */
   I2CA        : origin = 0x007900, length = 0x000040     /* I2C-A registers */

   PARTID     : origin = 0x3D7FFF, length = 0x000001     /* Part ID register  */
   CSM_RSVD   : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   CSM_PWL    : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations. */

   //DRAMH0      : origin =  0x008500, length = 0x000b00
}


SECTIONS  {
   //csmpasswds       : > CSM_PWL, PAGE = 1
   csm_rsvd         : > CSM_RSVD,   PAGE = 1

   codestart        : > BEGIN,     PAGE = 0
   ramfuncs         : > RAMM01      PAGE = 0

   .reset           : > PRAMH0,      PAGE = 0
   //.text            : > PRAMH0,      PAGE = 0
   //.cinit           : > PRAMH0,      PAGE = 0
   .text            : >> PRAMH0 | RAMM01,      PAGE = 0
   .cinit           : > RAMM01,      PAGE = 0

   /* Allocate data areas: */
   //.stack           : > RAMM1,       PAGE = 1
   .stack           : > PRAMH0,       PAGE = 0
/*   .bss             : > DRAMH0,      PAGE = 1
   .ebss            : > DRAMH0,      PAGE = 1
   .const           : > DRAMH0,      PAGE = 1
   .econst          : > DRAMH0,      PAGE = 1
   .sysmem          : > DRAMH0,      PAGE = 1*/
   .bss             : > PRAMH0,      PAGE = 0
   .ebss            : > PRAMH0,      PAGE = 0
   .const           : > PRAMH0,      PAGE = 0
   .econst          : > PRAMH0,      PAGE = 0
   .sysmem          : > PRAMH0,      PAGE = 0

/*** PIE Vect Table and Boot ROM Variables Structures ***/
  UNION run = PIE_VECT, PAGE = 1
   {
      PieVectTableFile
      GROUP  {
         EmuKeyVar
         EmuBModeVar
         FlashCallbackVar
         FlashScalingVar
      }
   }

/*** Peripheral Frame 0 Register Structures ***/
   DevEmuRegsFile    : > DEV_EMU,     PAGE = 1
   SysPwrCtrlRegsFile: > SYS_PWR_CTL, PAGE = 1
   FlashRegsFile     : > FLASH_REGS,  PAGE = 1
   CsmRegsFile       : > CSM,         PAGE = 1
   AdcResultFile     : > ADC_RESULT,  PAGE = 1
   CpuTimer0RegsFile : > CPU_TIMER0,  PAGE = 1
   CpuTimer1RegsFile : > CPU_TIMER1,  PAGE = 1
   CpuTimer2RegsFile : > CPU_TIMER2,  PAGE = 1
   PieCtrlRegsFile   : > PIE_CTRL,    PAGE = 1

/*** Peripheral Frame 1 Register Structures ***/
   ECap1RegsFile     : > ECAP1        PAGE = 1
   GpioCtrlRegsFile  : > GPIOCTRL     PAGE = 1
   GpioDataRegsFile  : > GPIODAT      PAGE = 1
   GpioIntRegsFile   : > GPIOINT      PAGE = 1

/*** Peripheral Frame 2 Register Structures ***/
   SysCtrlRegsFile   : > SYSTEM,      PAGE = 1
   SpiaRegsFile      : > SPIA,        PAGE = 1
   SciaRegsFile      : > SCIA,        PAGE = 1
   NmiIntruptRegsFile: > NMIINTRUPT,  PAGE = 1
   XIntruptRegsFile  : > XINTRUPT,    PAGE = 1
   AdcRegsFile       : > ADC,         PAGE = 1
   I2caRegsFile      : > I2CA,        PAGE = 1

/*** Peripheral Frame 3 Register Structures ***/
   Comp1RegsFile     : > COMP1,       PAGE = 1
   Comp2RegsFile     : > COMP2,       PAGE = 1
   EPwm1RegsFile     : > EPWM1        PAGE = 1
   EPwm2RegsFile     : > EPWM2        PAGE = 1
   EPwm3RegsFile     : > EPWM3        PAGE = 1
   EPwm4RegsFile     : > EPWM4        PAGE = 1


/*** Code Security Module Register Structures ***/
   CsmPwlFile        : > CSM_PWL,     PAGE = 1

/*** Device Part ID Register Structures ***/
   PartIdRegsFile    : > PARTID,      PAGE = 1

}

/*
//###########################################################################
//
// FILE:    28027_RAM_lnk.cmd
//
// TITLE:   Linker Command File For 28027 examples that run out of RAM
//
//          This ONLY includes all SARAM blocks on the 28027 device.
//          This does not include flash or OTP.
//
//          Keep in mind that L0 is protected by the code
//          security module.
//
//          What this means is in most cases you will want to move to
//          another memory map file which has more memory defined.
//
//###########################################################################
// $TI Release: F2802x Support Library v222 $
// $Release Date: Thu Jan 15 13:56:57 CST 2015 $
// $Copyright: Copyright (C) 2008-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
*/

/* ======================================================
// For Code Composer Studio V2.2 and later
// ---------------------------------------
// In addition to this memory linker command file,
// add the header linker command file directly to the project.
// The header linker command file is required to link the
// peripheral structures to the proper locations within
// the memory map.
//
// The header linker files are found in <base>\DSP2802x_headers\cmd
//
// For BIOS applications add:      DSP2802x_Headers_BIOS.cmd
// For nonBIOS applications add:   DSP2802x_Headers_nonBIOS.cmd
========================================================= */

/* ======================================================
// For Code Composer Studio prior to V2.2
// --------------------------------------
// 1) Use one of the following -l statements to include the
// header linker command file in the project. The header linker
// file is required to link the peripheral structures to the proper
// locations within the memory map                                    */

/* Uncomment this line to include file only for non-BIOS applications */
/* -l DSP2802x_Headers_nonBIOS.cmd */

/* Uncomment this line to include file only for BIOS applications */
/* -l DSP2802x_Headers_BIOS.cmd */

/* 2) In your project add the path to <base>\DSP2802x_headers\cmd to the
   library search path under project->build options, linker tab,
   library search path (-i).
/*========================================================= */

/* Define the memory block start/length for the DSP2802x
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F28027 are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         The L0 memory blocks is mirrored - that is
         it can be accessed in high memory or low memory.
         For simplicity only one instance is used in this
         linker file.

         Contiguous SARAM memory blocks can be combined
         if required to create a larger memory block.
*/

MEMORY
{
PAGE 0 :
   /* For this example, L0 is split between PAGE 0 and PAGE 1 */
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */
   BEGIN      : origin = 0x000000, length = 0x000002
   //RAMM0      : origin = 0x000050, length = 0x0003B0
   RAMM01     : origin = 0x000050, length = 0x0007B0
   //RAML0      : origin = 0x008000, length = 0x000800
   PRAML0     : origin = 0x008000, length = 0x001000
   //DRAML0      : origin = 0x008900, length = 0x000700

   IQTABLES   : origin = 0x3FE000, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2  : origin = 0x3FEB50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3  : origin = 0x3FEBDC, length = 0x0000AA     /* IQ Math Tables in Boot ROM */
   RESET      : origin = 0x3FFFC0, length = 0x000002
   BOOTROM    : origin = 0x3FF27C, length = 0x000D44

#if 1
   DEV_EMU     : origin = 0x000880, length = 0x000105     /* device emulation registers */
   SYS_PWR_CTL : origin = 0x000985, length = 0x000003     /* System power control registers */
   FLASH_REGS  : origin = 0x000A80, length = 0x000060     /* FLASH registers */
   CSM         : origin = 0x000AE0, length = 0x000010     /* code security module registers */
   CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
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
   //CSM_PWL     : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations. */
   CSM_PWL_P0  : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */
   PARTID      : origin = 0x3D7FFF, length = 0x000001     /* Part ID register location */
#endif


PAGE 1 :

   /* For this example, L0 is split between PAGE 0 and PAGE 1 */
   //BOOT_RSVD   : origin = 0x000002, length = 0x00004E     /* Part of M0, BOOT rom will use this for stack */
   //RAMM1       : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   //DRAML0      : origin = 0x008900, length = 0x000700
}


SECTIONS  {
   /* Setup for "boot to SARAM" mode:
      The codestart section (found in DSP28_CodeStartBranch.asm)
      re-directs execution to the start of user code.  */

   csmpasswds       : > CSM_PWL_P0,   PAGE = 0
   csm_rsvd         : > CSM_RSVD,     PAGE = 0

   codestart        : > BEGIN,     PAGE = 0
   ramfuncs         : >> PRAML0 | RAMM01,     PAGE = 0

   .text            : >> PRAML0 | RAMM01,    PAGE = 0
   .cinit           : > RAMM01,     PAGE = 0
   .pinit           : >> PRAML0 | RAMM01,     PAGE = 0
   .switch          : >> PRAML0 | RAMM01,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */

   .stack           : > RAMM01,     PAGE = 0
   //.ebss            : > DRAML0,    PAGE = 1
   //.econst          : > DRAML0,    PAGE = 1
   .ebss            : > PRAML0,    PAGE = 0
   .econst          : > PRAML0,    PAGE = 0
   .esysmem         : > RAMM01,     PAGE = 0

   IQmath           : > PRAML0,    PAGE = 0
   IQmathTables     : > IQTABLES,  PAGE = 0, TYPE = NOLOAD

  /* Uncomment the section below if calling the IQNexp() or IQexp()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables2    : > IQTABLES2, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNexpTable.obj> (IQmathTablesRam)

   }
   */
   /* Uncomment the section below if calling the IQNasin() or IQasin()
      functions from the IQMath.lib library in order to utilize the
      relevant IQ Math table in Boot ROM (This saves space and Boot ROM
      is 1 wait-state). If this section is not uncommented, IQmathTables2
      will be loaded into other memory (SARAM, Flash, etc.) and will take
      up space, but 0 wait-state is possible.
   */
   /*
   IQmathTables3    : > IQTABLES3, PAGE = 0, TYPE = NOLOAD
   {

              IQmath.lib<IQNasinTable.obj> (IQmathTablesRam)

   }
   */

#if 1
   /*UNION run = PIEVECT,   PAGE = 0
   {
      PieVectTableFile     : TYPE=DSECT
      GROUP {
         EmuKeyVar         : TYPE=DSECT
         EmuBModeVar       : TYPE=DSECT
         FlashCallbackVar  : TYPE=DSECT
         FlashScalingVar   : TYPE=DSECT
      }
   }*/

/*** Peripheral Frame 0 Register Structures ***/
   DevEmuRegsFile    : > DEV_EMU,     PAGE = 0
   SysPwrCtrlRegsFile: > SYS_PWR_CTL, PAGE = 0
   FlashRegsFile     : > FLASH_REGS,  PAGE = 0
   CsmRegsFile       : > CSM,         PAGE = 0
   AdcResultFile     : > ADC_RESULT,  PAGE = 0
   CpuTimer0RegsFile : > CPU_TIMER0,  PAGE = 0
   CpuTimer1RegsFile : > CPU_TIMER1,  PAGE = 0
   CpuTimer2RegsFile : > CPU_TIMER2,  PAGE = 0
   PieCtrlRegsFile   : > PIE_CTRL,    PAGE = 0
   PieVectTableFile  : > PIE_VECT,    PAGE = 0

/*** Peripheral Frame 1 Register Structures ***/
   ECap1RegsFile     : > ECAP1        PAGE = 0
   GpioCtrlRegsFile  : > GPIOCTRL     PAGE = 0
   GpioDataRegsFile  : > GPIODAT      PAGE = 0
   GpioIntRegsFile   : > GPIOINT      PAGE = 0

/*** Peripheral Frame 2 Register Structures ***/
   SysCtrlRegsFile   : > SYSTEM,      PAGE = 0
   SpiaRegsFile      : > SPIA,        PAGE = 0
   SciaRegsFile      : > SCIA,        PAGE = 0
   NmiIntruptRegsFile: > NMIINTRUPT,  PAGE = 0
   XIntruptRegsFile  : > XINTRUPT,    PAGE = 0
   AdcRegsFile       : > ADC,         PAGE = 0
   I2caRegsFile      : > I2CA,        PAGE = 0

/*** Peripheral Frame 3 Register Structures ***/
   Comp1RegsFile     : > COMP1,       PAGE = 0
   Comp2RegsFile     : > COMP2,       PAGE = 0
   EPwm1RegsFile     : > EPWM1        PAGE = 0
   EPwm2RegsFile     : > EPWM2        PAGE = 0
   EPwm3RegsFile     : > EPWM3        PAGE = 0
   EPwm4RegsFile     : > EPWM4        PAGE = 0

/*** Code Security Module Register Structures ***/
   //CsmPwlFile        : > CSM_PWL,     PAGE = 0
   CsmPwlFile        : > CSM_PWL_P0,     PAGE = 0

/*** Device Part ID Register Structures ***/
   PartIdRegsFile    : > PARTID,      PAGE = 0
#endif

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/

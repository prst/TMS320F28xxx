/*
//###########################################################################
//
// FILE:    F2802x_generic_flash.cmd
//
// TITLE:    Generic Linker Command File for f2802x devices
//
//###########################################################################
// $TI Release: LaunchPad f2802x Support Library v100 $
// $Release Date: Wed Jul 25 10:45:39 CDT 2012 $
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
// The header linker files are found in <base>\DSP2802_Headers\cmd
//
// For BIOS applications add:      F2802x_Headers_BIOS.cmd
// For nonBIOS applications add:   F2802x_Headers_nonBIOS.cmd
========================================================= */

/* ======================================================
// For Code Composer Studio prior to V2.2
// --------------------------------------
// 1) Use one of the following -l statements to include the
// header linker command file in the project. The header linker
// file is required to link the peripheral structures to the proper
// locations within the memory map                                    */

/* Uncomment this line to include file only for non-BIOS applications */
/* -l F2802x0_Headers_nonBIOS.cmd */

/* Uncomment this line to include file only for BIOS applications */
/* -l F2802x0_Headers_BIOS.cmd */

/* 2) In your project add the path to <base>\F2802x0_headers\cmd to the
   library search path under project->build options, linker tab,
   library search path (-i).
/*========================================================= */

/* Define the memory block start/length for the F280220
   PAGE 0 will be used to organize program sections
   PAGE 1 will be used to organize data sections

   Notes:
         Memory blocks on F2802x are uniform (ie same
         physical memory) in both PAGE 0 and PAGE 1.
         That is the same memory region should not be
         defined for both PAGE 0 and PAGE 1.
         Doing so will result in corruption of program
         and/or data.

         Contiguous SARAM memory blocks or flash sectors can be
         be combined if required to create a larger memory block.
*/

MEMORY
{
PAGE 0:    /* Program Memory */
           /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE1 for data allocation */

   //RAMM0       : origin = 0x000050, length = 0x0003B0     /* on-chip RAM block M0 */

   OTP         : origin = 0x3D7800, length = 0x000400     /* on-chip OTP */
   //FLASHB      : origin = 0x3F6000, length = 0x001000     /* on-chip FLASH */
   //FLASHA      : origin = 0x3F7000, length = 0x000F80     /* on-chip FLASH */
   FLASH      : origin = 0x3F6000, length = 0x001F80     /* on-chip FLASH */
   CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   BEGIN       : origin = 0x3F7FF6, length = 0x000002     /* Part of FLASHA.  Used for "boot to Flash" bootloader mode. */
   CSM_PWL_P0  : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */
   IQTABLES    : origin = 0x3FE000, length = 0x000B50     /* IQ Math Tables in Boot ROM */
   IQTABLES2   : origin = 0x3FEB50, length = 0x00008C     /* IQ Math Tables in Boot ROM */
   IQTABLES3   : origin = 0x3FEBDC, length = 0x0000AA      /* IQ Math Tables in Boot ROM */
   ROM         : origin = 0x3FF27C, length = 0x000D44     /* Boot ROM */
   RESET       : origin = 0x3FFFC0, length = 0x000002     /* part of boot ROM  */
   VECTORS     : origin = 0x3FFFC2, length = 0x00003E     /* part of boot ROM  */

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
//-----------------------------------------------------------
/*
   //RAMM0      : origin = 0x000050, length = 0x0003B0
   RAM_M01_2k     : origin = 0x000002, length = 0x000800
   //RAML0      : origin = 0x008000, length = 0x000800
   SARAM_4k     : origin = 0x008000, length = 0x001000
   //DRAML0      : origin = 0x008900, length = 0x000700
*/
//-----------------------------------------------------------
   //OTP         : origin = 0x3D7800, length = 0x000400     /* Part ID register location */
   PARTID      : origin = 0x3D7FFF, length = 0x000001     /* Part ID register location */
   //CSM_PWL     : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations. */

   // FLASH-MODE ONLY !!! - NOT FOR RAM-MODE
   //CSM_RSVD    : origin = 0x3F7F80, length = 0x000076     /* Part of FLASHA.  Program with all 0x0000 when CSM is in use. */
   //CSM_PWL_P0  : origin = 0x3F7FF8, length = 0x000008     /* Part of FLASHA.  CSM password locations in FLASHA */
//-----------------------------------------------------------

PAGE 1 :
   /* Data Memory */
   /* Memory (RAM/FLASH/OTP) blocks can be moved to PAGE0 for program allocation */
   /* Registers remain on PAGE1                                                  */

//-----------------------------------------------------------
   /* RAM info */
   /* SARAM     RAM  0x008000..0x009000  4K       */
   /* M0 vector RAM  0x0000..0x0040      64 byte  */
   /* M0        RAM  0x0000..0x0400      1K-64    */
   /* M1        RAM  0x0400..0x0800      1K       */

   RAML0       : origin = 0x008000, length = 0x001000     /* on-chip RAM block L0 */
   BOOT_RSVD   : origin = 0x000000, length = 0x000050     /* Part of M0, BOOT rom will use this for stack */
   RAMM0       : origin = 0x000050, length = 0x0003B0     /* on-chip RAM block M0 */
   RAMM1       : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */

//-----------------------------------------------------------
   DEV_EMU     : origin = 0x000880, length = 0x000180     /* device emulation registers */
   FLASH_REGS  : origin = 0x000A80, length = 0x000060     /* FLASH registers */
   CSM         : origin = 0x000AE0, length = 0x000010     /* code security module registers */
   ADC_MIRROR  : origin = 0x000B00, length = 0x000010     /* ADC Results register mirror */
   CPU_TIMER0  : origin = 0x000C00, length = 0x000008     /* CPU Timer0 registers */
   CPU_TIMER1  : origin = 0x000C08, length = 0x000008     /* CPU Timer0 registers (CPU Timer1 & Timer2 reserved TI use)*/
   CPU_TIMER2  : origin = 0x000C10, length = 0x000008     /* CPU Timer0 registers (CPU Timer1 & Timer2 reserved TI use)*/
   PIE_CTRL    : origin = 0x000CE0, length = 0x000020     /* PIE control registers */
   PIE_VECT    : origin = 0x000D00, length = 0x000100     /* PIE Vector Table */
   //ECANA       : origin = 0x006000, length = 0x000040     /* eCAN-A control and status registers */
   //ECANA_LAM   : origin = 0x006040, length = 0x000040     /* eCAN-A local acceptance masks */
   //ECANA_MOTS  : origin = 0x006080, length = 0x000040     /* eCAN-A message object time stamps */
   //ECANA_MOTO  : origin = 0x0060C0, length = 0x000040     /* eCAN-A object time-out registers */
   //ECANA_MBOX  : origin = 0x006100, length = 0x000100     /* eCAN-A mailboxes */
   //ECANB       : origin = 0x006200, length = 0x000040     /* eCAN-B control and status registers */
   //ECANB_LAM   : origin = 0x006240, length = 0x000040     /* eCAN-B local acceptance masks */
   //ECANB_MOTS  : origin = 0x006280, length = 0x000040     /* eCAN-B message object time stamps */
   //ECANB_MOTO  : origin = 0x0062C0, length = 0x000040     /* eCAN-B object time-out registers */
   //ECANB_MBOX  : origin = 0x006300, length = 0x000100     /* eCAN-B mailboxes */
   EPWM1       : origin = 0x006800, length = 0x000022     /* Enhanced PWM 1 registers */
   EPWM2       : origin = 0x006840, length = 0x000022     /* Enhanced PWM 2 registers */
   EPWM3       : origin = 0x006880, length = 0x000022     /* Enhanced PWM 3 registers */
   EPWM4       : origin = 0x0068C0, length = 0x000022     /* Enhanced PWM 4 registers */
   EPWM5       : origin = 0x006900, length = 0x000022     /* Enhanced PWM 5 registers */
   EPWM6       : origin = 0x006940, length = 0x000022     /* Enhanced PWM 6 registers */
   //ECAP1       : origin = 0x006A00, length = 0x000020     /* Enhanced Capture 1 registers */
   //ECAP2       : origin = 0x006A20, length = 0x000020     /* Enhanced Capture 2 registers */
   //ECAP3       : origin = 0x006A40, length = 0x000020     /* Enhanced Capture 3 registers */
   //ECAP4       : origin = 0x006A60, length = 0x000020     /* Enhanced Capture 4 registers */
   //EQEP1       : origin = 0x006B00, length = 0x000040     /* Enhanced QEP 1 registers */
   //EQEP2       : origin = 0x006B40, length = 0x000040     /* Enhanced QEP 2 registers */
   GPIOCTRL    : origin = 0x006F80, length = 0x000040     /* GPIO control registers */
   GPIODAT     : origin = 0x006FC0, length = 0x000020     /* GPIO data registers */
   GPIOINT     : origin = 0x006FE0, length = 0x000020     /* GPIO interrupt/LPM registers */
   SYSTEM      : origin = 0x007010, length = 0x000020     /* System control registers */
   SPIA        : origin = 0x007040, length = 0x000010     /* SPI-A registers */
   SCIA        : origin = 0x007050, length = 0x000010     /* SCI-A registers */
   XINTRUPT    : origin = 0x007070, length = 0x000010     /* external interrupt registers */
   ADC         : origin = 0x007100, length = 0x000020     /* ADC registers */
   SPIB        : origin = 0x007740, length = 0x000010     /* SPI-B registers */
   SCIB        : origin = 0x007750, length = 0x000010     /* SCI-B registers */
   SPIC        : origin = 0x007760, length = 0x000010     /* SPI-C registers */
   SPID        : origin = 0x007780, length = 0x000010     /* SPI-D registers */
   //I2CA        : origin = 0x007900, length = 0x000040     /* I2C-A registers */
}

/* Allocate sections to memory blocks.
   Note:
     codestart user defined section in DSP28_CodeStartBranch.asm used to redirect code
               execution when booting to flash
     ramfuncs  user defined section to store functions that will be copied from Flash into RAM
*/

SECTIONS
{

   /* Allocate program areas: */
   codestart         : > BEGIN       PAGE = 0

   ramfuncs          : LOAD = FLASH,
                       RUN = RAMM0,
                       LOAD_START(_RamfuncsLoadStart),
                       LOAD_END(_RamfuncsLoadEnd),
                       RUN_START(_RamfuncsRunStart),
                       PAGE = 1
   { /*
   		--library=Solar_Lib_IQ.lib<CNTL_2P2Z_IQ_ASM.obj>
   		--library=Solar_Lib_IQ.lib<CNTL_3P3Z_IQ_ASM.obj>
   		--library=Solar_Lib_IQ.lib<NOTCH_FLTR_IQ_ASM.obj>
   		--library=IQmath.lib<IQ24div.obj>
   		--library=IQmath.lib<IQ23sin.obj>
		--library=IQmath.lib<IQ23cos.obj>
		--library=IQmath.lib<IQ15rsmpy.obj>
		--library=IQmath.lib<IQ15rmpy.obj>
		--library=IQmath.lib<IQ15div.obj>
		--library=IQmath.lib<IQ4div.obj>
		--library=IQmath.lib<IQ15sqrt.obj>
		--library=IQmath.lib<IQ15isqrt.obj> */
   }

   //ramfuncs        : >> SARAM_4k | RAM_M01_2k,     PAGE = 0
   //ramfuncs        : >> RAMM1 | RAML0,     PAGE = 1

   //.cinit          : >  FLASHA | FLASHB,      PAGE = 0
   //.pinit          : >  FLASHA | FLASHB,      PAGE = 0
   //.text           : >> FLASHA | FLASHB,      PAGE = 0
   .cinit           : >  FLASH,      PAGE = 0
   .pinit           : >  FLASH,      PAGE = 0
   .text            : >> FLASH,      PAGE = 0

   csmpasswds        : > CSM_PWL_P0,  PAGE = 0
   csm_rsvd          : > CSM_RSVD,    PAGE = 0

   /* Allocate uninitalized data sections: */
   .stack           : >  RAMM0,             PAGE = 1
   .ebss            : >> RAMM0 | RAMM1 | RAML0,     PAGE = 1
   .esysmem          : >> RAMM0 | RAMM1 | RAML0,     PAGE = 1
   //.stack          : >  RAM_M01_2k,             PAGE = 0
   //.ebss           : >> SARAM_4k | RAM_M01_2k,     PAGE = 0
   //.esysmem        : >> SARAM_4k | RAM_M01_2k,     PAGE = 0

   /* Initalized sections go in Flash */
   /* For SDFlash to program these, they must be allocated to page 0 */
   //.econst         : >> FLASHA | FLASHB,   PAGE = 0
   //.switch         : >> FLASHA | FLASHB,   PAGE = 0
   .econst           : >> FLASH,   PAGE = 0
   .switch           : >> FLASH,   PAGE = 0

   /* Allocate IQ math areas: */
//   IQmath          : >> FLASHA | FLASHB,   PAGE = 0            /* Math Code */
//   IQmathTables    : >  IQTABLES,          PAGE = 0, TYPE = NOLOAD

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

   /* .reset is a standard section used by the compiler.  It contains the */
   /* the address of the start of _c_int00 for C Code.   /*
   /* When using the boot ROM this section and the CPU vector */
   /* table is not needed.  Thus the default type is set here to  */
   /* DSECT  */
   .reset           : > RESET,      PAGE = 0, TYPE = DSECT
   vectors           : > VECTORS     PAGE = 0, TYPE = DSECT


   PieVectTableFile : > PIE_VECT,   PAGE = 1

/*** Peripheral Frame 0 Register Structures ***/
   DevEmuRegsFile    : > DEV_EMU,     PAGE = 1
   FlashRegsFile     : > FLASH_REGS,  PAGE = 1
   CsmRegsFile       : > CSM,         PAGE = 1
   AdcMirrorFile     : > ADC_MIRROR,  PAGE = 1
   CpuTimer0RegsFile : > CPU_TIMER0,  PAGE = 1
   CpuTimer1RegsFile : > CPU_TIMER1,  PAGE = 1
   CpuTimer2RegsFile : > CPU_TIMER2,  PAGE = 1
   PieCtrlRegsFile   : > PIE_CTRL,    PAGE = 1

/*** Peripheral Frame 1 Register Structures ***/
   //ECanaRegsFile     : > ECANA,       PAGE = 1
   //ECanaLAMRegsFile  : > ECANA_LAM    PAGE = 1
   //ECanaMboxesFile   : > ECANA_MBOX   PAGE = 1
   //ECanaMOTSRegsFile : > ECANA_MOTS   PAGE = 1
   //ECanaMOTORegsFile : > ECANA_MOTO   PAGE = 1

   //ECanbRegsFile     : > ECANB,       PAGE = 1
   //ECanbLAMRegsFile  : > ECANB_LAM    PAGE = 1
   //ECanbMboxesFile   : > ECANB_MBOX   PAGE = 1
   //ECanbMOTSRegsFile : > ECANB_MOTS   PAGE = 1
   //ECanbMOTORegsFile : > ECANB_MOTO   PAGE = 1

   EPwm1RegsFile     : > EPWM1        PAGE = 0
   EPwm2RegsFile     : > EPWM2        PAGE = 0
   EPwm3RegsFile     : > EPWM3        PAGE = 0
   EPwm4RegsFile     : > EPWM4        PAGE = 0
   EPwm5RegsFile     : > EPWM5        PAGE = 1
   EPwm6RegsFile     : > EPWM6        PAGE = 1

   //ECap1RegsFile     : > ECAP1        PAGE = 1
   //ECap2RegsFile     : > ECAP2        PAGE = 1
   //ECap3RegsFile     : > ECAP3        PAGE = 1
   //ECap4RegsFile     : > ECAP4        PAGE = 1

   //EQep1RegsFile     : > EQEP1        PAGE = 1
   //EQep2RegsFile     : > EQEP2        PAGE = 1

   GpioCtrlRegsFile  : > GPIOCTRL     PAGE = 1
   GpioDataRegsFile  : > GPIODAT      PAGE = 1
   GpioIntRegsFile   : > GPIOINT      PAGE = 1

/*** Peripheral Frame 2 Register Structures ***/
   SysCtrlRegsFile   : > SYSTEM,      PAGE = 1
   SpiaRegsFile      : > SPIA,        PAGE = 1
   SciaRegsFile      : > SCIA,        PAGE = 1
   XIntruptRegsFile  : > XINTRUPT,    PAGE = 1
   AdcRegsFile       : > ADC,         PAGE = 0
   AdcResultFile     : > ADC_RESULT,  PAGE = 0
   SpibRegsFile      : > SPIB,        PAGE = 1
   ScibRegsFile      : > SCIB,        PAGE = 1
   SpicRegsFile      : > SPIC,        PAGE = 1
   SpidRegsFile      : > SPID,        PAGE = 1
   //I2caRegsFile      : > I2CA,        PAGE = 1

/*** Code Security Module Register Structures ***/
   //CsmPwlFile      : > CSM_PWL,     PAGE = 1

}

/*
//===========================================================================
// End of file.
//===========================================================================
*/


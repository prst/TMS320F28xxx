// TI File $Revision: /main/4 $
// Checkin $Date: October 16, 2009   15:43:43 $
//###########################################################################
//
// FILE:    DSP2802x_GlobalVariableDefs.c
//
// TITLE:   DSP2802x Global Variables and Data Section Pragmas.
//
//###########################################################################
// $TI Release: LaunchPad f2802x Support Library v100 $
// $Release Date: Wed Jul 25 10:45:39 CDT 2012 $
//###########################################################################

#include "../../include/F2802x_Device.h"     // DSP2802x Headerfile Include File
#include "../../include/DSP28x_Project.h"

//---------------------------------------------------------------------------
// Define Global Peripheral Variables:
//
//----------------------------------------
#if (1==USE_F28027_ADC)
#ifdef __cplusplus
#pragma DATA_SECTION("AdcRegsFile")
#else
#pragma DATA_SECTION(AdcRegs,"AdcRegsFile");
#endif
volatile struct ADC_REGS AdcRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("AdcResultFile")
#else
#pragma DATA_SECTION(AdcResult,"AdcResultFile");
#endif
volatile struct ADC_RESULT_REGS AdcResult;
#endif //(1==USE_F28027_ADC)

//----------------------------------------
#if (1==USE_F28027_COMP)
#ifdef __cplusplus
#pragma DATA_SECTION("Comp1RegsFile")
#else
#pragma DATA_SECTION(Comp1Regs,"Comp1RegsFile");
#endif
volatile struct COMP_REGS Comp1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("Comp2RegsFile")
#else
#pragma DATA_SECTION(Comp2Regs,"Comp2RegsFile");
#endif
volatile struct COMP_REGS Comp2Regs;
#endif //(1==USE_F28027_COMP)

//----------------------------------------
#if (1==USE_F28027_TIMER)
#ifdef __cplusplus
#pragma DATA_SECTION("CpuTimer0RegsFile")
#else
#pragma DATA_SECTION(CpuTimer0Regs,"CpuTimer0RegsFile");
#endif
volatile struct CPUTIMER_REGS CpuTimer0Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CpuTimer1RegsFile")
#else
#pragma DATA_SECTION(CpuTimer1Regs,"CpuTimer1RegsFile");
#endif
volatile struct CPUTIMER_REGS CpuTimer1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CpuTimer2RegsFile")
#else
#pragma DATA_SECTION(CpuTimer2Regs,"CpuTimer2RegsFile");
#endif
volatile struct CPUTIMER_REGS CpuTimer2Regs;
#endif //(1==USE_F28027_TIMER)

//----------------------------------------
#if (1==USE_F28027_FLASH)
#ifdef __cplusplus
#pragma DATA_SECTION("CsmPwlFile")
#else
#pragma DATA_SECTION(CsmPwl,"CsmPwlFile");
#endif
volatile struct CSM_PWL CsmPwl;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("CsmRegsFile")
#else
#pragma DATA_SECTION(CsmRegs,"CsmRegsFile");
#endif
volatile struct CSM_REGS CsmRegs;
#endif //(1==USE_F28027_FLASH)

//----------------------------------------
#if (1==USE_F28027_DEVEMU)
#ifdef __cplusplus
#pragma DATA_SECTION("DevEmuRegsFile")
#else
#pragma DATA_SECTION(DevEmuRegs,"DevEmuRegsFile");
#endif
volatile struct DEV_EMU_REGS DevEmuRegs;
#endif //(1==USE_F28027_FLASH)

//----------------------------------------
#if (1==USE_F28027_PWM)
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm1RegsFile")
#else
#pragma DATA_SECTION(EPwm1Regs,"EPwm1RegsFile");
#endif
volatile struct EPWM_REGS EPwm1Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm2RegsFile")
#else
#pragma DATA_SECTION(EPwm2Regs,"EPwm2RegsFile");
#endif
volatile struct EPWM_REGS EPwm2Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm3RegsFile")
#else
#pragma DATA_SECTION(EPwm3Regs,"EPwm3RegsFile");
#endif
volatile struct EPWM_REGS EPwm3Regs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EPwm4RegsFile")
#else
#pragma DATA_SECTION(EPwm4Regs,"EPwm4RegsFile");
#endif
volatile struct EPWM_REGS EPwm4Regs;
#endif //(1==USE_F28027_PWM)

//----------------------------------------
#if (1==USE_F28027_CAP)
#ifdef __cplusplus
#pragma DATA_SECTION("ECap1RegsFile")
#else
#pragma DATA_SECTION(ECap1Regs,"ECap1RegsFile");
#endif
volatile struct ECAP_REGS ECap1Regs;
#endif //(1==USE_F28027_CAP)

//----------------------------------------
#if (1==USE_F28027_GPIO)
#ifdef __cplusplus
#pragma DATA_SECTION("GpioCtrlRegsFile")
#else
#pragma DATA_SECTION(GpioCtrlRegs,"GpioCtrlRegsFile");
#endif
volatile struct GPIO_CTRL_REGS GpioCtrlRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("GpioDataRegsFile")
#else
#pragma DATA_SECTION(GpioDataRegs,"GpioDataRegsFile");
#endif
volatile struct GPIO_DATA_REGS GpioDataRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("GpioIntRegsFile")
#else
#pragma DATA_SECTION(GpioIntRegs,"GpioIntRegsFile");
#endif
volatile struct GPIO_INT_REGS GpioIntRegs;
#endif //(1==USE_F28027_GPIO)

//----------------------------------------
#if (1==USE_F28027_I2C)
#ifdef __cplusplus
#pragma DATA_SECTION("I2caRegsFile")
#else
#pragma DATA_SECTION(I2caRegs,"I2caRegsFile");
#endif
volatile struct I2C_REGS I2caRegs;
#endif //(1==USE_F28027_I2C)

//----------------------------------------
#if (1==USE_F28027_NMI)
#ifdef __cplusplus
#pragma DATA_SECTION("NmiIntruptRegsFile")
#else
#pragma DATA_SECTION(NmiIntruptRegs,"NmiIntruptRegsFile");
#endif
volatile struct NMIINTRUPT_REGS NmiIntruptRegs;
#endif //(1==USE_F28027_NMI)

//----------------------------------------
#if (1==USE_F28027_PARTID)
#ifdef __cplusplus
#pragma DATA_SECTION("PartIdRegsFile")
#else
#pragma DATA_SECTION(PartIdRegs,"PartIdRegsFile");
#endif
volatile struct PARTID_REGS PartIdRegs;
#endif //(1==USE_F28027_PARTID)

//----------------------------------------
#if (1==USE_F28027_PIE)
#ifdef __cplusplus
#pragma DATA_SECTION("PieCtrlRegsFile")
#else
#pragma DATA_SECTION(PieCtrlRegs,"PieCtrlRegsFile");
#endif
volatile struct PIE_CTRL_REGS PieCtrlRegs;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("PieVectTableFile")
#else
#pragma DATA_SECTION(PieVectTable,"PieVectTableFile");
#endif
struct PIE_VECT_TABLE PieVectTable;
#endif //(1==USE_F28027_PIE)

//----------------------------------------
#if (1==USE_F28027_SCI)
#ifdef __cplusplus
#pragma DATA_SECTION("SciaRegsFile")
#else
#pragma DATA_SECTION(SciaRegs,"SciaRegsFile");
#endif
volatile struct SCI_REGS SciaRegs;
#endif //(1==USE_F28027_SCI)

//----------------------------------------
#if (1==USE_F28027_SPI)
#ifdef __cplusplus
#pragma DATA_SECTION("SpiaRegsFile")
#else
#pragma DATA_SECTION(SpiaRegs,"SpiaRegsFile");
#endif
volatile struct SPI_REGS SpiaRegs;
#endif //(1==USE_F28027_SPI)

//----------------------------------------
#if (1==USE_F28027_SYSCTRL)
#ifdef __cplusplus
#pragma DATA_SECTION("SysCtrlRegsFile")
#else
#pragma DATA_SECTION(SysCtrlRegs,"SysCtrlRegsFile");
#endif
volatile struct SYS_CTRL_REGS SysCtrlRegs;
#endif //(1==USE_F28027_SYSCTRL)

//----------------------------------------
#if (1==USE_F28027_PWR)
#ifdef __cplusplus
#pragma DATA_SECTION("SysPwrCtrlRegsFile")
#else
#pragma DATA_SECTION(SysPwrCtrlRegs,"SysPwrCtrlRegsFile");
#endif
volatile struct SYS_PWR_CTRL_REGS SysPwrCtrlRegs;
#endif //(1==USE_F28027_PWR)


//----------------------------------------
#if (1==USE_F28027_FLASH)
#ifdef __cplusplus
#pragma DATA_SECTION("FlashRegsFile")
#else
#pragma DATA_SECTION(FlashRegs,"FlashRegsFile");
#endif
volatile struct FLASH_REGS FlashRegs;
#endif //(1==USE_F28027_FLASH)

//----------------------------------------
#if (1==USE_F28027_XINTS)
#ifdef __cplusplus
#pragma DATA_SECTION("XIntruptRegsFile")
#else
#pragma DATA_SECTION(XIntruptRegs,"XIntruptRegsFile");
#endif
volatile struct XINTRUPT_REGS XIntruptRegs;
#endif //(1==USE_F28027_XINTS)

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EmuKeyVar");
#else
#pragma DATA_SECTION(EmuKey,"EmuKeyVar");
#endif
Uint16 EmuKey;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("EmuBModeVar");
#else
#pragma DATA_SECTION(EmuBMode,"EmuBModeVar");
#endif
Uint16 EmuBMode;

//----------------------------------------
#ifdef __cplusplus
#pragma DATA_SECTION("FlashScalingVar");
#else
#pragma DATA_SECTION(Flash_CPUScaleFactor, "FlashScalingVar");
#endif
Uint32 Flash_CPUScaleFactor;

//----------------------------------------
#if (1==USE_F28027_FLASH)
#ifdef __cplusplus
#pragma DATA_SECTION("FlashCallbackVar");
#else
#pragma DATA_SECTION(Flash_CallbackPtr, "FlashCallbackVar");
#endif
void (*Flash_CallbackPtr) (void);
#endif //(1==USE_F28027_FLASH)

//===========================================================================
// End of file.
//===========================================================================


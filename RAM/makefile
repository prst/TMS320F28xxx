################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := c:/ti/ccsv6/tools/compiler/c2000_6.2.8

ORDERED_OBJS += \
$(GEN_CMDS__FLAG) \
"./f2802x_headers/source/F2802x_GlobalVariableDefs.obj" \
"./sci_io.obj" \
"./sci.obj" \
"./F28027_EPwmTimerInt.obj" \
"../F2802x_Headers_nonBIOS.cmd" \
"../lib/IQmath.lib" \
"../lib/SFO_TI_Build_V6.lib" \
"../lib/SFO_TI_Build_V6b.lib" \
"../lib/driverlib.lib" \
-l"D:\_PROJECTS\PryStan\2015\LauncPad_F28027F\epwm_timer_interrupts\F2802x_generic_ram.cmd" \
-l"C:\ti\ccsv6\tools\compiler\c2000_6.2.8\lib\rts2800_ml.lib" \
-l"D:\_PROJECTS\PryStan\2015\LauncPad_F28027F\epwm_timer_interrupts\lib\IQmath.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include lib/subdir_vars.mk
-include f2802x_headers/source/subdir_vars.mk
-include subdir_rules.mk
-include lib/subdir_rules.mk
-include f2802x_headers/source/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
Example_F2802xEPwmTimerInt.out \

EXE_OUTPUTS__QUOTED += \
"Example_F2802xEPwmTimerInt.out" \

BIN_OUTPUTS += \
Example_F2802xEPwmTimerInt.hex \

BIN_OUTPUTS__QUOTED += \
"Example_F2802xEPwmTimerInt.hex" \


# All Target
all: Example_F2802xEPwmTimerInt.out secondary-outputs

# Tool invocations
Example_F2802xEPwmTimerInt.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/bin/cl2000" -v28 -ml -mt -g --gcc --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --output_all_syms --cdebug_asm_data -z -m"Example_F2802xEPwmTimerInt.map" --stack_size=0x200 --warn_sections -i"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/lib" -i"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/include" -i"D:/_PROJECTS/PryStan/2015/LauncPad_F28027F/epwm_timer_interrupts" -i"/F2802x_common/lib" -i"D:/libs/math/IQmath/v15c/lib" --reread_libs --verbose_diagnostics --issue_remarks --xml_link_info="Example_F2802xEPwmTimerInt_linkInfo.xml" --entry_point=code_start --rom_model -o "Example_F2802xEPwmTimerInt.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

Example_F2802xEPwmTimerInt.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/bin/hex2000"  -o "Example_F2802xEPwmTimerInt.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "F28027_EPwmTimerInt.pp" "sci.pp" "sci_io.pp" "f2802x_headers\source\F2802x_GlobalVariableDefs.pp" 
	-$(RM) "F28027_EPwmTimerInt.obj" "sci.obj" "sci_io.obj" "f2802x_headers\source\F2802x_GlobalVariableDefs.obj" 
	-@echo 'Finished clean'
	-@echo ' '

secondary-outputs: $(BIN_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets


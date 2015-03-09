################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
f2802x_headers/source/F2802x_GlobalVariableDefs.obj: ../f2802x_headers/source/F2802x_GlobalVariableDefs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/bin/cl2000" -v28 -ml -mt --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.8/include" --include_path="/packages/ti/xdais" --include_path="D:/_PROJECTS/PryStan/2015" --include_path="D:/libs/math/IQmath/v15c/include" -g --gcc --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="f2802x_headers/source/F2802x_GlobalVariableDefs.pp" --obj_directory="f2802x_headers/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
OrbitOLED/OrbitOLEDInterface.obj: D:/workspace_v7/OrbitOLED/OrbitOLEDInterface.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.9.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="D:/workspace_v7" --include_path="J:/ENEL373/workspace_v7" --include_path="F:/workspace_v7" --include_path="E:/workspace_v7" --include_path="D:/workspace_v7/Heli_Project" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178" --include_path="C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.9.LTS/include" --define=ccs="ccs" --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="OrbitOLED/OrbitOLEDInterface.d_raw" --obj_directory="OrbitOLED" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
A51_UPPER_SRCS += \
F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/SILABS_STARTUP.A51 

C_SRCS += \
../src/F380_USB0_Bulk.c \
../src/F3xx_Flash.c \
../src/F3xx_USB0_Descriptor.c \
../src/F3xx_USB0_InterruptServiceRoutine.c \
../src/F3xx_USB0_Standard_Requests.c \
../src/F3xx_USB0_Vendor_Requests.c \
../src/distance_v2_main.c 

OBJS += \
./src/F380_USB0_Bulk.OBJ \
./src/F3xx_Flash.OBJ \
./src/F3xx_USB0_Descriptor.OBJ \
./src/F3xx_USB0_InterruptServiceRoutine.OBJ \
./src/F3xx_USB0_Standard_Requests.OBJ \
./src/F3xx_USB0_Vendor_Requests.OBJ \
./src/SILABS_STARTUP.OBJ \
./src/distance_v2_main.OBJ 


# Each subdirectory must supply rules for building sources it contributes
src/%.OBJ: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Compiler'
	C51 "@$(patsubst %.OBJ,%.__i,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

src/F380_USB0_Bulk.OBJ: C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/c8051f3xx.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Descriptor.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_InterruptServiceRoutine.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Register.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Bulk.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/C8051F380_defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Register_Enums.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/si_toolchain.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdint.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdbool.h

src/F3xx_Flash.OBJ: C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/c8051f3xx.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_Flash.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/C8051F380_defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Register_Enums.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/si_toolchain.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdint.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdbool.h

src/F3xx_USB0_Descriptor.OBJ: C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/c8051f3xx.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Descriptor.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_InterruptServiceRoutine.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Register_Enums.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/si_toolchain.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdint.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdbool.h

src/F3xx_USB0_InterruptServiceRoutine.OBJ: C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/c8051f3xx.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Register.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Descriptor.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_InterruptServiceRoutine.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Main.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Bulk.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Standard_Requests.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Vendor_Requests.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/C8051F380_defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Register_Enums.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/si_toolchain.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdint.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdbool.h

src/F3xx_USB0_Standard_Requests.OBJ: C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/c8051f3xx.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Register.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Descriptor.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_InterruptServiceRoutine.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/C8051F380_defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Register_Enums.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/si_toolchain.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdint.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdbool.h

src/F3xx_USB0_Vendor_Requests.OBJ: C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/c8051f3xx.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Register.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Descriptor.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_InterruptServiceRoutine.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Vendor_Requests.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Main.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Bulk.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/C8051F380_defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Register_Enums.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/si_toolchain.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Defs.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdint.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdbool.h

src/SILABS_STARTUP.OBJ: F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/SILABS_STARTUP.A51 src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Keil 8051 Assembler'
	AX51 "@$(patsubst %.OBJ,%.__ia,$@)" || $(RC)
	@echo 'Finished building: $<'
	@echo ' '

src/distance_v2_main.OBJ: F:/SiliconLabs/SimplicityStudio/v5/developer/toolchains/keil_8051/9.60/INC/MATH.H F:/SiliconLabs/SimplicityStudio/v5/developer/toolchains/keil_8051/9.60/INC/STDIO.H F:/SiliconLabs/SimplicityStudio/v5/developer/toolchains/keil_8051/9.60/INC/ABSACC.H F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/compiler_defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/C8051F380_defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/external_copied_files/SI_C8051F380_Defs.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/c8051f3xx.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Register.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Descriptor.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_InterruptServiceRoutine.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Main.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_USB0_Bulk.h C:/Users/52876/SimplicityStudio/v5_workspace/distance_usb_v1/src/F3xx_Flash.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/si_toolchain.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Register_Enums.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdint.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/shared/si8051Base/stdbool.h F:/SiliconLabs/SimplicityStudio/v5/developer/sdks/8051/v4.3.1/Device/C8051F380/inc/SI_C8051F380_Defs.h



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/eeprom.c \
../Src/main.c \
../Src/stm32f0xx_hal_msp.c \
../Src/stm32f0xx_it.c \
../Src/syscalls.c \
../Src/system_stm32f0xx.c \
../Src/usb_device.c \
../Src/usbd_cdc_if.c \
../Src/usbd_conf.c \
../Src/usbd_desc.c 

OBJS += \
./Src/eeprom.o \
./Src/main.o \
./Src/stm32f0xx_hal_msp.o \
./Src/stm32f0xx_it.o \
./Src/syscalls.o \
./Src/system_stm32f0xx.o \
./Src/usb_device.o \
./Src/usbd_cdc_if.o \
./Src/usbd_conf.o \
./Src/usbd_desc.o 

C_DEPS += \
./Src/eeprom.d \
./Src/main.d \
./Src/stm32f0xx_hal_msp.d \
./Src/stm32f0xx_it.d \
./Src/syscalls.d \
./Src/system_stm32f0xx.d \
./Src/usb_device.d \
./Src/usbd_cdc_if.d \
./Src/usbd_conf.d \
./Src/usbd_desc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER -DSTM32F072xB -I"/home/arielo/STM32_Workspace/LPS3005_fv0_1/Inc" -I"/home/arielo/STM32_Workspace/LPS3005_fv0_1/Drivers/STM32F0xx_HAL_Driver/Inc" -I"/home/arielo/STM32_Workspace/LPS3005_fv0_1/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"/home/arielo/STM32_Workspace/LPS3005_fv0_1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/home/arielo/STM32_Workspace/LPS3005_fv0_1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/home/arielo/STM32_Workspace/LPS3005_fv0_1/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"/home/arielo/STM32_Workspace/LPS3005_fv0_1/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



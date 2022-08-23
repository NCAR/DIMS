################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AT30TS74.c \
../Core/Src/GPS.c \
../Core/Src/HR4000.c \
../Core/Src/ISRegisters.c \
../Core/Src/MS5607.c \
../Core/Src/PID.c \
../Core/Src/TMP117.c \
../Core/Src/funcs.c \
../Core/Src/main.c \
../Core/Src/sdfs.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/AT30TS74.o \
./Core/Src/GPS.o \
./Core/Src/HR4000.o \
./Core/Src/ISRegisters.o \
./Core/Src/MS5607.o \
./Core/Src/PID.o \
./Core/Src/TMP117.o \
./Core/Src/funcs.o \
./Core/Src/main.o \
./Core/Src/sdfs.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/AT30TS74.d \
./Core/Src/GPS.d \
./Core/Src/HR4000.d \
./Core/Src/ISRegisters.d \
./Core/Src/MS5607.d \
./Core/Src/PID.d \
./Core/Src/TMP117.d \
./Core/Src/funcs.d \
./Core/Src/main.d \
./Core/Src/sdfs.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F427xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AT30TS74.d ./Core/Src/AT30TS74.o ./Core/Src/GPS.d ./Core/Src/GPS.o ./Core/Src/HR4000.d ./Core/Src/HR4000.o ./Core/Src/ISRegisters.d ./Core/Src/ISRegisters.o ./Core/Src/MS5607.d ./Core/Src/MS5607.o ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/TMP117.d ./Core/Src/TMP117.o ./Core/Src/funcs.d ./Core/Src/funcs.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/sdfs.d ./Core/Src/sdfs.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o

.PHONY: clean-Core-2f-Src


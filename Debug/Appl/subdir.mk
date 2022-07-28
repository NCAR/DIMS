################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Appl/AppTasks.c \
../Appl/CRC.c \
../Appl/D_XCAM.c \
../Appl/EPS.c \
../Appl/HR4000.c \
../Appl/Logging.c \
../Appl/RunTime.c \
../Appl/SD_Handler.c \
../Appl/main.c 

OBJS += \
./Appl/AppTasks.o \
./Appl/CRC.o \
./Appl/D_XCAM.o \
./Appl/EPS.o \
./Appl/HR4000.o \
./Appl/Logging.o \
./Appl/RunTime.o \
./Appl/SD_Handler.o \
./Appl/main.o 

C_DEPS += \
./Appl/AppTasks.d \
./Appl/CRC.d \
./Appl/D_XCAM.d \
./Appl/EPS.d \
./Appl/HR4000.d \
./Appl/Logging.d \
./Appl/RunTime.d \
./Appl/SD_Handler.d \
./Appl/main.d 


# Each subdirectory must supply rules for building sources it contributes
Appl/%.o: ../Appl/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DDEBUG_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F427xx -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FatFs/src" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/CMSIS/Include" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/Others" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/Sensors" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Inc" -I"C:/Users/iguser/workspace/OBC_MITCH/Services" -I"C:/Users/iguser/workspace/OBC_MITCH/Appl"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



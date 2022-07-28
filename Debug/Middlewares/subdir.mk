################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ESTTC.c \
../Middlewares/MCU_Init.c 

OBJS += \
./Middlewares/ESTTC.o \
./Middlewares/MCU_Init.o 

C_DEPS += \
./Middlewares/ESTTC.d \
./Middlewares/MCU_Init.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/%.o: ../Middlewares/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' -DDEBUG_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F427xx -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FatFs/src" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/CMSIS/Include" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/Others" -I"C:/Users/iguser/workspace/OBC_MITCH/Drivers/Sensors" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares" -I"C:/Users/iguser/workspace/OBC_MITCH/Middlewares/Inc" -I"C:/Users/iguser/workspace/OBC_MITCH/Services" -I"C:/Users/iguser/workspace/OBC_MITCH/Appl"  -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



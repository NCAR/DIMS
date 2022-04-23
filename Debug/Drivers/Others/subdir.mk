################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Others/EEPROM_emul.c 

OBJS += \
./Drivers/Others/EEPROM_emul.o 

C_DEPS += \
./Drivers/Others/EEPROM_emul.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Others/%.o: ../Drivers/Others/%.c Drivers/Others/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-D__weak=__attribute__((weak))' -DDEBUG_ENABLED '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F427xx -c -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/CMSIS/Include -I../Drivers/Others -I../Drivers/Sensors -I../Middlewares -I../Middlewares/Inc -I../Services -I../Appl -O0 -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Others

clean-Drivers-2f-Others:
	-$(RM) ./Drivers/Others/EEPROM_emul.d ./Drivers/Others/EEPROM_emul.o

.PHONY: clean-Drivers-2f-Others


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/hwDrivers/sensorDriver.c 

OBJS += \
./Drivers/hwDrivers/sensorDriver.o 

C_DEPS += \
./Drivers/hwDrivers/sensorDriver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/hwDrivers/%.o Drivers/hwDrivers/%.su: ../Drivers/hwDrivers/%.c Drivers/hwDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-hwDrivers

clean-Drivers-2f-hwDrivers:
	-$(RM) ./Drivers/hwDrivers/sensorDriver.d ./Drivers/hwDrivers/sensorDriver.o ./Drivers/hwDrivers/sensorDriver.su

.PHONY: clean-Drivers-2f-hwDrivers


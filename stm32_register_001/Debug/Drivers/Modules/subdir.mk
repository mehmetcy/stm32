################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Modules/buttondriver.c \
../Drivers/Modules/leddriver.c 

OBJS += \
./Drivers/Modules/buttondriver.o \
./Drivers/Modules/leddriver.o 

C_DEPS += \
./Drivers/Modules/buttondriver.d \
./Drivers/Modules/leddriver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Modules/buttondriver.o: ../Drivers/Modules/buttondriver.c Drivers/Modules/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F070xB -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I"D:/stmprojeler/stm32_register_001/Drivers/Modules" -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Modules/buttondriver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Modules/leddriver.o: ../Drivers/Modules/leddriver.c Drivers/Modules/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DSTM32F070xB -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I"D:/stmprojeler/stm32_register_001/Drivers/Modules" -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Modules/leddriver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


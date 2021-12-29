################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lcd/ILI9341_GFX.c \
../Core/Src/lcd/ILI9341_STM32_Driver.c \
../Core/Src/lcd/ILI9341_Touchscreen.c \
../Core/Src/lcd/LCD.c 

OBJS += \
./Core/Src/lcd/ILI9341_GFX.o \
./Core/Src/lcd/ILI9341_STM32_Driver.o \
./Core/Src/lcd/ILI9341_Touchscreen.o \
./Core/Src/lcd/LCD.o 

C_DEPS += \
./Core/Src/lcd/ILI9341_GFX.d \
./Core/Src/lcd/ILI9341_STM32_Driver.d \
./Core/Src/lcd/ILI9341_Touchscreen.d \
./Core/Src/lcd/LCD.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/lcd/%.o: ../Core/Src/lcd/%.c Core/Src/lcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


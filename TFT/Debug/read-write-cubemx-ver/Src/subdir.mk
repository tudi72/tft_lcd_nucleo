################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../read-write-cubemx-ver/Src/fatfs.c \
../read-write-cubemx-ver/Src/main.c \
../read-write-cubemx-ver/Src/stm32f4xx_hal_msp.c \
../read-write-cubemx-ver/Src/stm32f4xx_it.c \
../read-write-cubemx-ver/Src/system_stm32f4xx.c \
../read-write-cubemx-ver/Src/user_diskio.c 

OBJS += \
./read-write-cubemx-ver/Src/fatfs.o \
./read-write-cubemx-ver/Src/main.o \
./read-write-cubemx-ver/Src/stm32f4xx_hal_msp.o \
./read-write-cubemx-ver/Src/stm32f4xx_it.o \
./read-write-cubemx-ver/Src/system_stm32f4xx.o \
./read-write-cubemx-ver/Src/user_diskio.o 

C_DEPS += \
./read-write-cubemx-ver/Src/fatfs.d \
./read-write-cubemx-ver/Src/main.d \
./read-write-cubemx-ver/Src/stm32f4xx_hal_msp.d \
./read-write-cubemx-ver/Src/stm32f4xx_it.d \
./read-write-cubemx-ver/Src/system_stm32f4xx.d \
./read-write-cubemx-ver/Src/user_diskio.d 


# Each subdirectory must supply rules for building sources it contributes
read-write-cubemx-ver/Src/%.o: ../read-write-cubemx-ver/Src/%.c read-write-cubemx-ver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xE -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Tudi/Downloads/myairbridge-V4Sellskf/TFT/read-write-cubemx-ver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"


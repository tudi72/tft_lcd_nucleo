################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../read-write-cubemx-ver/startup_stm32f411xe.s 

OBJS += \
./read-write-cubemx-ver/startup_stm32f411xe.o 

S_DEPS += \
./read-write-cubemx-ver/startup_stm32f411xe.d 


# Each subdirectory must supply rules for building sources it contributes
read-write-cubemx-ver/%.o: ../read-write-cubemx-ver/%.s read-write-cubemx-ver/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"


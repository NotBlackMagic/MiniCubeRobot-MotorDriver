################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/HAL/adc.c \
../src/HAL/gpio.c \
../src/HAL/pwm.c \
../src/HAL/rcc.c \
../src/HAL/spi.c \
../src/HAL/stopwatch.c \
../src/HAL/timer.c \
../src/HAL/uart.c 

OBJS += \
./src/HAL/adc.o \
./src/HAL/gpio.o \
./src/HAL/pwm.o \
./src/HAL/rcc.o \
./src/HAL/spi.o \
./src/HAL/stopwatch.o \
./src/HAL/timer.o \
./src/HAL/uart.o 

C_DEPS += \
./src/HAL/adc.d \
./src/HAL/gpio.d \
./src/HAL/pwm.d \
./src/HAL/rcc.d \
./src/HAL/spi.d \
./src/HAL/stopwatch.d \
./src/HAL/timer.d \
./src/HAL/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/HAL/%.o: ../src/HAL/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DSTM32F103xB -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1xx" -I"../include/HAL" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



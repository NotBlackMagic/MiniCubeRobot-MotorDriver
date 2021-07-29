################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/stm32f1xx/stm32f1xx_ll_adc.c \
../system/src/stm32f1xx/stm32f1xx_ll_crc.c \
../system/src/stm32f1xx/stm32f1xx_ll_dac.c \
../system/src/stm32f1xx/stm32f1xx_ll_dma.c \
../system/src/stm32f1xx/stm32f1xx_ll_exti.c \
../system/src/stm32f1xx/stm32f1xx_ll_gpio.c \
../system/src/stm32f1xx/stm32f1xx_ll_i2c.c \
../system/src/stm32f1xx/stm32f1xx_ll_pwr.c \
../system/src/stm32f1xx/stm32f1xx_ll_rcc.c \
../system/src/stm32f1xx/stm32f1xx_ll_rtc.c \
../system/src/stm32f1xx/stm32f1xx_ll_spi.c \
../system/src/stm32f1xx/stm32f1xx_ll_tim.c \
../system/src/stm32f1xx/stm32f1xx_ll_usart.c \
../system/src/stm32f1xx/stm32f1xx_ll_utils.c 

OBJS += \
./system/src/stm32f1xx/stm32f1xx_ll_adc.o \
./system/src/stm32f1xx/stm32f1xx_ll_crc.o \
./system/src/stm32f1xx/stm32f1xx_ll_dac.o \
./system/src/stm32f1xx/stm32f1xx_ll_dma.o \
./system/src/stm32f1xx/stm32f1xx_ll_exti.o \
./system/src/stm32f1xx/stm32f1xx_ll_gpio.o \
./system/src/stm32f1xx/stm32f1xx_ll_i2c.o \
./system/src/stm32f1xx/stm32f1xx_ll_pwr.o \
./system/src/stm32f1xx/stm32f1xx_ll_rcc.o \
./system/src/stm32f1xx/stm32f1xx_ll_rtc.o \
./system/src/stm32f1xx/stm32f1xx_ll_spi.o \
./system/src/stm32f1xx/stm32f1xx_ll_tim.o \
./system/src/stm32f1xx/stm32f1xx_ll_usart.o \
./system/src/stm32f1xx/stm32f1xx_ll_utils.o 

C_DEPS += \
./system/src/stm32f1xx/stm32f1xx_ll_adc.d \
./system/src/stm32f1xx/stm32f1xx_ll_crc.d \
./system/src/stm32f1xx/stm32f1xx_ll_dac.d \
./system/src/stm32f1xx/stm32f1xx_ll_dma.d \
./system/src/stm32f1xx/stm32f1xx_ll_exti.d \
./system/src/stm32f1xx/stm32f1xx_ll_gpio.d \
./system/src/stm32f1xx/stm32f1xx_ll_i2c.d \
./system/src/stm32f1xx/stm32f1xx_ll_pwr.d \
./system/src/stm32f1xx/stm32f1xx_ll_rcc.d \
./system/src/stm32f1xx/stm32f1xx_ll_rtc.d \
./system/src/stm32f1xx/stm32f1xx_ll_spi.d \
./system/src/stm32f1xx/stm32f1xx_ll_tim.d \
./system/src/stm32f1xx/stm32f1xx_ll_usart.d \
./system/src/stm32f1xx/stm32f1xx_ll_utils.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/stm32f1xx/%.o: ../system/src/stm32f1xx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DTRACE -DSTM32F103xB -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1xx" -I"../include/HAL" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



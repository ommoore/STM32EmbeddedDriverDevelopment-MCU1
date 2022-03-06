################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/ds1307.c \
../bsp/lcd.c 

OBJS += \
./bsp/ds1307.o \
./bsp/lcd.o 

C_DEPS += \
./bsp/ds1307.d \
./bsp/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/%.o: ../bsp/%.c bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Oliver/Desktop/FastBit Embedded Brain Academy/CubeIDE Workspaces/Master Microcontroller and Embedded Driver Development (MCU1)/cortexmx/stm32f446RE_drivers/drivers/inc" -I"C:/Users/Oliver/Desktop/FastBit Embedded Brain Academy/CubeIDE Workspaces/Master Microcontroller and Embedded Driver Development (MCU1)/cortexmx/stm32f446RE_drivers/bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bsp

clean-bsp:
	-$(RM) ./bsp/ds1307.d ./bsp/ds1307.o ./bsp/lcd.d ./bsp/lcd.o

.PHONY: clean-bsp


################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/015rtc_lcd.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/015rtc_lcd.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/015rtc_lcd.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Oliver/Desktop/FastBit Embedded Brain Academy/CubeIDE Workspaces/Master Microcontroller and Embedded Driver Development (MCU1)/cortexmx/stm32f446RE_drivers/drivers/inc" -I"C:/Users/Oliver/Desktop/FastBit Embedded Brain Academy/CubeIDE Workspaces/Master Microcontroller and Embedded Driver Development (MCU1)/cortexmx/stm32f446RE_drivers/bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/015rtc_lcd.d ./Src/015rtc_lcd.o ./Src/syscalls.d ./Src/syscalls.o ./Src/sysmem.d ./Src/sysmem.o

.PHONY: clean-Src


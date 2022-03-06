################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f446xx_gpio_driver.c \
../drivers/src/stm32f446xx_i2c_driver.c \
../drivers/src/stm32f446xx_rcc_driver.c \
../drivers/src/stm32f446xx_spi_driver.c \
../drivers/src/stm32f446xx_usart_driver.c 

OBJS += \
./drivers/src/stm32f446xx_gpio_driver.o \
./drivers/src/stm32f446xx_i2c_driver.o \
./drivers/src/stm32f446xx_rcc_driver.o \
./drivers/src/stm32f446xx_spi_driver.o \
./drivers/src/stm32f446xx_usart_driver.o 

C_DEPS += \
./drivers/src/stm32f446xx_gpio_driver.d \
./drivers/src/stm32f446xx_i2c_driver.d \
./drivers/src/stm32f446xx_rcc_driver.d \
./drivers/src/stm32f446xx_spi_driver.d \
./drivers/src/stm32f446xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Oliver/Desktop/FastBit Embedded Brain Academy/CubeIDE Workspaces/Master Microcontroller and Embedded Driver Development (MCU1)/cortexmx/stm32f446RE_drivers/drivers/inc" -I"C:/Users/Oliver/Desktop/FastBit Embedded Brain Academy/CubeIDE Workspaces/Master Microcontroller and Embedded Driver Development (MCU1)/cortexmx/stm32f446RE_drivers/bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f446xx_gpio_driver.d ./drivers/src/stm32f446xx_gpio_driver.o ./drivers/src/stm32f446xx_i2c_driver.d ./drivers/src/stm32f446xx_i2c_driver.o ./drivers/src/stm32f446xx_rcc_driver.d ./drivers/src/stm32f446xx_rcc_driver.o ./drivers/src/stm32f446xx_spi_driver.d ./drivers/src/stm32f446xx_spi_driver.o ./drivers/src/stm32f446xx_usart_driver.d ./drivers/src/stm32f446xx_usart_driver.o

.PHONY: clean-drivers-2f-src


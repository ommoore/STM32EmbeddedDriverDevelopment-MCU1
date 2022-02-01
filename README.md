Udemy (FastBit Embedded Brain Academy) "Master Microcontroller and Embedded Driver Development (MCU1)" course project workspace. Various course examples as well as a library containing low-level peripheral drivers written for STM32F446RE MCU. Used STM32CubeIDE, STM32CubeMX, Nucleo-F446RE, Saleae Logic Analyzer.

**Detailed course notes can be found in the top-level directory**

_Projects below are listed in alphabetical order (not in chronological order based on the Udemy Course):_

- cortexmx
	- Section05_HelloWorld
	- Section05_HelloWorldSemiHosting
	- Section06_Debug_Tips_and_Tricks
	- Section07_MCU_Memory_Map
	- Section08_MCU_Bus_Interfaces
	- Section09_MCU_Clocks_and_Details
	- Section10_MCU_Clock_Exercise_HSE_Measurement
	- Section10_MCU_Clock_Exercise_HSI_Measurement
	- Section10_MCU_Clock_Tree
	- Section10_MCU_Clock_Tree_Code
	- Section11_12_13_Interrupt_NVIC_GPIO_Volatile_Notes
	- Section14_15_16_17_18_19_GPIO_Programming_Struct_and_Regs
	- stm32f446RE_drivers
		- Src
			- 001LedToggle.c
			- 002LedButton.c
			- 003LedButtonInterrupt.c
			- 004spi_tx_testing.c
			- 005spi_txonly_arduino.c
			- 006spi_cmd_handling.c
			- 007spi_message_rcv_it.c
			- 008i2c_master_tx_testing.c
			- 009i2c_master_rx_testing.c
			- 010i2c_master_rx_testingIT.c
			- 011i2c_slave_tx_string.c
			- 012i2c_slave_tx_string2.c
			- 013uart_tx.c
			- 014uart_case.c
			- 015rtc_lcd.c
		- bsp
			- ds1307.c
			- ds1307.h
 			- lcd.c
			- lcd.h
		- drivers
			- inc
				- stm32f446xx.h
				- stm32f446xx_gpio_driver.h
				- stm32f446xx_i2c_driver.h
				- stm32f446xx_rcc_driver.h
				- stm32f446xx_spi_driver.h
				- stm32f446xx_usart_driver.h
			- src
				- stm32f446xx_gpio_driver.c
				- stm32f446xx_i2c_driver.c
				- stm32f446xx_rcc_driver.c
				- stm32f446xx_spi_driver.c
				- stm32f446xx_usart_driver.c

################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

BUILD_ARTIFACT_NAME := Section08_MCU_Bus_Interfaces
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Section08_MCU_Bus_Interfaces.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Section08_MCU_Bus_Interfaces.list \

OBJCOPY_BIN += \
Section08_MCU_Bus_Interfaces.bin \


# All Target
all: main-build

# Main-build Target
main-build: Section08_MCU_Bus_Interfaces.elf secondary-outputs

# Tool invocations
Section08_MCU_Bus_Interfaces.elf: $(OBJS) $(USER_OBJS) C:\Users\Oliver\Desktop\FastBit\ Embedded\ Brain\ Academy\Master\ Microcontroller\ and\ Embedded\ Driver\ Development\ (MCU1)\cortexmx\Section08_MCU_Bus_Interfaces\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "Section08_MCU_Bus_Interfaces.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Oliver\Desktop\FastBit Embedded Brain Academy\Master Microcontroller and Embedded Driver Development (MCU1)\cortexmx\Section08_MCU_Bus_Interfaces\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Section08_MCU_Bus_Interfaces.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Section08_MCU_Bus_Interfaces.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Section08_MCU_Bus_Interfaces.list"
	@echo 'Finished building: $@'
	@echo ' '

Section08_MCU_Bus_Interfaces.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Section08_MCU_Bus_Interfaces.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets

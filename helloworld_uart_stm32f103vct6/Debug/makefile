################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
helloworld_uart_stm32f103vct6.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
helloworld_uart_stm32f103vct6.list \


# All Target
all: helloworld_uart_stm32f103vct6.elf secondary-outputs

# Tool invocations
helloworld_uart_stm32f103vct6.elf: $(OBJS) $(USER_OBJS) /home/clement/STM32CubeIDE/workspace_1.0.2/helloworld_uart_stm32f103vct6/STM32F103VCTX_FLASH.ld
	arm-none-eabi-gcc -o "helloworld_uart_stm32f103vct6.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"/home/clement/STM32CubeIDE/workspace_1.0.2/helloworld_uart_stm32f103vct6/STM32F103VCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="helloworld_uart_stm32f103vct6.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

helloworld_uart_stm32f103vct6.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "helloworld_uart_stm32f103vct6.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

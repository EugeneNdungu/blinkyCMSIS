################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f103xx_gpio_driver.c 

OBJS += \
./drivers/Src/stm32f103xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32f103xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f103xx_gpio_driver.o: ../drivers/Src/stm32f103xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -c -I../Inc -I"C:/Users/Eugene/Documents/STM32CubeIDE/MCU1-Course/stm32f1xxx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f103xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"


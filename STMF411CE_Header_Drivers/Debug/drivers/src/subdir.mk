################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f411xce_i2c_driver.c \
../drivers/src/stm32f411xce_nrf24L01_driver.c \
../drivers/src/stm32f411xce_spi_driver.c 

OBJS += \
./drivers/src/stm32f411xce_i2c_driver.o \
./drivers/src/stm32f411xce_nrf24L01_driver.o \
./drivers/src/stm32f411xce_spi_driver.o 

C_DEPS += \
./drivers/src/stm32f411xce_i2c_driver.d \
./drivers/src/stm32f411xce_nrf24L01_driver.d \
./drivers/src/stm32f411xce_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411CEUx -c -I../Inc -I"D:/Embedded/STM32F411CEU6-Black-Pill-Projects/STMF411CE_Header_Drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f411xce_i2c_driver.d ./drivers/src/stm32f411xce_i2c_driver.o ./drivers/src/stm32f411xce_i2c_driver.su ./drivers/src/stm32f411xce_nrf24L01_driver.d ./drivers/src/stm32f411xce_nrf24L01_driver.o ./drivers/src/stm32f411xce_nrf24L01_driver.su ./drivers/src/stm32f411xce_spi_driver.d ./drivers/src/stm32f411xce_spi_driver.o ./drivers/src/stm32f411xce_spi_driver.su

.PHONY: clean-drivers-2f-src


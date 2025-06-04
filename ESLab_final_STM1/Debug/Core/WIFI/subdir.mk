################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/WIFI/es_wifi.c \
../Core/WIFI/es_wifi_io.c \
../Core/WIFI/wifi.c 

OBJS += \
./Core/WIFI/es_wifi.o \
./Core/WIFI/es_wifi_io.o \
./Core/WIFI/wifi.o 

C_DEPS += \
./Core/WIFI/es_wifi.d \
./Core/WIFI/es_wifi_io.d \
./Core/WIFI/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/WIFI/%.o Core/WIFI/%.su Core/WIFI/%.cyclo: ../Core/WIFI/%.c Core/WIFI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32L475xx -DDEBUG -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-WIFI

clean-Core-2f-WIFI:
	-$(RM) ./Core/WIFI/es_wifi.cyclo ./Core/WIFI/es_wifi.d ./Core/WIFI/es_wifi.o ./Core/WIFI/es_wifi.su ./Core/WIFI/es_wifi_io.cyclo ./Core/WIFI/es_wifi_io.d ./Core/WIFI/es_wifi_io.o ./Core/WIFI/es_wifi_io.su ./Core/WIFI/wifi.cyclo ./Core/WIFI/wifi.d ./Core/WIFI/wifi.o ./Core/WIFI/wifi.su

.PHONY: clean-Core-2f-WIFI


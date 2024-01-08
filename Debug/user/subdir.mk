################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user/at32f403a_407_clock.c \
../user/at32f403a_407_int.c \
../user/main.c \
../user/touch.c \
../user/xmc_lcd.c 

OBJS += \
./user/at32f403a_407_clock.o \
./user/at32f403a_407_int.o \
./user/main.o \
./user/touch.o \
./user/xmc_lcd.o 

C_DEPS += \
./user/at32f403a_407_clock.d \
./user/at32f403a_407_int.d \
./user/main.d \
./user/touch.d \
./user/xmc_lcd.d 


# Each subdirectory must supply rules for building sources it contributes
user/%.o: ../user/%.c user/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -ffunction-sections  -g -DAT_START_F403A_V1 -DAT32F403AVGT7 -DUSE_STDPERIPH_DRIVER -I"../include" -I"../include/libraries/drivers/inc" -I"../include/libraries/cmsis/cm4/core_support" -I"../include/libraries/cmsis/cm4/device_support" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_initialize_hardware.c \
../src/_write.c \
../src/dwt.c \
../src/main.c \
../src/stm32f4xx_it.c \
../src/tests.c \
../src/utils.c \
../src/wolfson_pi_audio.c 

OBJS += \
./src/_initialize_hardware.o \
./src/_write.o \
./src/dwt.o \
./src/main.o \
./src/stm32f4xx_it.o \
./src/tests.o \
./src/utils.o \
./src/wolfson_pi_audio.o 

C_DEPS += \
./src/_initialize_hardware.d \
./src/_write.d \
./src/dwt.d \
./src/main.d \
./src/stm32f4xx_it.d \
./src/tests.d \
./src/utils.d \
./src/wolfson_pi_audio.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -D__FPU_PRESENT -DARM_MATH_CM4 -DDEBUG -DOS_USE_SEMIHOSTING -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_STDOUT -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4-hal" -I"../system/src/stm32f4_discovery" -I"../system/src/Components" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



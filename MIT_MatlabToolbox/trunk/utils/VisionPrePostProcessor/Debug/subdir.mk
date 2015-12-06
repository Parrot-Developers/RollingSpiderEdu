################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../../../embcode/rsedu_vis_helpers.c \
../image_module.c \
../main_offboard_image.c


OBJS += \
../../../embcode/rsedu_vis_helpers.o \
./image_module.o \
./main_offboard_image.o


C_DEPS += \
../../../embcode/rsedu_vis_helpers.d \
./image_module.d \
./main_offboard_image.d


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -DDELOS -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '



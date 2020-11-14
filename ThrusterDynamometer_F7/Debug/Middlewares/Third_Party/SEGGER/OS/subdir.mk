################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c 

OBJS += \
./Middlewares/Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o 

C_DEPS += \
./Middlewares/Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o: ../Middlewares/Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F767xx -DDEBUG -c -I"E:/Thruster_Dynamometer/ThrusterDynamometer_F7/Middlewares/Third_Party/SEGGER/Config" -I"E:/Thruster_Dynamometer/ThrusterDynamometer_F7/Thruster_Control_Module" -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I"E:/Thruster_Dynamometer/ThrusterDynamometer_F7/Middlewares/Third_Party/SEGGER/SEGGER" -I../Middlewares/Third_Party/LwIP/src/include/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/posix -I"E:/Thruster_Dynamometer/ThrusterDynamometer_F7/Middlewares/Third_Party/FreeRTOS/org/Source/portable/GCC/ARM_CM7/r0p1" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Middlewares/Third_Party/LwIP/system/arch -I../Middlewares/Third_Party/LwIP/src/include -I../LWIP/App -I"E:/Thruster_Dynamometer/ThrusterDynamometer_F7/Middlewares/Third_Party/FreeRTOS/org/Source/include" -I../Core/Inc -I"E:/Thruster_Dynamometer/ThrusterDynamometer_F7/Middlewares/Third_Party/FreeRTOS/org/Source/CMSIS_RTOS" -I"E:/Thruster_Dynamometer/ThrusterDynamometer_F7/Middlewares/Third_Party/SEGGER/OS" -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Drivers/CMSIS/Include -I../LWIP/Target -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/LwIP/src/include/netif -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"


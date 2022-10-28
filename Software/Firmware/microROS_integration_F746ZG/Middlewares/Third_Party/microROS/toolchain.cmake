include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "LwIP")

# Makefile flags
set(ARCH_CPU_FLAGS "-mcpu=cortex-m7  -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/microROS/include\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/include\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/include/private\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/lwip\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/lwip/apps\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/lwip/priv\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/compat/posix\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/netif\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/lwip/prot\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/FreeRTOS/Source/include\" -I\"/media/alejo/DATA/uROS_F7/Drivers/BSP/Components/lan8742\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/compat/posix/net\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/netif/ppp\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/compat/stdc\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include/compat/posix/sys\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/system\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/system/arch\" -I\"/media/alejo/DATA/uROS_F7/LWIP/App\" -I\"/media/alejo/DATA/uROS_F7/Drivers/CMSIS/Include\" -I\"/media/alejo/DATA/uROS_F7/Middlewares/Third_Party/LwIP/src/include\" -I\"/media/alejo/DATA/uROS_F7/Drivers/CMSIS/Device/ST/STM32F7xx/Include\" -I\"/media/alejo/DATA/uROS_F7/LWIP/Target\" -I\"/media/alejo/DATA/uROS_F7/Drivers/STM32F7xx_HAL_Driver/Inc\" -I\"/media/alejo/DATA/uROS_F7/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy\" -I\"/media/alejo/DATA/uROS_F7/Core/Inc\" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb")


set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++)

set(CMAKE_C_FLAGS_INIT "-std=gnu11 ${ARCH_CPU_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${ARCH_CPU_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)


set(__BIG_ENDIAN__ 0)

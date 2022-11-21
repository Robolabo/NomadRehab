include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "custom")

# Makefile flags
set(ARCH_CPU_FLAGS "-mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Drivers/STM32F4xx_HAL_Driver/Inc\" -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Middlewares/Third_Party/FreeRTOS/Source/include\" -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F\" -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2\" -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Drivers/CMSIS/Include\" -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Drivers/CMSIS/Device/ST/STM32F4xx/Include\" -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy\" -I\"/home/alejo/STM32CubeIDE/workspace/micrROS_ingration_F446RE/Core/Inc\" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(ARCH_OPT_FLAGS @ARCH_OPT_FLAGS@)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

set(CMAKE_C_FLAGS_INIT "-std=gnu11 ${ARCH_CPU_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${ARCH_CPU_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)


set(__BIG_ENDIAN__ 0)
include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(PLATFORM_NAME "LwIP")

# Makefile flags
set(ARCH_CPU_FLAGS "-mcpu=cortex-m7  -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F746xx -c -Og -ffunction-sections -fdata-sections -Wall -fstack-usage --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb")


include_directories(SYSTEM
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/microROS/include
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/include
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/Lab-Project-FreeRTOS-POSIX/include/private
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/lwip
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/lwip/apps
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/lwip/priv
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/compat/posix
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/netif
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/lwip/prot
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/FreeRTOS/Source/include
  ${CMAKE_CURRENT_LIST_DIR}/../../../Drivers/BSP/Components/lan8742
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/compat/posix/net
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/netif/ppp
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/compat/stdc
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/system
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/system/arch
  ${CMAKE_CURRENT_LIST_DIR}/../../../LWIP/App
  ${CMAKE_CURRENT_LIST_DIR}/../../../Drivers/CMSIS/Include
  ${CMAKE_CURRENT_LIST_DIR}/../../../Middlewares/Third_Party/LwIP/src/include
  ${CMAKE_CURRENT_LIST_DIR}/../../../Drivers/CMSIS/Device/ST/STM32F7xx/Include
  ${CMAKE_CURRENT_LIST_DIR}/../../../LWIP/Target
  ${CMAKE_CURRENT_LIST_DIR}/../../../Drivers/STM32F7xx_HAL_Driver/Inc
  ${CMAKE_CURRENT_LIST_DIR}/../../../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
  ${CMAKE_CURRENT_LIST_DIR}/../../../Core/Inc
)


set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++)

set(CMAKE_C_FLAGS_INIT "-std=gnu11 ${ARCH_CPU_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${ARCH_CPU_FLAGS} -DCLOCK_MONOTONIC=0" CACHE STRING "" FORCE)


set(__BIG_ENDIAN__ 0)

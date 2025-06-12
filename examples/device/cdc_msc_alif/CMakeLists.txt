include_guard()
cmake_minimum_required(VERSION 3.21)

# Project setup
project(alif_tinyusb_example C CXX ASM)

include(/opt/tinyusb/examples/device/cdc_msc_alif/toolchain.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m55 -mfloat-abi=hard -mlittle-endian -std=c99 -fdata-sections -mcmse -g3 -O0" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-m55 -mfloat-abi=hard -mlittle-endian -fdata-sections -mcmse -g3 -O0" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m55 -mfloat-abi=hard -mlittle-endian -fdata-sections -mcmse -g3 -O0" CACHE STRING "" FORCE)

set(ROOT /opt/tinyusb)
set(CMSIS_ROOT ${ROOT}/lib/CMSIS_6)
set(CMSIS_CORE_INCLUDE ${CMSIS_ROOT}/CMSIS/Core/Include)
set(ALIF_CMSIS ${ROOT}/hw/mcu/alif)
set(LINKER_SCRIPT ${ALIF_CMSIS}/Device/E7/AE722F80F55D5XX/linker_script/GCC/gcc_M55_HP.ld)

# --- Group: TinyUSB Example App CDC-ACM + MSD ---
add_library(Group_TinyUSB_Example_App__CDC_ACM_MSD OBJECT
  ${ROOT}/examples/device/cdc_msc_alif/src/main.c
  ${ROOT}/examples/device/cdc_msc_alif/src/msc_disk.c
  ${ROOT}/examples/device/cdc_msc_alif/src/usb_descriptors.c
)
target_include_directories(Group_TinyUSB_Example_App__CDC_ACM_MSD PUBLIC
  ${ROOT}/examples/device/cdc_msc_alif/src/
  ${ROOT}/src
  ${ROOT}/hw
)

target_compile_definitions(Group_TinyUSB_Example_App__CDC_ACM_MSD PUBLIC
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)

target_compile_options(Group_TinyUSB_Example_App__CDC_ACM_MSD PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)

# --- Group: Board ---
add_library(Group_Board OBJECT
  ${ROOT}/hw/bsp/board.c
  ${ROOT}/hw/bsp/alif/family.c
)
target_include_directories(Group_Board PUBLIC
  ${ROOT}/hw/bsp/alif/
  ${ROOT}/hw/bsp/alif/boards/alif_e7_dk
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/include
  ${CMSIS_CORE_INCLUDE}
  ${ROOT}/hw/mcu/alif/Device/common/include/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/config
  ${ROOT}/hw/mcu/alif/Device/E7/AE722F80F55D5XX/
  ${ROOT}/hw/
  ${ROOT}/src/
  ${ROOT}/examples/device/cdc_msc_alif/src
)

target_compile_definitions(Group_Board PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(Group_Board PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)

# --- Group: TinyUSB Alif Port ---
add_library(Group_TinyUSB_Alif_Port OBJECT
  ${ROOT}/src/portable/alif/alif_e7_dk/dcd_ensemble.c
)
target_include_directories(Group_TinyUSB_Alif_Port PUBLIC
  ${ROOT}/src/portable/alif/alif_e7_dk/
  ${ROOT}/hw/
  ${ROOT}/src/
  ${ROOT}/examples/device/cdc_msc_alif/src
  ${ROOT}/hw/bsp/alif/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/include
  ${CMSIS_CORE_INCLUDE}
  ${ROOT}/hw/mcu/alif/Device/common/include/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/config
  ${ROOT}/hw/mcu/alif/Device/E7/AE722F80F55D5XX/
)

target_compile_definitions(Group_TinyUSB_Alif_Port PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(Group_TinyUSB_Alif_Port PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)



# --- Group: TinyUSB device only ---
file(GLOB GROUP_TINYUSB_DEVICE_ONLY_SOURCES
  ${ROOT}/src/device/*.c
  ${ROOT}/src/class/audio/*.c
  ${ROOT}/src/class/cdc/*.c
  ${ROOT}/src/class/dfu/*.c
  ${ROOT}/src/class/hid/*.c
  ${ROOT}/src/class/midi/*.c
  ${ROOT}/src/class/msc/*.c
  ${ROOT}/src/class/net/*.c
  ${ROOT}/src/class/usbtmc/*.c
  ${ROOT}/src/class/vendor/*.c
  ${ROOT}/src/class/video/*.c
  ${ROOT}/src/host/*.c
  ${ROOT}/src/typec/*.c
  ${ROOT}/src/common/*.c
  ${ROOT}/src/*.c
)
add_library(Group_TinyUSB__device_only_ OBJECT
  ${GROUP_TINYUSB_DEVICE_ONLY_SOURCES}
)
target_include_directories(Group_TinyUSB__device_only_ PUBLIC
  ${ROOT}/src/device/
  ${ROOT}/src/class/
  ${ROOT}/src/portable/alif/alif_e7_dk/
  ${ROOT}/hw/
  ${ROOT}/src/
  ${ROOT}/src/common/
  ${ROOT}/examples/device/cdc_msc_alif/src
  ${ROOT}/hw/bsp/alif/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/include
  ${CMSIS_CORE_INCLUDE}
  ${ROOT}/hw/mcu/alif/Device/common/include/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/config
  ${ROOT}/hw/mcu/alif/Device/E7/AE722F80F55D5XX/
)

target_compile_definitions(Group_TinyUSB__device_only_ PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(Group_TinyUSB__device_only_ PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)


# --- ARM CMSIS Core 6.0.0 INTERFACE ---
add_library(ARM_CMSIS_CORE_6_0_0 INTERFACE)
target_include_directories(ARM_CMSIS_CORE_6_0_0 INTERFACE
  ${CMSIS_CORE_INCLUDE}
)

# --- Alif Semiconductor GPIO Peripheral ---
add_library(AlifSemiconductor_Device_SOC_Peripherals_GPIO_1_3_0 OBJECT
  ${ALIF_CMSIS}/Alif_CMSIS/Source/Driver_GPIO.c
)
target_include_directories(AlifSemiconductor_Device_SOC_Peripherals_GPIO_1_3_0 PUBLIC
  ${ALIF_CMSIS}/Alif_CMSIS/Include
  ${ALIF_CMSIS}/drivers/include
  ${ROOT}/src/portable/alif/alif_e7_dk/
  ${ROOT}/hw/
  ${ROOT}/src/
  ${ROOT}/examples/device/cdc_msc_alif/src
  ${ROOT}/hw/bsp/alif/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/include
  ${CMSIS_CORE_INCLUDE}
  ${ROOT}/hw/mcu/alif/Device/common/include/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/config
  ${ROOT}/hw/mcu/alif/Device/E7/AE722F80F55D5XX/
  ${ROOT}/hw/bsp/alif/boards/alif_e7_dk
)

target_compile_definitions(AlifSemiconductor_Device_SOC_Peripherals_GPIO_1_3_0 PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(AlifSemiconductor_Device_SOC_Peripherals_GPIO_1_3_0 PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)

# --- Alif Semiconductor PINCONF Peripheral ---
add_library(AlifSemiconductor_Device_SOC_Peripherals_PINCONF_1_3_0 OBJECT
  ${ALIF_CMSIS}/drivers/source/pinconf.c
)
target_include_directories(AlifSemiconductor_Device_SOC_Peripherals_PINCONF_1_3_0 PUBLIC
  ${ALIF_CMSIS}/drivers/include
  ${ROOT}/src/portable/alif/alif_e7_dk/
  ${ROOT}/hw/
  ${ROOT}/src/
  ${ROOT}/examples/device/cdc_msc_alif/src
  ${ROOT}/hw/bsp/alif/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/include
  ${CMSIS_CORE_INCLUDE}
  ${ROOT}/hw/mcu/alif/Device/common/include/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/config
  ${ROOT}/hw/mcu/alif/Device/E7/AE722F80F55D5XX/
)

target_compile_definitions(AlifSemiconductor_Device_SOC_Peripherals_PINCONF_1_3_0 PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(AlifSemiconductor_Device_SOC_Peripherals_PINCONF_1_3_0 PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)

# --- Alif Semiconductor USART Peripheral ---
add_library(AlifSemiconductor_Device_SOC_Peripherals_USART_1_3_0 OBJECT
  ${ALIF_CMSIS}/Alif_CMSIS/Source/Driver_USART.c
  ${ALIF_CMSIS}/drivers/source/uart.c
)
target_include_directories(AlifSemiconductor_Device_SOC_Peripherals_USART_1_3_0 PUBLIC
  ${ALIF_CMSIS}/Alif_CMSIS/Include
  ${ALIF_CMSIS}/drivers/include
  ${ROOT}/src/portable/alif/alif_e7_dk/
  ${ROOT}/hw/
  ${ROOT}/src/
  ${ROOT}/examples/device/cdc_msc_alif/src
  ${ROOT}/hw/bsp/alif/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/include
  ${CMSIS_CORE_INCLUDE}
  ${ROOT}/hw/mcu/alif/Device/common/include/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/config
  ${ROOT}/hw/mcu/alif/Device/E7/AE722F80F55D5XX/
  ${ROOT}/hw/bsp/alif/boards/alif_e7_dk
)

target_compile_definitions(AlifSemiconductor_Device_SOC_Peripherals_USART_1_3_0 PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(AlifSemiconductor_Device_SOC_Peripherals_USART_1_3_0 PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)

# --- Alif Semiconductor Device Startup ---
add_library(AlifSemiconductor_Device_Startup_C_Startup_1_3_0 OBJECT
  ${ALIF_CMSIS}/Device/common/source/clk.c
  ${ALIF_CMSIS}/Device/common/source/mpu_M55.c
  ${ALIF_CMSIS}/Device/common/source/system_M55.c
  ${ALIF_CMSIS}/Device/common/source/system_utils.c
  ${ALIF_CMSIS}/Device/common/source/tcm_partition.c
  ${ALIF_CMSIS}/Device/common/source/tgu_M55.c
  ${ALIF_CMSIS}/Device/core/M55_HP/source/startup_M55_HP.c
)
target_include_directories(AlifSemiconductor_Device_Startup_C_Startup_1_3_0 PUBLIC
  ${ALIF_CMSIS}/Device/common/include
  ${ROOT}/src/portable/alif/alif_e7_dk/
  ${ROOT}/hw/
  ${ROOT}/src/
  ${ROOT}/examples/device/cdc_msc_alif/src
  ${ROOT}/hw/bsp/alif/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/include
  ${CMSIS_CORE_INCLUDE}
  ${ROOT}/hw/mcu/alif/Device/common/include/
  ${ROOT}/hw/mcu/alif/Device/core/M55_HP/config
  ${ROOT}/hw/mcu/alif/Device/E7/AE722F80F55D5XX/
  ${ROOT}/hw/bsp/alif/boards/alif_e7_dk
)


target_compile_definitions(AlifSemiconductor_Device_Startup_C_Startup_1_3_0 PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(AlifSemiconductor_Device_Startup_C_Startup_1_3_0 PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)


# --- Executable (final firmware image) ---
add_executable(cdc_msc_alif)

# Add all objects to target
# List all object libraries above
set(ALL_LIBS
  Group_TinyUSB_Example_App__CDC_ACM_MSD
  Group_Board
  Group_TinyUSB_Alif_Port
  Group_TinyUSB__device_only_
  ARM_CMSIS_CORE_6_0_0
  AlifSemiconductor_Device_SOC_Peripherals_GPIO_1_3_0
  AlifSemiconductor_Device_SOC_Peripherals_PINCONF_1_3_0
  AlifSemiconductor_Device_SOC_Peripherals_USART_1_3_0
  AlifSemiconductor_Device_Startup_C_Startup_1_3_0
)

target_link_libraries(cdc_msc_alif PRIVATE ${ALL_LIBS})

# --- Linker options and script ---
target_link_options(cdc_msc_alif PRIVATE
  -T${LINKER_SCRIPT}
  --entry=Reset_Handler
  --specs=nosys.specs
  -Wl,-Map=linker.map,--cref,-print-memory-usage,--gc-sections,--no-warn-rwx-segments
)

# --- Compile definitions and options (target-wide, add/adjust as needed) ---
target_compile_definitions(cdc_msc_alif PRIVATE
  BOARD_ALIF_DEVKIT_VARIANT=4
  CFG_TUSB_MCU=OPT_MCU_NONE
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUP_DCD_ENDPOINT_MAX=8
  TUD_OPT_RHPORT=0
  UNICODE
  _UNICODE
  CORE_M55_HP
  _DEBUG
  M55_HP
  _RTE_
)
target_compile_options(cdc_msc_alif PRIVATE
  -mcpu=cortex-m55
  -mfloat-abi=hard
  -mlittle-endian
  -std=c99
  -fdata-sections
  -mcmse
  -g3
  -O0
)

# --- Output files (bin/hex) ---
add_custom_command(TARGET cdc_msc_alif POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:cdc_msc_alif> ${CMAKE_CURRENT_BINARY_DIR}/cdc_msc_alif.bin
)
add_custom_command(TARGET cdc_msc_alif POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:cdc_msc_alif> ${CMAKE_CURRENT_BINARY_DIR}/cdc_msc_alif.hex
)
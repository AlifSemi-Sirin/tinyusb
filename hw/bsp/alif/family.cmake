set(ALIF_CMSIS_DFP ${TOP}/hw/mcu/alif/ensemble-cmsis-dfp)
set(ALIF_BOARDLIB ${TOP}/hw/mcu/alif/boardlib)
set(ALIF_COMMON_APP_UTILS ${TOP}/hw/mcu/alif/common-app-utils)
set(CMSIS_DIR ${TOP}/lib/CMSIS_6)

# CORE variable validation and default
if(NOT DEFINED CORE)
  set(CORE m55_hp)
  message(STATUS "CORE not defined, defaulting to: ${CORE}")
endif()

# Validate CORE value
if(NOT CORE STREQUAL "m55_hp" AND NOT CORE STREQUAL "m55_he")
  message(FATAL_ERROR "The chip is not supported. CORE must be 'm55_hp' or 'm55_he', got: ${CORE}")
endif()

message(STATUS "Using CORE: ${CORE}")

# include board specific, for zephyr BOARD_ALIAS may be used instead
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake OPTIONAL RESULT_VARIABLE board_cmake_included)
message(STATUS "Trying to include board: ${BOARD}, success: ${board_cmake_included}")

if (NOT board_cmake_included)
  include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD_ALIAS}/board.cmake)
  message(STATUS "Fallback to BOARD_ALIAS: ${BOARD_ALIAS}")
endif ()

set(CMAKE_TOOLCHAIN_FILE ${TOP}/examples/build_system/cmake/toolchain/arm_${TOOLCHAIN}.cmake)
message(STATUS "Using toolchain file: ${CMAKE_TOOLCHAIN_FILE}")

#------------------------------------
# Functions
#------------------------------------

function(add_board_target BOARD_TARGET)
  if (TARGET ${BOARD_TARGET})
    return()
  endif ()

  set(LD_FILE_GNU ${ALIF_CMSIS_DFP}/Device/E7/AE722F80F55D5XX/linker_script/GCC/gcc_${MCU_VARIANT}.ld)
  message(STATUS "Setting linker file: ${LD_FILE_GNU}")

  if (NOT DEFINED STARTUP_FILE_${CMAKE_C_COMPILER_ID})
    set(STARTUP_FILE_GNU ${ALIF_CMSIS_DFP}/Device/core/${MCU_VARIANT}/source/startup_${MCU_VARIANT}.c)
    set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})
    message(STATUS "Setting startup file: ${STARTUP_FILE_GNU}")
  endif ()

  add_library(${BOARD_TARGET} STATIC
    ${ALIF_CMSIS_DFP}/Device/common/source/system_utils.c
    ${ALIF_CMSIS_DFP}/Device/common/source/system_M55.c
    ${ALIF_CMSIS_DFP}/Device/common/source/clk.c
    ${ALIF_CMSIS_DFP}/Device/common/source/mpu_M55.c
    ${ALIF_CMSIS_DFP}/Device/common/source/tcm_partition.c
    ${ALIF_CMSIS_DFP}/Device/common/source/tgu_M55.c
    ${ALIF_CMSIS_DFP}/Alif_CMSIS/Source/Driver_GPIO.c
    ${ALIF_CMSIS_DFP}/drivers/source/pinconf.c
    ${ALIF_CMSIS_DFP}/Alif_CMSIS/Source/Driver_USART.c
    ${ALIF_CMSIS_DFP}/drivers/source/uart.c
    ${ALIF_BOARDLIB}/devkit_e1c/board_init.c
    ${ALIF_BOARDLIB}/devkit_gen2/board_init.c
    ${ALIF_BOARDLIB}/appkit_gen2/board_init.c
    ${ALIF_COMMON_APP_UTILS}/logging/uart_tracelib.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )

  target_include_directories(${BOARD_TARGET} PUBLIC
    ${ALIF_CMSIS_DFP}/Alif_CMSIS/Include
    ${ALIF_CMSIS_DFP}/drivers/include
    ${ALIF_CMSIS_DFP}/Device/common/config
    ${ALIF_CMSIS_DFP}/Device/common/include
    ${ALIF_CMSIS_DFP}/Device/E7/AE722F80F55D5XX
    ${ALIF_CMSIS_DFP}/Device/core/${MCU_VARIANT}
    ${ALIF_CMSIS_DFP}/Device/core/${MCU_VARIANT}/include
    ${ALIF_CMSIS_DFP}/Device/core/${MCU_VARIANT}/config
    ${ALIF_CMSIS_DFP}/drivers/include
    ${ALIF_BOARDLIB}
    ${ALIF_COMMON_APP_UTILS}/logging
    ${CMSIS_DIR}/CMSIS/Core/Include
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    )

  # Add core definitions to board target
  if(CORE STREQUAL "m55_hp")
    target_compile_definitions(${BOARD_TARGET} PUBLIC CORE_M55_HP)
    target_compile_definitions(${BOARD_TARGET} PUBLIC M55_HP)
    message(STATUS "Adding CORE_M55_HP to board target ${BOARD_TARGET}")
  elseif(CORE STREQUAL "m55_he")
    target_compile_definitions(${BOARD_TARGET} PUBLIC CORE_M55_HE)
    target_compile_definitions(${BOARD_TARGET} PUBLIC M55_HE)
    message(STATUS "Adding CORE_M55_HE to board target ${BOARD_TARGET}")
  endif()

  update_board(${BOARD_TARGET})

  if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Defining Linker options")

    target_link_options(${BOARD_TARGET} PUBLIC
      "LINKER:--script=${LD_FILE_GNU}"
      --specs=nosys.specs
      -Wl,-Map=linker.map,--cref,-print-memory-usage,--gc-sections,--no-warn-rwx-segments
      )

    target_link_libraries(${BOARD_TARGET} PUBLIC
      -lm -lc -lgcc
      )

    target_compile_options(${BOARD_TARGET} PUBLIC
      -Wno-undef -Wno-strict-prototypes
      )
  endif ()

endfunction()

function(configure_freertos)

  if(CORE STREQUAL "m55_hp")
    add_compile_definitions(
      CORE_M55_HP
      M55_HP
      CDC_STACK_SZIE=CDC_STACK_SIZE
      ) 
  elseif(CORE STREQUAL "m55_he")
    add_compile_definitions(
      CORE_M55_HE
      M55_HE
      CDC_STACK_SZIE=CDC_STACK_SIZE
      ) 
  endif()

  set(FREERTOS_HEAP 4 CACHE STRING "FreeRTOS heap implementation")
  set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/FreeRTOSConfig CACHE STRING "FreeRTOS configuration file")

  # WORKAROUND: Add the board folder to the include path for the entire directory.
  # The FreeRTOS kernel is built as a separate library and does not automatically
  # inherit the board's include path where RTE_Components.h is located.
  # This ensures the kernel compilation can find the required headers.
  # There is not such problem with latest FreeRTOSKernel but 10.5.1 is not working
  # without setting FREERTOS_CONFIG_FILE_DIRECTORY.
  include_directories(
    ${ALIF_CMSIS_DFP}/Alif_CMSIS/Include
    ${ALIF_CMSIS_DFP}/drivers/include
    ${ALIF_CMSIS_DFP}/Device/common/config
    ${ALIF_CMSIS_DFP}/Device/common/include
    ${ALIF_CMSIS_DFP}/Device/E7/AE722F80F55D5XX
    ${ALIF_CMSIS_DFP}/Device/core/${MCU_VARIANT}
    ${ALIF_CMSIS_DFP}/Device/core/${MCU_VARIANT}/include
    ${ALIF_CMSIS_DFP}/Device/core/${MCU_VARIANT}/config
    ${ALIF_CMSIS_DFP}/drivers/include
    ${CMSIS_DIR}/CMSIS/Core/Include
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    )

endfunction()


function(family_configure_example TARGET RTOS)

  if(CORE STREQUAL "m55_hp")
    add_compile_definitions(
      CORE_M55_HP
      M55_HP
    ) 
  elseif(CORE STREQUAL "m55_he")
    add_compile_definitions(
      CORE_M55_HE
      M55_HE
      ) 
  endif()

  # Board target
  if (NOT RTOS STREQUAL zephyr)
    add_board_target(board_${BOARD})
    target_link_libraries(${TARGET} PUBLIC board_${BOARD})
  endif ()

  # Configure FreeRTOS heap if using FreeRTOS
  if (RTOS STREQUAL "freertos")
    configure_freertos()
  endif ()

  target_compile_definitions(${TARGET} PUBLIC
    CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
    TUP_DCD_ENDPOINT_MAX=4
    TUD_OPT_RHPORT=0
    BOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED
    CFG_TUSB_MEM_ALIGN=TU_ATTR_ALIGNED\(32\)
    CFG_TUSB_MEM_SECTION=__attribute__\(\(section\(\"usb_dma_buf\"\)\)\)
    BOARD_ALIF_DEVKIT_VARIANT=4
    UNICODE
    _UNICODE
    _DEBUG
    _RTE_
    )

  family_configure_common(${TARGET} ${RTOS})

  message(STATUS "Configuring family example for TARGET: ${TARGET}, RTOS: ${RTOS}, BOARD: ${BOARD}")

  target_sources(${TARGET} PRIVATE
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/family.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../board.c
    )

  target_include_directories(${TARGET} PUBLIC
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    )

  if (RTOS STREQUAL zephyr AND DEFINED BOARD_ALIAS)
    target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD_ALIAS})
    message(STATUS "Adding BOARD_ALIAS include dir: ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD_ALIAS}")
  endif ()

  family_add_tinyusb(${TARGET} OPT_MCU_NONE)
  target_sources(${TARGET} PRIVATE
    ${TOP}/src/portable/alif/alif_e7_dk/dcd_ensemble.c
    )

  # Workaround for Ensemble
  # Remove -ffunction-sections from global flags for all relevant languages
  foreach(var CMAKE_C_FLAGS CMAKE_CXX_FLAGS CMAKE_ASM_FLAGS)
    if(${var} MATCHES "-ffunction-sections")
      string(REPLACE "-ffunction-sections" "" new_flags "${${var}}")
      set(${var} "${new_flags}" CACHE STRING "Remove -ffunction-sections" FORCE)
    endif()
  endforeach()

endfunction()

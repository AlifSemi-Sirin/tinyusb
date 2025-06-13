include_guard()

set(ALIF_CMSIS ${TOP}/hw/mcu/alif)
set(CMSIS_DIR ${TOP}/lib/CMSIS_6)

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

  set(LD_FILE_GNU ${ALIF_CMSIS}/Device/E7/AE722F80F55D5XX/linker_script/GCC/gcc_${MCU_VARIANT}.ld)
  message(STATUS "Setting linker file: ${LD_FILE_GNU}")

  if (NOT DEFINED STARTUP_FILE_${CMAKE_C_COMPILER_ID})
    set(STARTUP_FILE_GNU ${ALIF_CMSIS}/Device/core/${MCU_VARIANT}/source/startup_${MCU_VARIANT}.c)
    set(STARTUP_FILE_Clang ${STARTUP_FILE_GNU})
    message(STATUS "Setting startup file: ${STARTUP_FILE_GNU}")
  endif ()

  add_library(${BOARD_TARGET} STATIC
    ${ALIF_CMSIS}/Device/common/source/system_utils.c
    ${ALIF_CMSIS}/Device/common/source/system_M55.c
    ${ALIF_CMSIS}/Device/common/source/clk.c
    ${ALIF_CMSIS}/Device/common/source/mpu_M55.c
    ${ALIF_CMSIS}/Device/common/source/tcm_partition.c
    ${ALIF_CMSIS}/Device/common/source/tgu_M55.c
    ${ALIF_CMSIS}/Alif_CMSIS/Source/Driver_GPIO.c
    ${ALIF_CMSIS}/drivers/source/pinconf.c
    ${ALIF_CMSIS}/Alif_CMSIS/Source/Driver_USART.c
    ${ALIF_CMSIS}/drivers/source/uart.c
    ${STARTUP_FILE_${CMAKE_C_COMPILER_ID}}
    )

  target_include_directories(${BOARD_TARGET} PUBLIC
    ${ALIF_CMSIS}/Alif_CMSIS/include
    ${ALIF_CMSIS}/drivers/include
    ${ALIF_CMSIS}/Device/common/config
    ${ALIF_CMSIS}/Device/common/include
    ${ALIF_CMSIS}/Device/E7/AE722F80F55D5XX
    ${ALIF_CMSIS}/Device/core/${MCU_VARIANT}
    ${ALIF_CMSIS}/Device/core/${MCU_VARIANT}/include
    ${ALIF_CMSIS}/Device/core/${MCU_VARIANT}/config
    ${ALIF_CMSIS}/drivers/include
    ${ALIF_CMSIS}/Alif_CMSIS/include
    ${CMSIS_DIR}/CMSIS/Core/Include
    ${TOP}/hw/bsp/alif/boards/alif_e7_dk/
    )

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


function(family_configure_example TARGET RTOS)

  # Board target
  if (NOT RTOS STREQUAL zephyr)
    add_board_target(board_${BOARD})
    target_link_libraries(${TARGET} PUBLIC board_${BOARD})
  endif ()

  target_compile_definitions(${TARGET} PUBLIC
    CFG_TUSB_MCU=OPT_MCU_NONE
    CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
    TUP_DCD_ENDPOINT_MAX=8
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

  if(CORE_M55_HP)
    target_compile_definitions(${TARGET} PUBLIC CORE_M55_HP)
    target_compile_definitions(${TARGET} PUBLIC M55_HP)
    add_compile_definitions(CORE_M55_HP)
    add_compile_definitions(M55_HP)
    message(STATUS "CORE_M55_HP is defined and added")
  endif()
  
  if(CORE_M55_HE)
    target_compile_definitions(${TARGET} PUBLIC CORE_M55_HE)
    target_compile_definitions(${TARGET} PUBLIC M55_HE)
    add_compile_definitions(CORE_M55_HP)
    add_compile_definitions(M55_HE)
    message(STATUS "CORE_M55_HE is defined and added")
  endif()

  # Workaround for Ensemble
  # Remove -ffunction-sections from global flags for all relevant languages
  foreach(var CMAKE_C_FLAGS CMAKE_CXX_FLAGS CMAKE_ASM_FLAGS)
    if(${var} MATCHES "-ffunction-sections")
      string(REPLACE "-ffunction-sections" "" new_flags "${${var}}")
      set(${var} "${new_flags}" CACHE STRING "Remove -ffunction-sections" FORCE)
    endif()
  endforeach()

endfunction()

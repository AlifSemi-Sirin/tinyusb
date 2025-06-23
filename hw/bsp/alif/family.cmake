include_guard()

# Define an option MCU_VARIANT, default to "HP". It controls which CORE_M55_*
# macro will be defined and which TinyUSB MCU option is used.
# set(MCU_VARIANT "HP" CACHE STRING "Select core variant: HE or HP")
# set_property(CACHE MCU_VARIANT PROPERTY STRINGS HE HP)

# Include board specific settings
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake OPTIONAL RESULT_VARIABLE board_cmake_included)
message(>>> "CMAKE_CURRENT_LIST_DIR / BOARD = ${CMAKE_CURRENT_LIST_DIR} / ${BOARD}")
if (NOT board_cmake_included)
  message(FATAL_ERROR "Board CMake not found for BOARD=${BOARD}")
endif()

function(family_configure_example TARGET RTOS)

if(CORE_M55_HP)
  add_compile_definitions(
    CORE_M55_HP
    M55_HP
  ) 
else()
  add_compile_definitions(
    CORE_M55_HE
    M55_HE
    ) 
endif()

# Debug: dump key variables
message(STATUS ">>> CMAKE_CURRENT_LIST_DIR           = ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS ">>> CMAKE_CURRENT_FUNCTION_LIST_DIR  = ${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
message(STATUS ">>> TARGET                           = ${TARGET}")
message(STATUS ">>> BOARD                            = ${BOARD}")
message(STATUS ">>> TOP                              = ${TOP}")
message(STATUS ">>> Selected MCU_VARIANT             = ${MCU_VARIANT}")

# Board target
if (NOT RTOS STREQUAL zephyr)
  add_board_target(board_${BOARD})
  target_link_libraries(${TARGET} PUBLIC board_${BOARD})
endif ()

# USB DMA section configuration for Zephyr builds
target_compile_definitions(${TARGET} PUBLIC
  CFG_TUSB_RHPORT0_MODE=OPT_MODE_DEVICE
  TUD_OPT_RHPORT=0  
  TUP_DCD_ENDPOINT_MAX=8
  BOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED
  CFG_TUSB_MEM_SECTION=__attribute__\(\(section\(\".usb_dma_buf\"\)\)\)
  CFG_TUSB_MEM_ALIGN=TU_ATTR_ALIGNED\(32\)
)

family_configure_common(${TARGET} ${RTOS})

#---------- Port Specific ----------
# These files are built for each example since it depends on example's tusb_config.h
target_sources(${TARGET} PRIVATE
  # BSP
  ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/family.c
  ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../board.c
  # HAL
  # ${HAL_DIR}/common/src/system.c
  )
  
target_include_directories(${TARGET} PUBLIC
  # family, hw, board
  ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
  ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../
  ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
  )
if (RTOS STREQUAL zephyr AND DEFINED BOARD_ALIAS AND NOT BOARD STREQUAL BOARD_ALIAS)
  target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD_ALIAS})
endif ()
 
target_sources(${TARGET} PRIVATE
  ${TOP}/src/portable/alif/alif_e7_dk/dcd_ensemble.c
)

family_add_tinyusb(${TARGET} OPT_MCU_NONE)

# Flashing
family_flash_jlink(${TARGET})

endfunction()

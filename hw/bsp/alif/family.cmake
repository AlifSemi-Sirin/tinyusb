include_guard()

# Define an option MCU_VARIANT, default to "HP". It controls which CORE_M55_*
# macro will be defined and which TinyUSB MCU option is used.
# set(MCU_VARIANT "HP" CACHE STRING "Select core variant: HE or HP")
# set_property(CACHE MCU_VARIANT PROPERTY STRINGS HE HP)

# Set family for tinyusb portable path selection
set(FAMILY_MCUS ALIF_E7 CACHE INTERNAL "")

# Include board specific settings
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake OPTIONAL RESULT_VARIABLE board_cmake_included)
if (NOT board_cmake_included)
  message(FATAL_ERROR "Board CMake not found for BOARD=${BOARD}")
endif()

# add system files
set(HAL_DIR ${TOP}/modules/hal/alif/)


function(family_configure_example TARGET RTOS)

# Debug: dump key variables
message(STATUS ">>> CMAKE_CURRENT_LIST_DIR           = ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS ">>> CMAKE_CURRENT_FUNCTION_LIST_DIR  = ${CMAKE_CURRENT_FUNCTION_LIST_DIR}")
message(STATUS ">>> TARGET                           = ${TARGET}")
message(STATUS ">>> BOARD                            = ${BOARD}")
message(STATUS ">>> TOP                              = ${TOP}")
message(STATUS ">>> Selected MCU_VARIANT             = ${MCU_VARIANT}")
message(STATUS ">>> Selected HAL_DIR                 = ${HAL_DIR}")

# Board target
  if (NOT RTOS STREQUAL zephyr)
    add_board_target(board_${BOARD})
    target_link_libraries(${TARGET} PUBLIC board_${BOARD})
  endif ()

  # family_configure_common(${TARGET} ${RTOS})

  #---------- Port Specific ----------
  # These files are built for each example since it depends on example's tusb_config.h
  target_sources(${TARGET} PRIVATE
    # BSP
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/family.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../board.c
    # HAL
    ${HAL_DIR}/common/src/system.c

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

# Map MCU_VARIANT to compile definitions and TinyUSB option                   #
if (MCU_VARIANT STREQUAL "ENSEMBLE7_HE")
  message(STATUS "Building for CORE_M55_HE")
  add_compile_definitions(CORE_M55_HE)
  family_add_tinyusb(${TARGET} OPT_MCU_ALIF_E7_HE)
  target_sources(${TARGET} PRIVATE
    # ${TOP}/src/portable/alif/alif_e7_dk_rtss_he/dcd_ensemble.c
    )
    
  elseif (MCU_VARIANT STREQUAL "ENSEMBLE7_HP")
    message(STATUS "Building for CORE_M55_HP")
    add_compile_definitions(CORE_M55_HP)

    # Add TinyUSB target and port source
    family_add_tinyusb(${TARGET} OPT_MCU_ALIF_E7_HP)
    target_sources(${TARGET} PRIVATE
      ${TOP}/src/portable/alif/alif_e7_dk_rtss_hp/dcd_ensemble.c
      )
else()
  message(FATAL_ERROR "Unsupported MCU_VARIANT='${MCU_VARIANT}'.")
endif()

  # Flashing
#  family_add_bin_hex(${TARGET})
  family_flash_jlink(${TARGET})
#  family_flash_adafruit_nrfutil(${TARGET})
endfunction()

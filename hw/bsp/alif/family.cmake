include_guard()

# include board specific, for zephyr BOARD_ALIAS may be used instead
include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD}/board.cmake OPTIONAL RESULT_VARIABLE board_cmake_included)
if (NOT board_cmake_included)
  include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD_ALIAS}/board.cmake)
endif ()

#------------------------------------
# Functions
#------------------------------------


function(family_configure_example TARGET RTOS)
  family_configure_common(${TARGET} ${RTOS})

  #---------- Port Specific ----------
  # These files are built for each example since it depends on example's tusb_config.h
  target_sources(${TARGET} PRIVATE
    # BSP
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/family.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../board.c
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}/src/system_utils.c
    )
  target_include_directories(${TARGET} PUBLIC
    # family, hw, board
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../../
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}
    ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}/include
    )

  message(STATUS "============================================================================================================================")
  message(STATUS "RTOS: ${RTOS}")
  message(STATUS "BOARD_ALIAS: ${BOARD_ALIAS}")
  message(STATUS "BOARD: ${BOARD}")
  message(STATUS "============================================================================================================================")

  #if (RTOS STREQUAL zephyr AND DEFINED BOARD_ALIAS AND NOT BOARD STREQUAL BOARD_ALIAS)
  if (RTOS STREQUAL zephyr AND DEFINED BOARD_ALIAS)
    target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD_ALIAS})

    message(STATUS "============================================================================================================================")
    message(STATUS "TARGET: ${TARGET}")
    message(STATUS "Visibility: PUBLIC")
    message(STATUS "Include path: ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD_ALIAS}")
    message(STATUS "CMAKE_CURRENT_FUNCTION_LIST_DIR: $CMAKE_CURRENT_FUNCTION_LIST_DIR")
    message(STATUS "BOARD: ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/boards/${BOARD}")
    message(STATUS "============================================================================================================================")
  endif ()

  # Add TinyUSB target and port source
  family_add_tinyusb(${TARGET} OPT_MCU_NRF5X)
  target_sources(${TARGET} PRIVATE
    ${TOP}/src/portable/alif/alif_e7_dk/dcd_ensemble.c
    )

  if(CORE_M55_HP)
    target_compile_definitions(${TARGET} PUBLIC CORE_M55_HP)
    target_compile_definitions(${TARGET} PUBLIC M55_HP)
  endif()

endfunction()

# set(MCU_VARIANT ENSEMBLE7_HP)
if(CORE_M55_HP)
  set(MCU_VARIANT M55_HP)
else()
  set(MCU_VARIANT M55_HE)
endif()

# Define custom linker script for board
set(BOARD_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/alif_e7.ld)

message(STATUS "Using BOARD_LINKER_SCRIPT: ${BOARD_LINKER_SCRIPT}")

function(update_board TARGET)
endfunction()

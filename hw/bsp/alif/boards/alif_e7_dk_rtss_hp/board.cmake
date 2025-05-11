set(MCU_VARIANT ENSEMBLE7_HP)

# Define custom linker script for board
set(BOARD_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/alif_e7.ld)

message(STATUS "Using BOARD_LINKER_SCRIPT: ${BOARD_LINKER_SCRIPT}")

function(update_board TARGET)
endfunction()




set(CMAKE_SYSTEM_CPU cortex-m55 CACHE INTERNAL "System Processor")

set(CPU Cortex-M55)
set(FPU DP_FPU)
set(DSP DSP)
set(MVE FP_FVE)
set(BYTE_ORDER Little-endian)

if(CORE STREQUAL "m55_hp")
  set(MCU_VARIANT M55_HP)
elseif(CORE STREQUAL "m55_he")
  set(MCU_VARIANT M55_HE)
endif()

function(update_board TARGET)
endfunction()

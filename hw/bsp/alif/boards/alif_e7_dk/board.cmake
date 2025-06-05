set(CMAKE_SYSTEM_CPU cortex-m55 CACHE INTERNAL "System Processor")

set(CPU Cortex-M55)
set(FPU DP_FPU)
set(DSP DSP)
set(MVE FP_FVE)
set(BYTE_ORDER Little-endian)

if(CORE_M55_HP)
  set(MCU_VARIANT M55_HP)
else()
  set(MCU_VARIANT M55_HE)
endif()

function(update_board TARGET)
endfunction()

set(CMAKE_SYSTEM_CPU cortex-m33 CACHE INTERNAL "System Processor")

if(CORE_M55_HP)
  set(MCU_VARIANT M55_HP)
else()
  set(MCU_VARIANT M55_HE)
endif()

function(update_board TARGET)
endfunction()

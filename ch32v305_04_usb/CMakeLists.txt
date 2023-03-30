cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tools")
include(toolchain)

set(PRJ_NAME ch32v305_fw)


project(${PRJ_NAME}
  LANGUAGES ASM C CXX
)


set(EXECUTABLE ${PRJ_NAME}.elf)


file(GLOB SRC_FILES CONFIGURE_DEPENDS
  src/*.c
  src/bsp/*.c
  src/bsp/device/*.c

  src/bsp/startup/startup_ch32v30x_D8C.S
)

file(GLOB_RECURSE SRC_FILES_RECURSE CONFIGURE_DEPENDS
  src/ap/*.c
  src/common/*.c
  src/hw/*.c
  src/lib/ch32v_hal/src/*.c
)

add_executable(${EXECUTABLE} 
  ${SRC_FILES}
  ${SRC_FILES_RECURSE}
)



target_include_directories(${EXECUTABLE} PRIVATE 
  src  
  src/ap
  src/bsp
  src/bsp/device
  src/common 
  src/common/core
  src/common/hw/include 
  src/hw
  src/lib

  src/lib/ch32v_hal/inc

  src/hw/driver/usb
  src/hw/driver/usb/cdc
  )

target_compile_definitions(${EXECUTABLE} PRIVATE
  -DCH32V30x_D8C 
  )

target_compile_options(${EXECUTABLE} PRIVATE
  -march=rv32imafc
  -mabi=ilp32f
  -msmall-data-limit=8
  -mno-save-restore
  
  -fmessage-length=0 
  -fsigned-char

  -fdata-sections
  -ffunction-sections

  -Wunused -Wuninitialized

  -std=gnu99 -MMD -MP

  #-save-temps -fverbose-asm  # make asm file
  
  -Wall
  -g3
  -O0
  )

target_link_options(${EXECUTABLE} PRIVATE
  -T../src/bsp/ldscript/ch32v305.ld

  -march=rv32imafc
  -mabi=ilp32f
  -msmall-data-limit=8
  -mno-save-restore
  
  -fmessage-length=0 
  -fsigned-char

  -fdata-sections
  -ffunction-sections

  -Wunused -Wuninitialized
  
  --specs=nano.specs 
  --specs=nosys.specs
  
  -nostartfiles
  
  -lc
  -lm
  -lstdc++
  -lsupc++
  -ffreestanding

  -Wl,-Map=${PRJ_NAME}.map,--cref
  -Wl,--gc-sections
  -Xlinker -print-memory-usage -Xlinker
  )

  add_custom_command(TARGET ${EXECUTABLE} 
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -O binary ${EXECUTABLE} ${PROJECT_NAME}.bin
    COMMENT "Invoking: Make Binary"
    )
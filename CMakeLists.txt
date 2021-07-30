cmake_minimum_required(VERSION 3.10)

# set the project name
project(test LANGUAGES C ASM)
set(CMAKE_SYSTEM_PROCESSOR "rv32imac")

# Set your SDK ROOT here
set(SDK_ROOT "sdk") 
set(DRIVER_ROOT "${SDK_ROOT}/GD32VF103_standard_peripheral")
set(SDK_CORE_ROOT "${SDK_ROOT}/RISCV")
set(SDK_INIT_ROOT "${SDK_CORE_ROOT}/env_Eclipse")
get_filename_component(LINKER_SCRIPT "${SDK_INIT_ROOT}/GD32VF103xB.lds" REALPATH)

set(TARGET main)

# add the executable
add_executable(${TARGET}.elf 
    src/main.c
    src/communication.c
    src/config.c
    src/motor_control.c
    src/pid.c
    
    src/systick.c
    ${DRIVER_ROOT}/system_gd32vf103.c
    ${DRIVER_ROOT}/Source/gd32vf103_gpio.c
    ${DRIVER_ROOT}/Source/gd32vf103_timer.c
    ${DRIVER_ROOT}/Source/gd32vf103_usart.c
    ${DRIVER_ROOT}/Source/gd32vf103_rcu.c
    ${DRIVER_ROOT}/Source/gd32vf103_eclic.c
    ${SDK_CORE_ROOT}/drivers/n200_func.c
    ${SDK_INIT_ROOT}/init.c
    ${SDK_INIT_ROOT}/handlers.c
    ${SDK_INIT_ROOT}/entry.S
    ${SDK_INIT_ROOT}/start.S
) 

target_include_directories(${TARGET}.elf PRIVATE 
    src
    ${DRIVER_ROOT}
    ${DRIVER_ROOT}/Include
    ${SDK_CORE_ROOT}/drivers
)

target_compile_definitions(${TARGET}.elf PRIVATE 
    "GD32VF103V_EVAL"
)

target_compile_options(${TARGET}.elf PRIVATE 
    "-O3"
    "-g"
    "-Wall"
    "-flto"
    "-fshort-enums"
    "-ffunction-sections"
    # "-fdata-sections"
)

target_link_options(${TARGET}.elf PRIVATE 
    "--specs=nosys.specs"
    "--specs=nano.specs"
    "-nostartfiles"
    "-fno-exceptions"
    "-flto"
    "-Xlinker"
    "-T" "${LINKER_SCRIPT}"
    "-Wl,-Map=${TARGET}.map"
    "-Wl,--print-memory-usage"
)

# Post processing command to create a hex file 
add_custom_command(TARGET ${TARGET}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary  ${TARGET}.elf  ${TARGET}.bin
        COMMENT "Invoking: Hexdump")

        
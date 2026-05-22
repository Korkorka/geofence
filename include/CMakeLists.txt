add_library(vectors "lib/vectors.c")

target_link_libraries(vectors PUBLIC pico_stdlib)

target_include_directories(vectors PUBLIC "${CMAKE_CURRENT_LIST_DIR}/lib")

add_library(hardware "lib/hardware.c")

target_link_libraries(hardware PUBLIC 
    pico_stdlib
    hardware_gpio
    hardware_irq
    hardware_uart
    m
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(hardware PUBLIC pico_cyw43_arch_none)
endif()

target_include_directories(hardware PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
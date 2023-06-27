add_library(vesc-uart-pico INTERFACE)

target_sources(vesc-uart-pico
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/src/VescUartPico.cpp
        ${CMAKE_CURRENT_LIST_DIR}src/crc.c
        ${CMAKE_CURRENT_LIST_DIR}src/buffer.c
)

target_include_directories(vesc-uart-pico INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

# Add the standard library to the build
target_link_libraries(vesc-uart-pico
    INTERFACE
        pico_stdlib
        pico_time)

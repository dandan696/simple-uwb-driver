cmake_minimum_required(VERSION 3.0)
project(serial)

include_directories(include)

file(GLOB SOURCES src/*/*.cc)

add_library(serial STATIC
    src/serial.cc
    ${SOURCES}
)

add_executable(main
    src/main.cc
)
target_link_libraries(main serial)


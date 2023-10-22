cmake_minimum_required(VERSION 3.22.1)

set(
    TARGET
    hklx_linux_x86
)
set(
    DIR_HERKULEX
    ..
)

project(${TARGET} VERSION 1.0.1 DESCRIPTION "A C++ SDK for the Herkulex servomotors")

set(
    SOURCES
    ${DIR_HERKULEX}/src/herkulex_sdk/port_handler.cpp
    ${DIR_HERKULEX}/src/herkulex_sdk/packet_manager.cpp
    ${DIR_HERKULEX}/src/herkulex_sdk/servo_herkulex.cpp
)

add_library(
    ${TARGET} SHARED
    ${SOURCES}
)

set_target_properties(${TARGET} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER include/herkulex_sdk/herkulex_sdk.h)

target_include_directories(${TARGET} PRIVATE ${DIR_HERKULEX}/include)

install(TARGETS ${TARGET}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

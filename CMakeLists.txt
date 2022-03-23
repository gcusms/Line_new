cmake_minimum_required(VERSION 3.10)
project(Robot_wolf)

add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED -pthread")

#opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#include the cvui.hpp
include_directories(../)
include_directories(.)

add_executable(${PROJECT_NAME} main.cpp)

#link the opencv packate
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
#fmt
find_package(fmt REQUIRED)
target_link_libraries(${PROJECT_NAME} fmt::fmt)
#Camera
add_library(MVSDK SHARED IMPORTED)
add_library(Camera devices/camera/mv_video_capture.cpp)
set(MIMDVISION_SDK_LIBRARY /lib/libMVSDK.so)
set_property(TARGET MVSDK PROPERTY IMPORTED_LOCATION ${MIMDVISION_SDK_LIBRARY})
target_link_libraries(${PROJECT_NAME} Camera MVSDK)


# new serial
set(serial_SRCS
    devices/serial/src/serial.cc
    devices/serial/src/impl/unix.cc
    devices/serial/src/impl/list_ports/list_ports_linux.cc
    devices/serial/serial/serial.h
    devices/serial/serial/v8stdint.h
)
include_directories(devices/serial)
add_library(serial ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} serial)

set(CONFIG_FILE_PATH ${PROJECT_SOURCE_DIR}/configs)
set(SOURCE_PATH ${PROJECT_SOURCE_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(${PROJECT_NAME} PRIVATE "SOURCE_PATH=\"${PROJECT_SOURCE_DIR}\"")


set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

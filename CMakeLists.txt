cmake_minimum_required(VERSION 3.10)
project(Robot_wolf)

add_executable(${PROJECT_NAME} main.cpp)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED -pthread")

include_directories(
    #OpenVINO推理引擎的头文件
    /home/wolf/intel/openvino_2021/deployment_tools/inference_engine/include/
    /home/wolf/intel/openvino_2021/deployment_tools/ngraph/include/
)

#查找必要的依赖包
find_package(OpenCV REQUIRED)
set(InferenceEngine_DIR "/home/wolf/intel/openvino_2021/deployment_tools/inference_engine/share")
find_package(InferenceEngine)
set(ngraph_DIR "/home/wolf/intel/openvino_2021/deployment_tools/ngraph")
# find_package(ngraph REQUIRED)
set(ngraph_LIBRARIES "/home/wolf/intel/openvino_2021/deployment_tools/ngraph/lib/libngraph.so")
set(ngraph_INCLUDE_DIRS "/home/wolf/intel/openvino_2021/deployment_tools/ngraph/include/")



# 编译detector.so black_box
include_directories(${PROJECT_SOURCE_DIR}/infer)
add_library (detector SHARED ${CMAKE_CURRENT_SOURCE_DIR}/infer/detector.cpp)
target_include_directories(detector
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC ${OpenCV_INCLUDE_DIR}
        PUBLIC ${InferenceEngine_INCLUDE_DIRS}
        # PUBLIC ${ngraph_INCLUDE_DIRS}
)
target_link_libraries(detector
        ${InferenceEngine_LIBRARIES}
        # ${ngraph_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME} 
        PRIVATE
        detector
            )    

#opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#include the cvui.hpp
include_directories(../)
include_directories(.)


#link the opencv packate
target_link_libraries( ${PROJECT_NAME} 
                PRIVATE    
                ${OpenCV_LIBS} )
#fmt
find_package(fmt REQUIRED)
target_link_libraries(${PROJECT_NAME} 
                PRIVATE
                fmt::fmt)
#Camera
add_library(MVSDK SHARED IMPORTED)
add_library(Camera devices/camera/mv_video_capture.cpp)
set(MIMDVISION_SDK_LIBRARY /lib/libMVSDK.so)
set_property(TARGET MVSDK PROPERTY IMPORTED_LOCATION ${MIMDVISION_SDK_LIBRARY})
target_link_libraries(${PROJECT_NAME} 
                PRIVATE
                Camera MVSDK)

#Realsense
find_package(realsense2 REQUIRED)
target_link_libraries(${PROJECT_NAME}  
                        PRIVATE
                        ${realsense2_LIBRARY}
                        )

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
target_link_libraries(${PROJECT_NAME} 
                    PRIVATE
                    serial)

set(CONFIG_FILE_PATH ${PROJECT_SOURCE_DIR}/configs)
set(SOURCE_PATH ${PROJECT_SOURCE_DIR})
target_compile_definitions(${PROJECT_NAME}  PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(${PROJECT_NAME}  PRIVATE "SOURCE_PATH=\"${PROJECT_SOURCE_DIR}\"")


set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)



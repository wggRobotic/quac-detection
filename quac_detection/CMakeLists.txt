cmake_minimum_required(VERSION 3.18)
project(quac_detection LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_QRCODE_DETECTION_SERVER "Build qrcode_detection_server" OFF)
option(BUILD_YOLO_DETECTION_SERVER "Build yolo_detection_server" OFF)

if(NOT BUILD_QRCODE_DETECTION_SERVER AND NOT BUILD_YOLO_DETECTION_SERVER)
  set(BUILD_QRCODE_DETECTION_SERVER ON)
  set(BUILD_YOLO_DETECTION_SERVER ON)
endif()

##################################################
# qrcode_detection_server
##################################################

if(BUILD_QRCODE_DETECTION_SERVER)

  find_package(OpenCV REQUIRED)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(quac_interfaces REQUIRED)

  # quirc
  add_library(quirc STATIC
    quirc/lib/decode.c
    quirc/lib/identify.c
    quirc/lib/quirc.c
    quirc/lib/version_db.c
  )

  add_executable(qrcode_detection_server
    src/detection_server.cpp
    src/qrcode_detection_server.cpp
  )

  target_include_directories(qrcode_detection_server PRIVATE
    ${OPENCV_INCLUDE_DIRS}
    include/
    quirc/lib
  )

  target_link_libraries(qrcode_detection_server
    ${OpenCV_LIBS}
    quirc
  )

  ament_target_dependencies(qrcode_detection_server
    rclcpp
    quac_interfaces
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  install(
    TARGETS qrcode_detection_server
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

##################################################
# yolo_detection_server
##################################################

if(BUILD_YOLO_DETECTION_SERVER)

  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)

  # Jetson Orin = sm_87, Xavier = sm_72
  set(CMAKE_CUDA_ARCHITECTURES "72;87")
  add_compile_definitions(JETSON_PLATFORM)

  if(NOT DEFINED TENSORRT_DIR)
    # Try common TensorRT locations
    set(TENSORRT_SEARCH_PATHS
      "/usr/lib/x86_64-linux-gnu"          # Ubuntu x86 system install
      "/usr/lib/aarch64-linux-gnu"          # Jetson / aarch64 system
      "/usr/local/tensorrt"                 # Manual install
      "$ENV{TENSORRT_DIR}"                  # Environment variable
      "${CMAKE_SOURCE_DIR}/TensorRT"        # Local copy
    )
    foreach(path ${TENSORRT_SEARCH_PATHS})
      if(EXISTS "${path}/libnvinfer.so" OR EXISTS "${path}/lib/libnvinfer.so")
        set(TENSORRT_DIR "${path}")
        break()
      endif()
    endforeach()
  endif()

  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS ${TENSORRT_DIR}/include /usr/include/x86_64-linux-gnu /usr/include/aarch64-linux-gnu /usr/include
  )

  # Find TensorRT libraries
  find_library(NVINFER_LIB nvinfer
    HINTS ${TENSORRT_DIR}/lib ${TENSORRT_DIR} /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
  )
  find_library(NVONNXPARSER_LIB nvonnxparser
    HINTS ${TENSORRT_DIR}/lib ${TENSORRT_DIR} /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
  )

  # CUDA Preprocessing Library
  add_library(cuda_preprocessing STATIC
    YOLOs-CPP-TensorRT/include/yolos/core/cuda_preprocessing.cu
  )
  target_include_directories(cuda_preprocessing PRIVATE
    YOLOs-CPP-TensorRT/include
    ${TENSORRT_INCLUDE_DIR}
  )
  target_link_libraries(cuda_preprocessing PRIVATE
    CUDA::cudart
  )
  set_target_properties(cuda_preprocessing PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    POSITION_INDEPENDENT_CODE ON
  )

  find_package(OpenCV REQUIRED)
  find_package(CUDA REQUIRED)
  find_package(CUDAToolkit REQUIRED)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(quac_interfaces REQUIRED)

  add_executable(yolo_detection_server
    src/detection_server.cpp
    src/yolo_detection_server.cpp
  )

  target_include_directories(yolo_detection_server PRIVATE
    ${CUDA_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
    include/
    YOLOs-CPP-TensorRT/include
  )

  target_link_libraries(yolo_detection_server
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
    nvinfer
    CUDA::cudart
    cuda_preprocessing
  )

  ament_target_dependencies(yolo_detection_server
    rclcpp
    quac_interfaces
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  if(NVONNXPARSER_LIB)
    target_link_libraries(yolo_detection_server ${NVONNXPARSER_LIB})
  endif()

  target_link_options(yolo_detection_server PRIVATE
    -Wl,--allow-shlib-undefined
  )

  install(
    TARGETS yolo_detection_server
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_package()
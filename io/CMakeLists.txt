cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal_protocol.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()



target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)

message(STATUS "Checking ROS2 dependencies:")
message(STATUS "  - ament_cmake: ${ament_cmake_FOUND}")
message(STATUS "  - rclcpp: ${rclcpp_FOUND}")
message(STATUS "  - sp_msgs: ${sp_msgs_FOUND}")
message(STATUS "  - geometry_msgs: ${geometry_msgs_FOUND}")
message(STATUS "  - sensor_msgs: ${sensor_msgs_FOUND}")

# 只要找到了核心的 rclcpp 和 geometry_msgs 就可以尝试编译 nav2aim
if(NOT rclcpp_FOUND OR NOT geometry_msgs_FOUND OR NOT sensor_msgs_FOUND)
    message(WARNING "Core ROS2 dependencies (rclcpp/geometry_msgs/sensor_msgs) not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/aim2nav.cpp ros2/nav2aim.cpp)

    # 设置 ROS2 依赖
    ament_target_dependencies(io rclcpp geometry_msgs sensor_msgs)
endif()

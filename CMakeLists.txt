cmake_minimum_required(VERSION 3.16.3)

project(rm_vision)

enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)
find_package(fastcdr QUIET)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

# The supported application is deliberately limited to the migrated
# Detector -> Tracker -> Solver -> Planner -> Gimbal chain. Historical
# applications remain as source files only and are not CMake targets.
add_executable(QYG_hero_debug src/QYG_hero_debug.cpp)
target_link_libraries(QYG_hero_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim_core tools io)

function(add_qyg_core_test target source)
  add_executable(${target} ${source})
  target_compile_options(${target} PRIVATE -UNDEBUG)
  target_link_libraries(
    ${target}
    ${OpenCV_LIBS} fmt::fmt yaml-cpp Eigen3::Eigen auto_aim_core tools io)
  add_test(NAME ${target} COMMAND ${target} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endfunction()

add_qyg_core_test(gimbal_orientation_test tests/gimbal_orientation_test.cpp)
add_qyg_core_test(solver_camera_to_world_test tests/solver_camera_to_world_test.cpp)
add_qyg_core_test(planner_command_adapter_test tests/planner_command_adapter_test.cpp)
add_qyg_core_test(target_tracking_safety_test tests/target_tracking_safety_test.cpp)

# PlotJuggler receives QYG debug diagnostics through ROS2 when available.
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND)
  target_compile_definitions(QYG_hero_debug PRIVATE QYG_ROS2_DEBUG_TELEMETRY)
  ament_target_dependencies(QYG_hero_debug rclcpp std_msgs)
endif()

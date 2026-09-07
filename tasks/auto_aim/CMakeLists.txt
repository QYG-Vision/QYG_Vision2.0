cmake_minimum_required(VERSION 3.16)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim_core OBJECT
    armor.cpp
    classifier.cpp 
    debug_box_logger.cpp
    debug_overlay.cpp
    detector.cpp
    solver.cpp
    planner_command_adapter.cpp
    target.cpp
    tracker.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim_core io openvino::runtime tinympcstatic)

# Keep the historical CMake target name available for source targets that are
# intentionally excluded from the default build.  The supported hero debug
# executable links auto_aim_core directly and does not compile the old
# Aimer/Shooter/CommandGener decision chain.
add_library(auto_aim ALIAS auto_aim_core)

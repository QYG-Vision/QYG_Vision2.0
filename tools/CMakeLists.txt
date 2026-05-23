cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT 
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    vofa_plotter.cpp
    trajectory.cpp
    recorder.cpp
    csv_logger.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

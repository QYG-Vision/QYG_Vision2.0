#include "../io/ros2/nav2aim.hpp"
#include "../tools/logger.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    io::Nav2Aim nav2aim;
    nav2aim.start();
    
    std::cout << "Starting nav2aim test... Listening to /cmd_vel" << std::endl;

    while (rclcpp::ok()) {
        auto state = nav2aim.get_latest_state();
        std::cout << "Received cmd_vel -> vx: " << state.linear.x
                  << ", vy: " << state.linear.y
                  << ", vz: " << state.linear.z
                  << ", wx: " << state.angular.x
                  << ", wy: " << state.angular.y
                  << ", wz: " << state.angular.z
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return 0;
}
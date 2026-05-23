#include "tasks/nav_pub/get_nav_state.hpp"

#include <chrono>
#include <cstdlib>
#include <thread>

void pub_point()
{
  io::GetNavState nav_state;
  nav_state.start();

  bool goal_sent = false;

  const char * navigate_cmd_home = R"(ros2 action send_goal /red_standard_robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: map},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
    }
  }
}")";
    const char * navigate_cmd_point = R"(ros2 action send_goal /red_standard_robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{
        pose: {
        header: {frame_id: map},
        pose: {
            position: {x: 1.0, y: 0.0, z: 0.0},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        }
        }
    }")";

  while (rclcpp::ok()) {
    if (nav_state.has_received_state()) {
      auto mode = nav_state.get_current_mode();

      if (mode == 0) {
        if (!goal_sent) {
          std::system(navigate_cmd_home);//回家
          goal_sent = true;
        }
      } else if (mode == 1) {
        if (!goal_sent) {
          std::system(navigate_cmd_point);//去占点
          goal_sent = true;
        }
      } else {
        goal_sent = false;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
}
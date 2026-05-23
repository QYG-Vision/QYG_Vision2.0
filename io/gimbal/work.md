通讯流程

1. gimbal接收数据：
    uint8_t current_mode = 0;       // 帧ID：字节2
    float actual_vx = 0.0f;         // x速度：3-6
    float actual_vy = 0.0f;         // y速度：7-10
    float actual_wz = 0.0f;         // z角速度：11-14
    float imu_yaw = 0.0f;           // IMU yaw：15-18（角度制）
    float imu_pitch = 0.0f;         // IMU pitch：19-22（角度制）
    float yaw_angular = 0.0f;       // 偏航角速度：23-26
    float pitch_angular = 0.0f;     // 俯仰角速度：27-30
    float odom_x = 0.0f;            // 里程计：31-34
    uint8_t chassis_state = 0;      // 状态：35
    uint8_t mode = 0;               // 视觉模式：36
    float vyaw = 0.0f;              // 视觉yaw：37-40
    float vpitch = 0.0f;            // 视觉pitch：41-44
    float vroll = 0.0f;             // 视觉roll：45-48
2. 将gimbal数据通过节点topic vision2nav发出到 state_pub_ = this->create_publisher<nav_ec_communication::msg::ChassisState>(
            "/ec2nav_chassis_state", 10);
3.  通过gimbal发送数据给下位机，发送给下位机的数据包含结算出来的开火指令，云台角度以及导航解算出来的云台速度
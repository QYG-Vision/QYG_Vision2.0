#include "io/gimbal/gimbal.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "opencv2/opencv.hpp"

const std::string keys = 
	"{@config-path   | configs/QYG_sentry.yaml}";

int main(int argc ,char* argv[]){

	cv::CommandLineParser cli(argc, argv, keys);
	auto config_Path = cli.get<std::string>("@config-path");

	tools::Exiter exiter;
	io::Gimbal gimbal(config_Path);

	while(!exiter.exit()){
		auto state = gimbal.state();
		tools::logger()->info("yaw: {}, pitch: {}, bullet_speed: {}, yaw_imu: {}, pitch_imu: {}, roll_imu: {}, mode: {}",
			state.yaw, state.pitch, state.bullet_speed, state.yaw_imu, state.pitch_imu, state.roll_imu, state.mode);


		gimbal.send(0,0,0.115,1.151332);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	return 0;
}

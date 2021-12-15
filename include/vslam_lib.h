#include <slamcore/slamcore.hpp>
#include <sw/redis++/redis++.h>

bool init_slam_sdk(sw::redis::Redis* redis, std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam_sys, bool* is_running, std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed);
bool check_map_data(sw::redis::Redis* redis);
void feed_encoder_data(std::string msg, std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed, slamcore::IDT* sample_counter);
int modulo(int a, int b);

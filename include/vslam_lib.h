#include <slamcore/slamcore.hpp>
#include <sw/redis++/redis++.h>

void send_pose_information(sw::redis::Redis* redis, const slamcore::Map2DInterface& map,
              const slamcore::PoseInterface<slamcore::camera_clock>& pose);
bool check_map_data(sw::redis::Redis* redis);
void feed_encoder_data(std::string msg, std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed, slamcore::IDT* sample_counter);
double modulo(double a, double b);
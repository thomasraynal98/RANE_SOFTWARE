#include "global_path_lib.h"

#include <sw/redis++/redis++.h>

std::vector<Pair> format_lidar_data(std::string raw_msg);
bool check_process_LCDS(sw::redis::Redis* redis);
cv::Mat show_local_environnement(cv::Mat grid, std::vector<Pair> data_lidar);
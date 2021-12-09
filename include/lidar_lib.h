#include <string.h>
#include <iostream>
#include "CYdLidar.h"
#include <vector>
#include <sw/redis++/redis++.h>

using namespace sw::redis;

struct Lidar_data
{
    double angle, value;
};

struct Lidar
{
    std::string port;
    CYdLidar * laser;
    bool ret;
    bool is_On{false};

    std::vector<Lidar_data> sample;
};

void publish_lidar_sample(sw::redis::Redis* redis);
#include <lidar_lib.h>
#include <string.h>
#include <iostream>
#include <sw/redis++/redis++.h>

using namespace sw::redis;

void publish_lidar_sample(sw::redis::Redis* redis, Lidar* soyBoy)
{
    std::string sample_string = "";
    for(Lidar_data point : soyBoy->sample)
    {
        sample_string += std::to_string(point.angle) + "," + std::to_string(point.value) + ",";
    }
    redis->publish("raw_data_lidar", sample_string);
}

void recover_raw_lidar_data(std::string raw_lidar_data)
{
    
}
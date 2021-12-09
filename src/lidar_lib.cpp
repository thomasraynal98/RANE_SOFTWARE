#include <lidar_lib.h>
#include <string.h>
#include <iostream>
#include <sw/redis++/redis++.h>

using namespace sw::redis;

void publish_lidar_sample(sw::redis::Redis* redis, Lidar* soyBoy)
{
    redis->publish("brute_lidar", "lidar");
}
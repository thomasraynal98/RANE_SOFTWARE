#include <sw/redis++/redis++.h>
#include <iostream>
#include <chrono>

using namespace sw::redis;

int main()
{
    auto redis = Redis("tcp://127.0.0.1:6379");
    redis.set("key", "val", std::chrono::seconds(10));

    while(true)
    {
        
    }
}
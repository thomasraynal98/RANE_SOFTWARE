#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <slamcore/slamcore.hpp>

#include "vslam_lib.h"

using namespace sw::redis;

/*
    DESCRIPTION: the program will be compute the position of the robot in previous visited map.
*/

auto redis = Redis("tcp://127.0.0.1:6379");
std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam_sys;
std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed; 
slamcore::IDT sample_counter = 0;
std::thread thread_A, thread_B;
bool slam_is_running = false;

void callback_command(std::string channel, std::string msg)
{
    // new encoder input.
    feed_encoder_data(msg, robot_feed, &sample_counter);
}

void function_thread_A()
{
    // THREAD DESCRIPTION: this thread will recover REDIS variable.

    //
    int frequency       = 20;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //

    while(true)
    {
        //
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //

        if(check_map_data(&redis))
        {
            init_slam_sdk(&redis, std::move(slam_sys), &slam_is_running, robot_feed);
        }
    }
}

void function_thread_B()
{
    // THREAD DESCRIPTION: this function will run SLAMCORE SDK.

    while(true)
    {
        if(slam_is_running)
        {
            slam_sys->start();

            while(slam_sys->spinOnce())
            {}
            slam_is_running = false;
        }
        else
        {
            // MAKE PAUSE.
            auto next = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(200);
            std::this_thread::sleep_until(next);
        }
    }
}

int main()
{
    auto sub = redis.subscriber();
    sub.on_message(callback_command);
    sub.subscribe("raw_data_encoder");

    // run thread.
    thread_A = std::thread(&function_thread_A);
    thread_B = std::thread(&function_thread_B);

    thread_A.join();
    thread_B.join();
}
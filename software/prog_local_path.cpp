#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "local_path_lib.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_A, thread_B, thread_C;
bool process_LCDS = false;
bool debug_mode = true;

cv::Mat grid_RGB_1(40, 80, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat grid_Gray_1(40, 80, CV_8UC1, cv::Scalar(255));

std::vector<Pair> lidar_data;

void function_thread_A()
{
    // THREAD DESCRIPTION: it allow to recover the data from redis.
    // (except subscriber: raw_data_lidar)

    ///TIMER//////////////////////////////////////////////////////////////////////////////////////////////////
    int frequency       = 50;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    ///END////////////////////////////////////////////////////////////////////////////////////////////////////

    while(true)
    {
        ///TIMER///////////////////////////////////////////////////////////////////
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        ///END/////////////////////////////////////////////////////////////////////

        process_LCDS = check_process_LCDS(&redis);
    }
}

void function_thread_B(sw::redis::Subscriber* sub)
{
    // THREAD DESCRIPTION: subscribe REDIS.
    while(true)
    {
        sub->consume();
    }
}

void function_thread_C()
{
    // THREAD DESCRIPTION: if it's good it's will compute command for autonomous mode.

    ///TIMER//////////////////////////////////////////////////////////////////////////////////////////////////
    int frequency       = 30;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    ///END////////////////////////////////////////////////////////////////////////////////////////////////////

    while(true)
    {
        ///TIMER///////////////////////////////////////////////////////////////////
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        ///END/////////////////////////////////////////////////////////////////////

        if(debug_mode)
        {   
            show_local_environnement(grid_RGB_1.clone(), lidar_data);
            // cv::namedWindow("Local_env",cv::WINDOW_AUTOSIZE);
            // cv::resize(copy_debug_visual_map, copy_debug_visual_map, cv::Size(0,0),1.0,1.0,cv::INTER_LINEAR);
            // cv::imshow("Local_env", grid_RGB_1);

            // char d=(char)cv::waitKey(25);
            // if(d==27)
            //     break;
        }
    }
}

void callback_command(std::string channel, std::string msg)
{
    // reception lidar data.
    std::cout << "YOOOO\n";
    lidar_data = format_lidar_data(msg);
    std::cout << lidar_data.size() << "\n";
}

int main()
{
    auto sub = redis.subscriber();
    sub.on_message(callback_command);
    sub.subscribe("raw_data_lidar");

    thread_A = std::thread(&function_thread_A);
    thread_B = std::thread(&function_thread_B, &sub);
    thread_C = std::thread(&function_thread_C);
    thread_A.join();
    thread_B.join();
    thread_C.join();
}
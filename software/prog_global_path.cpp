#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_path_lib.h"

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_A, thread_B;
bool is_compute_global_path = false;
cv::Mat current_grid;

void function_thread_A()
{
    // THREAD DESCRIPTION: this thread listen REDIS

    //
    int frequency       = 100;
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

        is_compute_global_path = check_redis_variable(&redis);
        // std::cout << is_compute_global_path << std::endl;

        // check if the map is ready.
        if(check_if_map_is_ready(&redis) && current_grid.empty())
        {
            // std::cout << "yo" << std::endl;
            import_map_png(&redis, &current_grid);
        }

        // // DEBUG:
        // if(true)
        // {
        //     Pair current = get_current_position(&redis);
        //     cv::Mat cloneur = current_grid.clone();
        //     cv::circle(cloneur, cv::Point((int)(current.first),(int)(current.second)),5, cv::Scalar(150), cv::FILLED, 5,0);

        //     cv::namedWindow("Local_env5",cv::WINDOW_AUTOSIZE);
        //     // cv::resize(current_grid, current_grid, cv::Size(0,0),9.0,9.0,6);
        //     // cv::rotate(grid, grid, 1);
        //     cv::imshow("Local_env5", cloneur);
        //     char d=(char)cv::waitKey(25);
        // }
    }
}
void function_thread_B()
{
    // THREAD DESCRIPTION: if compute global path is "true" compute it. 
    //
    int frequency       = 10;
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

        // std::cout << is_compute_global_path << " et " << !current_grid.empty() << std::endl;
        // if(!current_grid.empty()) { std::cout << current_grid.size() << std::endl;}
        if(is_compute_global_path && !current_grid.empty())
        {
            std::cout << "je suis la." << std::endl;
            redis.set("State_global_path_is_computing", "true");
            compute_global_path(&redis, &current_grid);
            redis.set("State_global_path_is_computing", "false");

            auto next = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(100);
            std::this_thread::sleep_until(next);
        }
    }
}

int main()
{
    // run thread.
    thread_A = std::thread(&function_thread_A);
    thread_B = std::thread(&function_thread_B);

    thread_A.join();
    thread_B.join();
}
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
bool debug_mode = false;

cv::Mat grid_RGB_1(40, 80, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat grid_Gray_1(40, 80, CV_8UC1, cv::Scalar(255));

cv::Mat grid_RGB_2(40, 80, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat grid_Gray_2(40, 80, CV_8UC1, cv::Scalar(255));

std::vector<double> position;
std::vector<double> current_speed;
std::vector<Pair> lidar_data;
std::vector<Path_keypoint> global_path;
std::string global_path_msg = "";
Path_keypoint target_keypoint;

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
        get_robot_speed(&redis, &current_speed);

        // download new path if detected.
        if((*(redis.get("State_global_path"))).compare(global_path_msg) != 0)
        {   
            redis.set("State_destination_is_reach", "false");
            std::cout << "COMPUTE" << (*(redis.get("State_global_path"))) << "\n";
            get_global_path(&redis, &global_path);
            global_path_msg = (*(redis.get("State_global_path")));
        }
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
            show_local_environnement(grid_RGB_1.clone(), lidar_data, current_speed);
        }

        // 0#. update keypoint path information.
        update_data(&redis, &global_path, &position);

        if(process_LCDS)
        {
            // 1#. compute target keypoint.
            select_target_keypoint(&global_path, &target_keypoint);

            // 2#. detect if we are arrived.
            // if(/destination_reach(&global_path.back(), position))
            if(true)
            {
                // 3#. project some KP and TKP in LCDS.
                std::vector<Pair> projected_keypoint = project_keypoint_in_lidar_referencial(&global_path, &position, &target_keypoint);
                
                std::cout << "oui" << projected_keypoint.size() << std::endl;
                for(auto p : projected_keypoint){ std::cout << p.first << "," << p.second << "\n";}

                if(true)
                {
                    cv::Mat clone = grid_RGB_2.clone();

                    for(auto data : projected_keypoint)
                    {
                        cv::circle(clone, cv::Point((int)(data.first),(int)(data.second)),0, cv::Scalar(255,0,0), cv::FILLED, 0, 0);
                    }
                    // show.
                    cv::namedWindow("Local_env2",cv::WINDOW_AUTOSIZE);
                    cv::resize(clone, clone, cv::Size(0,0),18.0,18.0,6);
                    // cv::rotate(grid, grid, 1);
                    cv::imshow("Local_env2", clone);

                    char d=(char)cv::waitKey(25);
                    // if(d==27)
                    //     break;
                }

                // 4#. descision

            }
            else
            {
                redis.set("State_destination_is_reach", "true");
            }
        }
    }
}

void callback_command(std::string channel, std::string msg)
{
    // reception lidar data.
    lidar_data = format_lidar_data(msg);
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
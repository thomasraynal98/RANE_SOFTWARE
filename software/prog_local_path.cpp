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
Param_nav navigation_param;

cv::Mat grid_RGB_1(80, 160, CV_8UC3, cv::Scalar(255, 255, 255));
// cv::Mat grid_Gray_1(40, 80, CV_8UC1, cv::Scalar(255));

cv::Mat grid_RGB_2(80, 160, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat grid_Gray_2(80, 160, CV_8UC1, cv::Scalar(255));

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
    int frequency       = 10;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    auto next = std::chrono::high_resolution_clock::now();
    ///END////////////////////////////////////////////////////////////////////////////////////////////////////

    while(true)
    {
        ///TIMER///////////////////////////////////////////////////////////////////
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
            if(get_global_path(&redis, &global_path))
            {
                global_path_msg = (*(redis.get("State_global_path"))); // to compare.
            }
        }

        // download navigation param.
        // if(!navigation_param.get_param) { get_navigation_param(&redis, &navigation_param);}
        get_navigation_param(&redis, &navigation_param);
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
    auto next = std::chrono::high_resolution_clock::now();
    ///END////////////////////////////////////////////////////////////////////////////////////////////////////

    bool destination_is_blocked = false;

    while(true)
    {
        ///TIMER///////////////////////////////////////////////////////////////////
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        next = std::chrono::high_resolution_clock::now();
        ///END/////////////////////////////////////////////////////////////////////

        // if(debug_mode)
        // {   
        //     cv::Mat clone = grid_RGB_1.clone();
        //     show_local_environnement(&clone, &lidar_data, &current_speed);
        // }

        cv::Mat cloned = grid_RGB_2.clone();
        cv::Mat clonedg = grid_Gray_2.clone();

        bool LINEMODE;
        LINEMODE = true;

        std::vector<Pair> FDP;
        compute_new_TKP(&cloned, &FDP, &lidar_data, \
                &clonedg, &redis, &target_keypoint);

        // 0#. update keypoint path information.
        if(!global_path.empty())
        {
            update_data(&redis, &global_path, &position);

            // 1#. detect if we are arrived && if the global path is not computing.
            if(!destination_reach(&global_path.back(), position) && \
            (*(redis.get("State_global_path_is_computing"))).compare("false") == 0 && \
            (*(redis.get("State_destination_is_reach"))).compare("false") == 0 && \
            (*(redis.get("State_is_autonomous"))).compare("true") == 0 && \
            (*(redis.get("State_slamcore"))).compare("OK") == 0)
            {
                // 1#. compute target keypoint.
                select_target_keypoint_2(&global_path, &target_keypoint);

                // 3#. project some KP and TKP in LCDS.
                std::vector<Pair> projected_keypoint; 
                project_keypoint_in_lidar_referencial(&global_path, &position, &target_keypoint, &projected_keypoint);

                // 4#. check if we have problem.
                cv::Mat clone = grid_RGB_2.clone();

                // TODO: change TKP_problem because we change process, we only need to 
                // write lidar sample on clone cv::mat.
                // simulation_problem(2000, &clone, &current_speed, &lidar_data);
                TKP_problem(&clone, &target_keypoint, &lidar_data);

                // 4a#. compute a new TKP based on LCDS.
                cv::Mat clone_gray = grid_Gray_2.clone();
                if(compute_new_TKP(&clone, &projected_keypoint, &lidar_data, \
                &clone_gray, &redis, &target_keypoint))
                {
                    compute_motor_autocommandeNico(&redis, &target_keypoint, 1, \
                    &position, &navigation_param, &LINEMODE);
                }
                else
                {
                    // Check if we destination is blocked du to drift.
                    if(target_keypoint.distance_KPD < 3.0)
                    {
                        if(destination_is_blocked)
                        {
                            redis.set("State_destination_is_reach", "true");
                            destination_is_blocked = false;
                        }
                        else
                        {
                            std::cout << "[DEBUG: destination is blocked]" <<  \
                            target_keypoint.distance_KPD << std::endl;
                            destination_is_blocked = true;
                            next += std::chrono::milliseconds((int)1500);
                            std::this_thread::sleep_until(next);
                        }
                    }
                    else
                    {
                        // This case occurend when the robot is not in the good rotation.
                        compute_motor_autocommandeNico(&redis, &target_keypoint, 0, \
                        &position, &navigation_param, &LINEMODE);
                    }
                }

                // // TODO:DEBUG=============================================================
                // cv::Mat clone3 = grid_RGB_1.clone();
                // for(auto kp : projected_keypoint)
                // {
                //     cv::circle(clone3, cv::Point((int)(kp.first),(int)(kp.second)),0, cv::Scalar(255,0,0), cv::FILLED, 0, 0);
                // }
                // cv::namedWindow("Local_env2",cv::WINDOW_AUTOSIZE);
                // cv::resize(clone3, clone3, cv::Size(0,0),9.0,9.0,6);
                // // cv::rotate(grid, grid, 1);
                // cv::imshow("Local_env2", clone3);
                // TODO:DEBUG=============================================================

                // char d=(char)cv::waitKey(25);

                // 5#. security stop, if lidar data is on 20 cm trajector
                if(security_break(&lidar_data))
                {
                    redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
                    next += std::chrono::milliseconds((int)1000);
                    std::this_thread::sleep_until(next);
                }
            }
            else
            {
                redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
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
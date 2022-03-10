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

    ///TIMER
    int frequency = 10;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    auto next = std::chrono::high_resolution_clock::now();
    ///END
    bool destination_is_blocked = false;

    while(true)
    {
        //!
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        next = std::chrono::high_resolution_clock::now();
        //!

        if(!global_path.empty())
        {
            update_data(&redis, &global_path, &position);

            if(!destination_reach(&global_path.back(), position) && \
            (*(redis.get("State_global_path_is_computing"))).compare("false") == 0 && \
            (*(redis.get("State_destination_is_reach"))).compare("false") == 0 && \
            (*(redis.get("State_is_autonomous"))).compare("true") == 0 && \
            (*(redis.get("State_slamcore"))).compare("OK") == 0)
            {
                // 1#. compute the classic target keypoint.
                select_target_keypoint_2(&global_path, &target_keypoint);

                // 2#. project global path keypoint into the current local lidar map.
                std::vector<Pair> projected_keypoint; 
                project_keypoint_in_lidar_referencial(&global_path, &position, &target_keypoint, &projected_keypoint);
            
                // 3#. write lidar sample on clone cv::mat.
                cv::Mat clone = grid_RGB_2.clone();
                draw_lidar_data(&clone, &target_keypoint, &lidar_data);

                // 4#. new LCDS.
                double angle_ORIENTATION = 0; double angle_RKP = 0;
                angle_ORIENTATION = position[2]; angle_RKP = compute_vector_RKP(target_keypoint.coordinate, position);
                double alpha      = (angle_RKP - angle_ORIENTATION) * M_PI / 180; // difference in rad between robot angle and targetvector angle
                alpha             += M_PI;
                alpha             = atan2(sin(alpha), cos(alpha));                //[-PI:PI]
                if(alpha > 0 && alpha < M_PI_2) alpha = M_PI - alpha;
                else{ alpha = -(M_PI + alpha);}

                int command_type = is_on_site_rotation(alpha);
                if(command_type != 0)
                {
                    // Classic on site rotation.
                    if(command_type == -1)
                    {   //? ROTATION LEFT
                        redis.publish("command_micro", "-1/0.15/-1/0.15/-1/0.15/1/0.15/1/0.15/1/0.15/");
                    }
                    else
                    {   //? ROTATION RIGHT
                        redis.publish("command_micro", "1/0.15/1/0.15/1/0.15/-1/0.15/-1/0.15/-1/0.15/");
                    }
                }
                else
                {
                    // new LCDS checking for navigation.
                    // Create a gray map and color the invisible area.
                    cv::Mat clone_gray = grid_Gray_2.clone();
                    for(auto sample : lidar_data)
                    {   
                        // try avoid.
                        cv::circle(clone_gray, cv::Point((int)(sample.first),(int)(sample.second)),13, cv::Scalar(200), cv::FILLED, 0,0);
                    }
                    for(auto sample : lidar_data)
                    {   
                        // no center in.
                        cv::circle(clone_gray, cv::Point((int)(sample.first),(int)(sample.second)),7, cv::Scalar(0), cv::FILLED, 0,0);
                    }
                    cv::Mat clone_gray_invisible = clone_gray.clone();
                    draw_invisible_map(&clone_gray, &clone_gray_invisible);

                    // Get the fartest keypoint on the projected map.
                    Pair fartest_PKP;
                    double distance_max = 0;
                    for(int i = 0; i < projected_keypoint.size(); i++)
                    {
                        double distance = sqrt(pow(79-projected_keypoint[i].first,2)+pow(79-projected_keypoint[i].second,2));
                        if(distance > distance_max)
                        {
                            distance_max = distance;
                            fartest_PKP.first  = projected_keypoint[i].first;
                            fartest_PKP.second = projected_keypoint[i].second;
                        }
                    }

                    // check if the fartest_PKP is in invisible zone.
                    // choose a new one if is in invisible zone.
                    Pair new_destination; bool new_dest = true;
                    if((int)clone_gray_invisible.at<uchar>((int)(fartest_PKP.second), (int)(fartest_PKP.first)) == 0)
                    {   
                        // check the horizontal point left.
                        Pair left_destination; left_destination.first = -1; left_destination.second = -1;
                        for(int i = (int)(fartest_PKP.first); i >= 0; i--)
                        {
                            if((int)clone_gray_invisible.at<uchar>((int)(fartest_PKP.second), (int)(i)) != 0)
                            {
                                left_destination.first = i;
                                left_destination.second = (int)(fartest_PKP.second);
                            }
                        }
                        // check the horizontal point right.
                        Pair right_destination; right_destination.first = -1; right_destination.second = -1;
                        for(int i = (int)(fartest_PKP.first); i <= 159; i++)
                        {
                            if((int)clone_gray_invisible.at<uchar>((int)(fartest_PKP.second), (int)(i)) != 0)
                            {
                                right_destination.first = i;
                                right_destination.second = (int)(fartest_PKP.second);
                            }
                        }

                        // choose the new_destination.
                        if(left_destination.first != -1)
                        {
                            new_destination.first  = left_destination.first;
                            new_destination.second = left_destination.second;
                        }
                        if(right_destination.first != -1)
                        {
                            new_destination.first  = right_destination.first;
                            new_destination.second = right_destination.second;  
                        }
                        if(left_destination.first != -1 && right_destination.first != -1)
                        {
                            int dif_left  = (int)(fartest_PKP.first) - left_destination.first;
                            int dif_right = right_destination.first  - (int)(fartest_PKP.first);

                            if(dif_left - dif_right < 20)
                            {
                                new_destination.first  = right_destination.first;
                                new_destination.second = right_destination.second;
                            }
                            else
                            {
                                if(dif_left > dif_right)
                                {
                                    new_destination.first  = right_destination.first;
                                    new_destination.second = right_destination.second;   
                                }
                                else
                                {
                                    new_destination.first  = left_destination.first;
                                    new_destination.second = left_destination.second;  
                                }
                            }
                        }
                        if(left_destination.first == -1 && right_destination.first == -1)
                        {
                            new_dest = false;
                        }
                    }
                    else
                    {
                        new_destination.first  = fartest_PKP.first;
                        new_destination.second = fartest_PKP.second;
                    }

                    // compute A* with this new destination.
                    bool result_astar = false;
                    Pair current_position = {79,79}; std::vector<Pair> local_path;
                    if(new_dest)
                    {
                        result_astar = aStarSearch(clone_gray_invisible, current_position, new_destination, &redis, 1, &local_path);
                    }

                    if(result_astar)
                    {
                        redis.set("Error_debug", "NO_ERROR");

                        // select point for angle choose.
                        if(local_path.size() > 8) 
                        { 
                            target_keypoint.coordinate.first  = local_path[8].first;
                            target_keypoint.coordinate.second = local_path[8].second;
                        }
                        else
                        {
                            target_keypoint.coordinate.first  = local_path[local_path.size()-1].first;
                            target_keypoint.coordinate.second = local_path[local_path.size()-1].second;
                        }

                        // compute target angle.
                        int index_i = target_keypoint.coordinate.first - 79;
                        int index_j = 79 - target_keypoint.coordinate.second;
                        double angle_TKP = 9999;
                        if(index_i != 0) { angle_TKP = atan2((double)(index_j),(double)(index_i));} //in rad (-PI to PI)
                        else
                        {
                            index_i = 0.01;
                            angle_TKP = atan2((double)(index_j),(double)(index_i));
                        }
                        target_keypoint.target_angle = angle_TKP - M_PI_2;

                        // command nico.
                        compute_motor_autocommandeNico2(&redis, &target_keypoint, &navigation_param);
                    }
                    else
                    {
                        //! MAYBE WE ARE IN DESTINATION.
                        if(target_keypoint.distance_KPD < 2.5)
                        {
                            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
                            redis.set("State_destination_is_reach", "true");
                            redis.set("State_robot", "WAITING_FOR_CODE");
                            redis.set("Error_debug", "LOCAL_A*_FAIL_NEXT_TO_END");
                        }
                        else
                        {
                            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
                            redis.set("State_robot", "WAITING");
                            redis.set("Error_debug", "LOCAL_A*_FAIL");
                        }
                    }
                }
            
            }
            else
            {
                if(destination_reach(&global_path.back(), position)) redis.set("State_robot", "WAITING_FOR_CODE");
                else{redis.set("State_robot", "WAITING");}
                redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
            }
        }
        else
        {
            redis.set("State_robot", "WAITING");
            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
        }
    }

    //? ORIGINAL CODE (DOWN) //////////////////////////////////
    // while(true)
    // {
    //     ///TIMER///////////////////////////////////////////////////////////////////
    //     next                       += std::chrono::milliseconds((int)time_of_loop);
    //     std::this_thread::sleep_until(next);
    //     next = std::chrono::high_resolution_clock::now();
    //     ///END/////////////////////////////////////////////////////////////////////

    //     // if(debug_mode)
    //     // {   
    //     //     cv::Mat clone = grid_RGB_1.clone();
    //     //     show_local_environnement(&clone, &lidar_data, &current_speed);
    //     // }

    //     cv::Mat cloned = grid_RGB_2.clone();
    //     cv::Mat clonedg = grid_Gray_2.clone();

    //     bool LINEMODE;
    //     LINEMODE = true;

    //     std::vector<Pair> FDP;
    //     compute_new_TKP(&cloned, &FDP, &lidar_data, \
    //             &clonedg, &redis, &target_keypoint);

    //     // A#. update keypoint path information.
    //     if(!global_path.empty())
    //     {
    //         update_data(&redis, &global_path, &position);

    //         // B#. detect if we are arrived && if the global path is not computing.
    //         if(!destination_reach(&global_path.back(), position) && \
    //         (*(redis.get("State_global_path_is_computing"))).compare("false") == 0 && \
    //         (*(redis.get("State_destination_is_reach"))).compare("false") == 0 && \
    //         (*(redis.get("State_is_autonomous"))).compare("true") == 0 && \
    //         (*(redis.get("State_slamcore"))).compare("OK") == 0)
    //         {
    //             // 1#. compute the classic target keypoint.
    //             select_target_keypoint_2(&global_path, &target_keypoint);

    //             // 2#. project global path keypoint into the current local lidar map.
    //             std::vector<Pair> projected_keypoint; 
    //             project_keypoint_in_lidar_referencial(&global_path, &position, &target_keypoint, &projected_keypoint);

    //             // 3#. write lidar sample on clone cv::mat.
    //             cv::Mat clone = grid_RGB_2.clone();
    //             draw_lidar_data(&clone, &target_keypoint, &lidar_data);

    //             // 4#. compute a new TKP based on LCDS.
    //             cv::Mat clone_gray = grid_Gray_2.clone();
    //             if(compute_new_TKP(&clone, &projected_keypoint, &lidar_data, \
    //             &clone_gray, &redis, &target_keypoint))
    //             {
    //                 compute_motor_autocommandeNico(&redis, &target_keypoint, 1, \
    //                 &position, &navigation_param, &LINEMODE);
    //             }
    //             else
    //             {
    //                 //! This mean that no Global path KP could be projected in LCDS.
    //                 // Check if we destination is blocked du to drift.
    //                 if(target_keypoint.distance_KPD < 3.0)
    //                 {
    //                     if(destination_is_blocked)
    //                     {
    //                         redis.set("State_destination_is_reach", "true");
    //                         destination_is_blocked = false;
    //                     }
    //                     else
    //                     {
    //                         std::cout << "[DEBUG: destination is blocked]" <<  \
    //                         target_keypoint.distance_KPD << std::endl;
    //                         destination_is_blocked = true;
    //                         next += std::chrono::milliseconds((int)1500);
    //                         std::this_thread::sleep_until(next);
    //                     }
    //                 }
    //                 else
    //                 {
    //                     // This case occurend when the robot is not in the good rotation.
    //                     compute_motor_autocommandeNico(&redis, &target_keypoint, 0, \
    //                     &position, &navigation_param, &LINEMODE);
    //                 }
    //             }

    //             // 5#. security stop, if lidar data is on 20 cm trajector
    //             if(security_break(&lidar_data))
    //             {
    //                 redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
    //                 next += std::chrono::milliseconds((int)1000);
    //                 std::this_thread::sleep_until(next);
    //             }
    //         }
    //         else
    //         {
    //             redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
    //             redis.set("State_destination_is_reach", "true");
    //             redis.set("State_robot", "WAITING_FOR_CODE");
    //         }
    //     }
    // }
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
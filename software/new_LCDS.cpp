#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <bits/stdc++.h> 

#include <LCDS_lib.h>

//! COMMENTAIRE DESCRIPTION.
// commentaire programme.
//TODO: zone à debug.
//? questionnement programme.
//! message alerte & timer & structure information.

//! NAMESPACE.
using namespace sw::redis;

//! VARIABLE PARAM.
double max_m_dist = 200;
int lidarWindows_size = 30;
double LCDS_resolution = 0.05;
double m_LCDS_width    = 8.0;
double m_LCDS_height   = 3.0;
int p_LCDS_width       = (int)(m_LCDS_width/LCDS_resolution);
int p_LCDS_height      = (int)(m_LCDS_height/LCDS_resolution)*2;

//! VARIABLE OPENCV.
cv::Mat LCDS_color(p_LCDS_height, p_LCDS_width, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat LCDS_compute(p_LCDS_height, p_LCDS_width, CV_8UC1, cv::Scalar(255));

//! VARIABLE.
auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_A, thread_B, thread_C;
std::vector<Lidar_data> new_lidar_sample; 
std::string redis_input_str;
std::string GPKP_string = "";
std::vector<Pixel_position> GPKP;
Param_nav navigation_param;
Robot_complete_position position_robot;
std::vector<Lidar_sample> lidarWindows;
Intermediate_LCDS_KP ILKP(-1,-1);

//! VARIABLE SECONDAIRE.
int lidar_count = 0;

//! THREAD.
void callback_command(std::string channel, std::string msg)
{
    // reception lidar data.
    get_new_lidar_sample(&new_lidar_sample, msg);

    // add new lidar sample to lidarWindows.
    add_lidar_sample_to_lidarWindows(lidar_count, &lidarWindows, &new_lidar_sample, &position_robot);

    // increment lidar counter.
    lidar_count += 1;
    if(lidar_count >= lidarWindows_size) lidar_count = 0;
}

void function_thread_subscriber(sw::redis::Subscriber* sub)
{
    while(true)
    {
        sub->consume();
    }
}

void function_thread_redisData()
{
    //TODO: This thread will get all vital data from redis for LCDS.
    //TODO: 1. Global Path Keypoint Point (GPKP)
    //TODO: 2. The navigation parameters.

    //!T
    int frequency = 100;
    double time_of_loop = 1000/frequency;    
    auto next = std::chrono::high_resolution_clock::now();
    //!T

    while(true)
    {
        //!T
        next += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        next = std::chrono::high_resolution_clock::now();
        //!T

        // If the new GPKP is different than the current one, get it.
        redis_input_str = *(redis.get("State_global_path"));
        if(redis_input_str.compare(GPKP_string) != 0 && \
           redis_input_str.length() > 2 && \
           redis_input_str.compare("no_path") != 0)
        {
            // ILKP_reset(&ILKP);
            get_new_GPKP(&GPKP, redis_input_str);
            GPKP_string = redis_input_str;
        }

        // Get the navigation parameters.
        get_navigation_param(&redis, &navigation_param);

        // Detect if we have a loop closure.
        if(new_relocalisation(&redis))
        {
            reset_lidarWindows(lidarWindows);
            lidar_count = 0;
        }
    }
}

void function_thread_LCDS()
{
    //TODO: The main thread.
    //TODO: 1. An LCDS process is run when a new position is detected.
    //TODO: 2. An LCDS process is run when we are in Autonomous mode, and the GPKP is processed.
    //TODO: 3. AN LCDS process is run if we are not reach the destination yet.

    Pixel_position Local_destination(-1,-1);

    std::vector<bool> GPKP_notYetReached_b;
    GPKP_notYetReached_b.reserve(GPKP.capacity());
    setup_new_GPKP_notYetReached_b(&GPKP_notYetReached_b);

    std::vector<Pixel_position> GPKP_onLCDS;
    GPKP_onLCDS.reserve(GPKP.capacity());
    setup_new_GPKP(&GPKP_onLCDS); //! name of this function is not pertinant but function is.
    
    std::vector<Pixel_position> LW_onLCDS;
    LW_onLCDS.reserve(360*lidarWindows_size);
    setup_new_GPKP(&LW_onLCDS); //! name of this function is not pertinant but function is.

    std::vector<Pixel_position> Local_trajectory;
    Local_trajectory.reserve((int)(p_LCDS_width*1.5));
    setup_new_GPKP(&Local_trajectory); //! name of this function is not pertinant but function is.

    //TODO: Transform to redis_variable
    int64 t0 = cv::getTickCount();
    int64 t1 = cv::getTickCount();

    //TODO REMOVE & TEST
    int stop_command_counter = 0;
    
    while(true)
    {
        if(is_new_position_detected(&position_robot, &redis) && \
        (*(redis.get("State_global_path_is_computing"))).compare("false") == 0 && \
        (*(redis.get("State_destination_is_reach"))).compare("false") == 0 && \
        (*(redis.get("State_is_autonomous"))).compare("true") == 0 && \
        (*(redis.get("State_slamcore"))).compare("OK") == 0)
        {   
            //TODO: Transform to redis_variable
            t0 = cv::getTickCount();
            double secs = (t0-t1)/cv::getTickFrequency();
            // std::cout << "time for loops : " << secs << " Hz: " << 1/secs << std::endl;
            t1 = cv::getTickCount();

            // Filter the GPKP and show only unreach KP.
            filter_GPKP(&position_robot, &GPKP, &GPKP_notYetReached_b);

            // Create vector of projected GPKP on LCDS.
            project_GPKP_onLCDS(&LCDS_color, &GPKP, &GPKP_notYetReached_b, &GPKP_onLCDS, &position_robot);

            // Project LidarWindows on LCDS.
            cv::Mat LCDS_compute_clone = LCDS_compute.clone();
            project_LW_onLCDS(&position_robot, &lidarWindows, &LW_onLCDS, &LCDS_compute_clone, lidar_count);

            // Select Destination.
            select_local_destination(&Local_destination, &ILKP, &position_robot, &LCDS_compute_clone, &GPKP_onLCDS);
            
            // Create trajectory.
            create_trajectory(&redis, &Local_destination, &LCDS_compute_clone, Local_trajectory, &position_robot, &ILKP);

            // Reach destination checking.
            destination_is_reach(&GPKP, &position_robot, 3.0, &redis, Local_trajectory);
            
            // Compute motor commande.
            motor_control("MODEL_ADVANCE", Local_trajectory, &Local_destination, &LCDS_compute_clone, &redis, &position_robot, &navigation_param, stop_command_counter);

            if((*(redis.get("State_stream_LCDS"))).compare("ON") == 0)
            {
                debug_alpha(&LCDS_color, &LW_onLCDS, &GPKP_onLCDS, &lidarWindows, &position_robot, &Local_destination, &Local_trajectory, &ILKP, &redis);
            }
        }
        else
        {
            //! add an 100 ms sleep code.
            if((*(redis.get("State_slamcore"))).compare("OK") != 0 || (*(redis.get("State_destination_is_reach"))).compare("false") != 0) 
            { publish_basic_motor_control(&redis, 2); }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

//! MAIN STRUCTURE.
int main()
{   
    // Initialisation variable.
    new_lidar_sample.reserve(360);
    setup_new_lidar_sample(&new_lidar_sample);

    int max_GPKP_size = (int)(max_m_dist / std::stod(*(redis.get("Param_distance_btw_kp"))));
    GPKP.reserve(max_GPKP_size);
    setup_new_GPKP(&GPKP);

    lidarWindows.reserve(lidarWindows_size);
    setup_lidarWindows(&lidarWindows);

    // Initialisation subscriber to lidar raw data.
    auto sub = redis.subscriber();
    sub.on_message(callback_command);
    sub.subscribe("raw_data_lidar");

    // Initialisation all thread.
    thread_A = std::thread(&function_thread_subscriber, &sub);
    thread_B = std::thread(&function_thread_redisData);
    thread_C = std::thread(&function_thread_LCDS);
    thread_A.join();
    thread_B.join();
    thread_C.join();
    return 0;
}
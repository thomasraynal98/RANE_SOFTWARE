#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <random>
#include <atomic>
#include <string>

#include <slamcore/slamcore.hpp>
#include "vslam_lib.h"

using namespace sw::redis;

/*
    DESCRIPTION: the program will be compute the position ofslam the robot in previous visited map.
*/

auto redis = Redis("tcp://127.0.0.1:6379");
std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam_sys;
slamcore::PoseInterface<slamcore::camera_clock>::CPtr pose;
std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed; 
std::shared_ptr<slamcore::HeightMappingSubsystemInterface> heightMappingSubSystem;
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

        if(check_map_data(&redis) && !slam_is_running)
        {
            // ******************************************************************
            // Initialise SLAMcore API
            // ******************************************************************
            slamcore::slamcoreInit(
            slamcore::LogSeverity::Info, [](const slamcore::LogMessageInterface& message) {
            const time_t time = std::chrono::system_clock::to_time_t(message.getTimestamp());
            struct tm tm;
            localtime_r(&time, &tm);

            std::cerr << "[" << message.getSeverity() << " " << std::put_time(&tm, "%FT%T%z")
                        << "] " << message.getMessage() << "\n";
            });

            // ******************************************************************
            // Create/Connect SLAM System
            // ******************************************************************
            slamcore::v0::SystemConfiguration sysCfg;
            sysCfg.EnableWheelOdometry = false; /// @note Remember to enable this option! 
            // sysCfg.ConfigFilePath = ""; /// @note Please pass the path your wheel odometry calibration
                                        /// file here!
            // sysCfg.Source = slamcore::DataSource::RealSense;
            // sysCfg.GenerateMap = false;
            sysCfg.LoadSessionFilePath = ("../data_software/map/" + *(redis.get("Param_localisation")) + "_" + *(redis.get("Param_id_current_map")) + ".session").c_str();

            std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam = slamcore::createSLAMSystem(sysCfg);
            if (!slam)
            {
                std::cerr << "Error creating SLAM system!" << std::endl;
                slamcore::slamcoreDeinit();
                slam_is_running = false;
            }

            std::cout << "Starting SLAM..." << std::endl;

            // ******************************************************************
            // Open the device
            // ******************************************************************
            slam->open();

            // ******************************************************************
            // Enable pose stream
            // ******************************************************************
            slam->setStreamEnabled(slamcore::Stream::Pose, true);
            slam->setStreamEnabled(slamcore::Stream::MetaData, true);
            slam->setStreamEnabled(slamcore::Stream::Velocity, true);

            // ******************************************************************
            // Create the MobileRobot subsystem
            // ******************************************************************
            // if(!slam->getProperty<bool>(slamcore::Property::FeatureKinematicsEnabled))
            // {
            //     std::cerr << "This system doesn't support wheel odometry data!" << std::endl;
            //     slamcore::slamcoreDeinit();
            //     return -1;
            // }

            // if(!slam->isSubsystemSupported(slamcore::SubsystemType::MobileRobot))
            // {
            //     std::cerr << "This system doesn't support wheel odometry data!" << std::endl;
            //     slamcore::slamcoreDeinit();
            //     return -1;
            // }

            // robot_feed = slam->getSubsystem<slamcore::MobileRobotSubsystemInterface>();

            // *****************************************************************
            // Register callbacks!
            // *****************************************************************
    
            heightMappingSubSystem = slam->getSubsystem<slamcore::HeightMappingSubsystemInterface>();

            slam->registerCallback<slamcore::Stream::Pose>(
            [&pose](const slamcore::PoseInterface<slamcore::camera_clock>::CPtr& poseObj) 
            {   
                pose = poseObj;
            });

            // TODO: [BUG] resolve this problem.
            slam->registerCallback<slamcore::Stream::MetaData>(
            [&redis](const slamcore::MetaDataInterface::CPtr& metaObj) 
            {
                slamcore::MetaDataID ID = metaObj->getID();
                switch (ID)
                {
                case slamcore::MetaDataID::TrackingStatus:
                {   
                    int message;
                    std::string state = "NOT_INITIALISED";
                    metaObj->getValue(message);
                    if(message == 0) { state = "NOT_INITIALISED";}
                    if(message == 1) { state = "OK";}
                    if(message == 2) { state = "LOST";}
                    if(message == 3) 
                    { 
                        state = "RELOCALISED";
                        redis.set("State_need_compute_global_path", "true");
                    }
                    if(message == 4) 
                    { 
                        state = "LOOP_CLOSURE";
                        redis.set("State_need_compute_global_path", "true");
                    }

                    redis.set("State_slamcore", state);
                }
                break;
                }
            });

            slam->registerCallback<slamcore::Stream::Velocity>(
            [](const slamcore::VelocityInterface<slamcore::camera_clock>::CPtr& velObj) {
                std::string message = "";
                message = std::to_string(velObj->getLinear().z()) + "/" + std::to_string(velObj->getLinear().x()) + "/" + \
                std::to_string(velObj->getAngular().y()) + "/";
                redis.set("State_robot_speed", message);
            });

            // INIT ATTRIBU OBJECT.
            slam_sys = std::move(slam);
            slam_is_running = true;
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
            {
                heightMappingSubSystem->fetch();
                const auto map = heightMappingSubSystem->get();
                send_pose_information(&redis, *map, *pose);
            }
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
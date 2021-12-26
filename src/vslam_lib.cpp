#include "vslam_lib.h"
#include <sw/redis++/redis++.h>
#include <slamcore/slamcore.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <iostream>

#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <chrono>
#include <random>
#include <atomic>

bool init_slam_sdk(sw::redis::Redis* redis, std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam_sys, bool* is_running, \
std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed)
try
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
    sysCfg.ConfigFilePath = ""; /// @note Please pass the path your wheel odometry calibration
                                /// file here!

    std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam =
        slamcore::createSLAMSystem(sysCfg);
    if (!slam)
    {
        std::cerr << "Error creating SLAM system!" << std::endl;
        slamcore::slamcoreDeinit();
        return -1;
    }

    std::cout << "Starting SLAM..." << std::endl;

    // ******************************************************************
    // Open the device
    // ******************************************************************
    slam->openWithSession(("../data_software/map/" + *(redis->get("Param_localisation")) + "_" + *(redis->get("Param_id_current_map")) + ".session").c_str());

    // ******************************************************************
    // Print versions
    // ******************************************************************
    const std::string slam_version =
        slam->getProperty<std::string>(slamcore::Property::FirmwareVersion);
    const std::string slam_build_ver =
        slam->getProperty<std::string>(slamcore::Property::FirmwareBuildVersion);
    const std::string slam_build_type =
        slam->getProperty<std::string>(slamcore::Property::FirmwareBuildType);

    std::cout << "Client Version: " << slamcore::getVersion() << "/"
                << slamcore::getBuildVersion() << "/" << slamcore::getBuildType() << std::endl;
    std::cout << "SLAM Version: " << slam_version << "/" << slam_build_ver << "/"
                << slam_build_type << std::endl;

    // ******************************************************************
    // Enable pose stream
    // ******************************************************************
    slam->setStreamEnabled(slamcore::Stream::Pose, true);
    slam->setStreamEnabled(slamcore::Stream::MetaData, true);
    // ******************************************************************
    // Create the MobileRobot subsystem
    // ******************************************************************
    if(!slam->getProperty<bool>(slamcore::Property::FeatureKinematicsEnabled))
    {
        std::cerr << "This system doesn't support wheel odometry data!" << std::endl;
        slamcore::slamcoreDeinit();
        return -1;
    }

    if(!slam->isSubsystemSupported(slamcore::SubsystemType::MobileRobot))
    {
        std::cerr << "This system doesn't support wheel odometry data!" << std::endl;
        slamcore::slamcoreDeinit();
        return -1;
    }

    robot_feed = slam->getSubsystem<slamcore::MobileRobotSubsystemInterface>();

    // *****************************************************************
    // Register callbacks!
    // *****************************************************************
    slam->registerCallback<slamcore::Stream::Pose>(
    [redis](const slamcore::PoseInterface<slamcore::camera_clock>::CPtr& poseObj) 
    {   
        /* update translation. */
        std::string message_position;
        double x, y, z, r, p, yaw, q1, q2, q3, q4, pixel_yaw;
        x                  += poseObj->getTranslation().x();
        y                  += poseObj->getTranslation().y();
        z                  += poseObj->getTranslation().z();

        // robot_position.position.last_time = std::chrono::high_resolution_clock::now();

        /* update orientation. */
        q1 = poseObj->getRotation().x();
        q2 = poseObj->getRotation().y();
        q3 = poseObj->getRotation().z();
        q4 = poseObj->getRotation().w();

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q4 * q1 + q2 * q3);
        double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
        r = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q4 * q2 - q3 * q1);
        if (std::abs(sinp) >= 1)
            p = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            p = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q4 * q3 + q1 * q2);
        double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
        yaw = std::atan2(siny_cosp, cosy_cosp);

        // TODO : please god don't judge me.
        pixel_yaw = 360 - modulo((int)(yaw*(180/M_PI))+90,360);

        message_position += std::to_string(x) + "/" + std::to_string(y) + "/" + std::to_string(z) + "/" + std::to_string(r) + "/" + std::to_string(p) + "/" + std::to_string(yaw) + "/";
        redis->set("State_robot_position", message_position);
        
        // convert for the pixel grid camera position.
        int i = (int)((x +  std::stod(*(redis->get("Param_3D_to_pixel_A"))))  / 0.05);
        int j = (int)((std::stod(*(redis->get("Param_3D_to_pixel_B"))) - y)  / 0.05);
        message_position = std::to_string(i) + "/" + std::to_string(j) + "/";

        // convert for the pixel grid center robot position.
        double cam_to_center = std::stod(*(redis->get("Param_robot_length")));
        int center_i = x + sin(M_PI-yaw)*cam_to_center;
        int center_j = y + cos(M_PI-yaw)*cam_to_center;
        message_position = std::to_string(center_i) + "/" + std::to_string(center_j) + "/" + std::to_string(pixel_yaw) + "/";
        redis->set("State_robot_position_png", message_position);

    });

    // TODO: [BUG] resolve this problem.
    slam->registerCallback<slamcore::Stream::MetaData>(
    [redis](const slamcore::MetaDataInterface::CPtr& metaObj) 
    {
        slamcore::MetaDataID ID = metaObj->getID();
        switch (ID)
        {
        case slamcore::MetaDataID::TrackingStatus:
        {   
            std::string message;
            metaObj->getValue(message);
            redis->set("State_slamcore", message);
        }
        break;
        }
    });
    // INIT ATTRIBU OBJECT.
    slam_sys = std::move(slam);
    *is_running = true;
    return true;
}
catch (const slamcore::slam_exception& ex)
{
  std::cerr << "system_error exception! " << ex.what() << " / " << ex.code().message()
            << " / " << ex.code().value() << std::endl;
  slamcore::slamcoreDeinit();
  return -1;
}
catch (const std::exception& ex)
{
  std::cerr << "Uncaught std::exception! " << ex.what() << std::endl;
  slamcore::slamcoreDeinit();
  return -1;
}
catch (...)
{
  std::cerr << "Uncaught unknown exception!" << std::endl;
  slamcore::slamcoreDeinit();
  return -1;
}

bool check_map_data(sw::redis::Redis* redis)
{
    if(((*(redis->get("State_map_validate"))).compare("false") == 0) && \
    ((*(redis->get("State_map_available"))).compare("false") == 0))
    {
        return false;
    }
    return true;
}

int modulo(int a, int b) { return a < 0 ? b - (-a % b): a % b; }

void feed_encoder_data(std::string msg, std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed, slamcore::IDT* sample_counter)
{
    // parse msg string to 3 double value.
    
    // random number generator for the sake of the example
    std::uniform_real_distribution<double> dist_m(-10.0,10.0);
    std::uniform_real_distribution<double> dist_angle(-M_PI,M_PI);
    std::default_random_engine re;

    std::shared_ptr<slamcore::WheelOdometrySample>
    sample(new slamcore::WheelOdometrySample());

    // translation
    const slamcore::Vector translation(dist_m(re), 0.0, 0.0); // X, Y, 0. // Y = 0.0 car le robot ne va pas sur la droite ou la gauche.
    // peut etre que Y != 0 car si le robot tourne plus vite d'un coté que l'autre ok.

    // rotation
    const double theta = dist_angle(re); // rotation around the Z axis

    // convert to quaternion
    const double cz = std::cos(theta * 0.5);
    const double sz = std::sin(theta * 0.5);
    slamcore::Vector rotation(0.0, 0.0, sz, cz); // X, Y, Z, W

    // fill the measurement
    sample->setValue(translation, rotation);

    // fill the metadata
    sample->setID(*sample_counter++);
    sample->setSourceAcquisitionTimestamp(std::chrono::system_clock::now());
    // when this sample was captured

    // feed
    robot_feed->feedOdometry(sample);
}
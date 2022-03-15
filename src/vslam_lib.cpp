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

void send_pose_information(sw::redis::Redis* redis, const slamcore::Map2DInterface& map,
              const slamcore::PoseInterface<slamcore::camera_clock>& pose)
{
    const slamcore::VectorInterface& poseTranslation = pose.getTranslation();
    
    const slamcore::Vector mapPoint(poseTranslation.x(), poseTranslation.y());
    slamcore::CellCoordinates cellCoordinates;
    if(slamcore::pointWithinMap(map, mapPoint))
    {
        cellCoordinates = mapPointToCell(map, mapPoint);
    }

    std::string message_position;
    double x, y, z, r, p, yaw, q1, q2, q3, q4, pixel_yaw;
    x                  = poseTranslation.x();
    y                  = poseTranslation.y();
    z                  = poseTranslation.z();

    /* update orientation. */
    q1 = pose.getRotation().x();
    q2 = pose.getRotation().y();
    q3 = pose.getRotation().z();
    q4 = pose.getRotation().w();

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

    // Compute State_robot_position_center.
    double cam_to_center = std::stod(*(redis->get("Param_robot_length")))/2;
    message_position = std::to_string(x + sin(M_PI-yaw)*cam_to_center) + "/" + std::to_string(y + cos(M_PI-yaw)*cam_to_center) + "/0/0/0/0/";
    redis->set("State_robot_position_center", message_position);
    
    // convert for the pixel grid camera position.
    int i = cellCoordinates.x;
    int j = cellCoordinates.y;
    message_position = std::to_string(i) + "/" + std::to_string(map.getHeight() -j) + "/";

    // convert for the pixel grid center robot position.
    const slamcore::Vector mapPoint2(x + sin(M_PI-yaw)*cam_to_center, y + cos(M_PI-yaw)*cam_to_center);
    if(slamcore::pointWithinMap(map, mapPoint2))
    {
        cellCoordinates = mapPointToCell(map, mapPoint2);
    }

    int center_i = cellCoordinates.x;
    int center_j = cellCoordinates.y;
    message_position += std::to_string(center_i) + "/" + std::to_string(map.getHeight() - center_j) + "/" + std::to_string(pixel_yaw) + "/";
    redis->set("State_robot_position_png", message_position);
}

bool check_map_data(sw::redis::Redis* redis)
{
    if(((*(redis->get("State_map_validate"))).compare("true") == 0) && \
    ((*(redis->get("State_map_available"))).compare("true") == 0))
    {
        return true;
    }
    return false;
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
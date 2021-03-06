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
    // double tempo = yaw * (180/ M_PI) + 90.0;
    pixel_yaw = 360.0 - (yaw * 180 / M_PI + 180);
    if(pixel_yaw < 0) pixel_yaw += 360;
    // std::cout << "PIXEL = " << pixel_yaw << std::endl;

    // TODO : please god don't judge me. 2222
    double wait = yaw*(180/M_PI)+90;
    if(wait < 0) wait += 360;
    
    pixel_yaw = 360 - wait;

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

// int modulo(int a, int b) { return a < 0 ? b - (-a % b): a % b; }

double modulo(double a, double b) {
    double n = std::ceil(a / b);
    double r = a - b * n;
    if (a < 0) return b - r;
    return r;
}

void feed_encoder_data(std::string msg, std::shared_ptr<slamcore::MobileRobotSubsystemInterface> robot_feed, slamcore::IDT* sample_counter, double* grad, double* dx, double* dy)
{
    // parse msg string to 3 double value.

    //! On va garder la convention suivante, la valeur que j'envoie doit etre de l'odom??trie r??el.
    //! C'est ?? dire qu'il va envoyer l'estimation de la position et non juste le delta Translation ou Rotation.
    
    // random number generator for the sake of the example
    // std::uniform_real_distribution<double> dist_m(-10.0,10.0);
    // std::uniform_real_distribution<double> dist_angle(-M_PI,M_PI);
    // std::default_random_engine re;

    //! Get string value.
    std::string T;
    std::stringstream X(msg);
    int i = 0;
    std::vector<double> value_vec;
    while(std::getline(X, T, '/'))
    {
        // std::cout << T << std::endl;
        if(i > 0 && i < 7) { value_vec.push_back(std::stod(T));}
        i++;
    }

    //! BAD STUFF
    double wheel_perimetre  = 2.0 * M_PI * 10.0;

    double tic_per_rotation = 1796.0;
    double entraxe_en_tic   = tic_per_rotation * 54.0 / wheel_perimetre;
    double tic_per_meter    = tic_per_rotation * 100.0 / wheel_perimetre;

    double delta = (value_vec[0] + value_vec[1] + value_vec[2] + value_vec[3] + value_vec[4] + value_vec[5]) / 6;
    double delta_rot = (((value_vec[3] + value_vec[4] + value_vec[5])/3) - ((value_vec[0] + value_vec[1] + value_vec[2])/3))/2;
    
    *grad += delta_rot / entraxe_en_tic;
    *dx += cos(*grad) * delta / tic_per_meter;
    *dy += sin(*grad) * delta / tic_per_meter;

    std::cout << "grad=" << *grad << " dx=" << *dx << " dy=" << *dy << std::endl;

    std::shared_ptr<slamcore::WheelOdometrySample> sample(new slamcore::WheelOdometrySample());

    // translation
    slamcore::Vector translation(*dx, *dy, 0.0); // X, Y, 0.
    const double cz = std::cos(*grad * 0.5);
    const double sz = std::sin(*grad * 0.5);
    slamcore::Vector rotation(0.0, 0.0, sz, cz); // X, Y, Z, W

    // fill the measurement
    sample->setValue(translation, rotation);
    // fill the metadata
    sample->setID((*sample_counter)++);
    sample->setSourceAcquisitionTimestamp(std::chrono::system_clock::now());
    // when this sample was captured

    // feed
    robot_feed->feedOdometry(sample);
}
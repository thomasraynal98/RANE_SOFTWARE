#ifndef LCDS_LIB_H
#define LCDS_LIB_H

#include <string.h>
#include <iostream>
#include <vector>
#include <sw/redis++/redis++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "global_path_lib.h"

struct Lidar_data
{
    double angle, value;

    void operator=(Lidar_data& lidar_data2)
    {
        angle = lidar_data2.angle;
        value = lidar_data2.value;
    }

    // Constructor
    Lidar_data(double a, double b)
        : angle(a)
        , value(b)
        {}
};

struct Pixel_position
{
    int idx_row, idx_col;

    // Constructor
    Pixel_position(int col, int row)
        : idx_row(row)
        , idx_col(col)
        {}

    void operator=(Pixel_position& pixel2)
    {
        idx_row = pixel2.idx_row;
        idx_col = pixel2.idx_col;
    }
};

struct Robot_position_transformation
{
    //? Cette structure va contenir la transformation entre un sample de lidar T0 et le nouveau
    //? sample du lidar T1.
    //? |ET| elle va contenir la position du robot à un instant T.

    double x, y, yaw_deg;
    double delta_x, delta_y, detal_yaw_deg;

    Robot_position_transformation()
        : x(0)
        , y(0)
        , yaw_deg(0)
        , delta_x(0)
        , delta_y(0)
        , detal_yaw_deg(0)
        {}
};

struct Robot_complete_position
{
    //? Cette structure va comporter le maximun de données de position dans plusieurs referenciel
    //? du robot.
    double x_cam, y_cam, yaw_deg_cam;
    double x_center, y_center, yaw_deg_center;
    Pixel_position p_cam, p_center;

    Robot_complete_position()
        : x_cam(0)
        , y_cam(0)
        , yaw_deg_cam(0)
        , x_center(0)
        , y_center(0)
        , yaw_deg_center(0)
        , p_cam(-1,-1)
        , p_center(-1,-1)
        {}
    
    void operator=(Robot_complete_position& observation2)
    {
        x_cam = observation2.x_cam;
        y_cam = observation2.y_cam;
        yaw_deg_cam = observation2.yaw_deg_cam;
        x_center = observation2.x_center;
        y_center = observation2.y_center;
        yaw_deg_center = observation2.yaw_deg_center;
        p_cam = observation2.p_cam;
        p_center = observation2.p_center;
    }
    bool operator==(Robot_complete_position& observation2)
    {
        if(x_cam == observation2.x_cam && \
           y_cam == observation2.y_cam && \
        x_center == observation2.x_center && \
        y_center == observation2.y_center) return true;
        return false;
    }
};

struct Lidar_sample
{
    Robot_position_transformation viewpoint;
    std::vector<Lidar_data> observation;
};

struct Param_nav
{
    double V;
    double K;
    double F;
    int back_angle;
    double stall_pwm;
    double unstall_pwm;

    Param_nav()
        : V(0)
        , K(0)
        , F(0)
        , back_angle(0)
        , stall_pwm(0)
        , unstall_pwm(0)
        {}
};

struct Intermediate_LCDS_KP
{
    double angle, distance;
    //! This variable is usable just for one points of view.
    Pixel_position px_onLCDS;
    Robot_position_transformation viewpoint;

    // Constructor
    Intermediate_LCDS_KP(double a, double b)
        : angle(a)
        , distance(b)
        , px_onLCDS(-1,-1)
        {}

    void operator=(Intermediate_LCDS_KP& KP)
    {
        angle = KP.angle;
        distance = KP.distance;
        px_onLCDS = KP.px_onLCDS;
    }
};

//TODO: PART A function.
void setup_new_lidar_sample(std::vector<Lidar_data>* new_lidar_sample);
void setup_new_GPKP_notYetReached_b(std::vector<bool>* GPKP_notYetReached_b);
void get_new_lidar_sample(std::vector<Lidar_data>* new_lidar_sample, std::string raw_lidar_sample);
void setup_new_GPKP(std::vector<Pixel_position>* GPKP);
void get_new_GPKP(std::vector<Pixel_position>* GPKP, std::string redis_input_str);
void get_navigation_param(sw::redis::Redis* redis, Param_nav* navigation_param);
void setup_lidarWindows(std::vector<Lidar_sample>* lidarWindows);
void add_lidar_sample_to_lidarWindows(int lidar_count, std::vector<Lidar_sample>* lidarWindows, std::vector<Lidar_data>* new_lidar_sample, Robot_complete_position* position_robot);
bool is_new_position_detected(Robot_complete_position* position_robot, sw::redis::Redis* redis);
void filter_GPKP(Robot_complete_position* position_robot, std::vector<Pixel_position>* GPKP, std::vector<bool>* GPKP_notYetReached_b);
double distance_btw_pixel(Pixel_position p1, Pixel_position p2, double resolution);
double distance_btw_pts(double pts_A_x, double pt_A_y, double pt_B_x, double pt_B_y);
void project_GPKP_onLCDS(cv::Mat* LCDS_color, std::vector<Pixel_position>* GPKP, std::vector<bool>* GPKP_notYetReached_b, std::vector<Pixel_position>* GPKP_onLCDS, Robot_complete_position* position_robot);
double compute_distance_RKP(Robot_complete_position* position_robot, Pixel_position* KP);
double compute_angle_RKP_onLCDS(Robot_complete_position* position_robot, Pixel_position* KP);
double compute_angle_btw_Robot_and_observation_pts(double pt_x, double pt_y, Robot_complete_position* position_robot);
double deg_to_rad(double deg);
double rad_to_deg(double rad);
void project_LW_onLCDS(Robot_complete_position* position_robot, std::vector<Lidar_sample>* lidarWindows, std::vector<Pixel_position>* LW_onLCDS, cv::Mat* LCDS_color, int lidar_count);
double compute_angle_btw_angle(double angle_principal, double angle_secondaire);
bool new_relocalisation(sw::redis::Redis* redis);
void reset_lidarWindows(std::vector<Lidar_sample>* lidarWindows);

//TODO: PART B function.
void project_ILKP_onLCDS(cv::Mat* LCDS_color, Robot_complete_position* position_robot, Intermediate_LCDS_KP* ILKP);
void select_local_destination(Pixel_position* Local_destination, Intermediate_LCDS_KP* ILKP, Robot_complete_position* position_robot, cv::Mat* LCDS_compute, std::vector<Pixel_position>* GPKP);
bool ILKP_is_onLCDS(cv::Mat* LCDS_compute, Intermediate_LCDS_KP* ILKP);
void ILKP_reset(Intermediate_LCDS_KP* ILKP);
bool ILKP_is_reach(cv::Mat* LCDS_compute, Intermediate_LCDS_KP* ILKP, double m_validation_distance);
bool ILKP_is_available(cv::Mat* LCDS_compute, Intermediate_LCDS_KP* ILKP);
bool PKP_is_available(cv::Mat* LCDS_compute, Pixel_position* px);
void create_trajectory(sw::redis::Redis* redis, Pixel_position* Local_destination, cv::Mat* LCDS_compute, std::vector<Pixel_position>* Local_trajectory, Robot_complete_position* position_robot, Intermediate_LCDS_KP* ILKP);
void reset_Pixel_position_vector(std::vector<Pixel_position>* Local_trajectory);
Pixel_position select_FPKP(Pixel_position* Local_destination, cv::Mat* LCDS_compute, std::vector<Pixel_position>* GPKP_onLCDS, int unknow_option);
bool is_the_same(Pixel_position* px_A, Pixel_position* px_B);

//TODO: DEBUG FONCTION.
void debug_data(std::vector<Lidar_data>* new_lidar_sample, std::vector<Lidar_sample>* lidarWindows);
void debug2_data(std::vector<Pixel_position>* GPKP, std::vector<bool>* GPKP_notYetReached_b);
void debug_alpha(cv::Mat* LCDS_color, std::vector<Pixel_position>* LW_onLCDS, std::vector<Pixel_position>* GPKP_onLCDS, std::vector<Lidar_sample>* lidarWindows, Robot_complete_position* position_robot, Pixel_position* Local_destination, std::vector<Pixel_position>* Local_trajectory, Intermediate_LCDS_KP* ILKP);
void debug_add_robotShape(cv::Mat* LCDS_color);

#endif
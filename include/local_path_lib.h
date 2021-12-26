#ifndef LOCAL_PATH_LIB_H
#define LOCAL_PATH_LIB_H

#include <sw/redis++/redis++.h>
#include "global_path_lib.h"

struct Path_keypoint 
{
    /*
        DESCRIPTION: this structure include all information
            of path between robot and destination.
    */

    // Pixel coordinate of path keypoint.
    Pair coordinate;
    // Do the robot reach this keypoint.
    bool isReach;
    // Distance from robot to keypoint.
    double distance_RKP;
    // Distance from current keypoint to destination. (Pas à vol d'oiseau)
    double distance_KPD;
    // Angle between robot orientation and robot to keypoint orientation.
    double target_angle; 
    // Angle between last KP, futur KP & with current KP to middle.
    double validation_angle;
    // True if this pixel is try_avoid area.
    bool isTryAvoidArea;
    // Distance to declare that robot reach point.
    double distance_validation;
    
    // Constructor
    Path_keypoint()
        : coordinate()
        , isReach(false)
        , distance_RKP(-1)
        , distance_KPD(-1)
        , target_angle(-1)
        , validation_angle(-1)
        , isTryAvoidArea(false)
        , distance_validation(-1)
        {}
};

struct Param_nav
{
    double V;
    double K;
    double F;
    int back_angle;
    double stall_pwm;
    double unstall_pwm;
    bool get_param;

    Param_nav()
        : V(0)
        , K(0)
        , F(0)
        , back_angle(0)
        , stall_pwm(0)
        , unstall_pwm(0)
        , get_param(false)
        {}
};

std::vector<Pair> format_lidar_data(std::string raw_msg);
bool check_process_LCDS(sw::redis::Redis* redis);
void show_local_environnement(cv::Mat* grid, std::vector<Pair>* data_lidar, std::vector<double>* encoder_data);
void get_robot_speed(sw::redis::Redis* redis, std::vector<double>* encoder_data);
void get_global_path(sw::redis::Redis* redis, std::vector<Path_keypoint>* encoder_data);
double compute_distance_validation(Path_keypoint kp);
double compute_target_angle(Pair kp, std::vector<double> current_position);
double compute_distance_RPK(Pair kp, std::vector<double> current_position);
double compute_vector_RKP(const Pair& kp, std::vector<double> current_position);
std::vector<double> get_current_position_n(sw::redis::Redis* redis);
double compute_validation_angle(const Pair& kpPrev, const Pair& kpCurrent, const Pair& kpNext);
double compute_vector_RKP_2(const Pair& kpCurrent, const Pair& kp2);
void select_target_keypoint(std::vector<Path_keypoint>* global_path_keypoint, Path_keypoint* target_keypoint);
void return_nearest_path_keypoint(double threshold, std::vector<Path_keypoint>* global_path_keypoint, std::vector<Path_keypoint*>* possible_candidate_target_keypoint);
bool destination_reach(Path_keypoint* destination, std::vector<double> current_position);
void update_data(sw::redis::Redis* redis, std::vector<Path_keypoint>* global_keypoint, std::vector<double>* current_position);
std::vector<Pair> project_keypoint_in_lidar_referencial(std::vector<Path_keypoint>* global_keypoint, std::vector<double>* current_position, Path_keypoint* TKP);
std::vector<Pair> transform_angle_in_lidar_ref(std::vector<Path_keypoint*> keypoints_list_for_projection, std::vector<double>* position, Path_keypoint* TKP);
bool simulation_problem(int futur_ms, cv::Mat* grid, std::vector<double>* current_speed, std::vector<Pair>* data_lidar);
bool TKP_problem(cv::Mat* grid, Path_keypoint* TKP, std::vector<Pair>* data_lidar);
void select_target_keypoint_2(std::vector<Path_keypoint>* global_path_keypoint, Path_keypoint* target_keypoint);
void compute_new_TKP(cv::Mat* grid_RGB, std::vector<Pair>* projected_keypoint, std::vector<Pair>* data_lidar, cv::Mat* grid_gray, sw::redis::Redis* redis, Path_keypoint* TKP);
void get_navigation_param(sw::redis::Redis* redis, Param_nav* navigation_param);
void compute_motor_autocommandeNico(sw::redis::Redis* redis, Path_keypoint* TKP, int option, std::vector<double>* position, Param_nav* navigation_param);
bool security_break(std::vector<Pair>* data_lidar);
#endif
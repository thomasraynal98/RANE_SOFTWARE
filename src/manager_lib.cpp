#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <sw/redis++/redis++.h>

#include "manager_lib.h"

bool read_ID_file_and_push(sw::redis::Redis* redis)
{
    cv::FileStorage fsSettings("../data_software/parameter/ID_file.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings identification." << std::endl;
        return false;
    }

    std::string read_data;

    fsSettings["Param_modele"] >> read_data;
    redis->set("Param_modele", read_data);

    fsSettings["Param_version"] >> read_data;
    redis->set("Param_version", read_data);

    fsSettings["Param_matricule"] >> read_data;
    redis->set("Param_matricule", read_data);

    fsSettings["Param_exploitation"] >> read_data;
    redis->set("Param_exploitation", read_data);

    fsSettings["Param_prenom"] >> read_data;
    redis->set("Param_prenom", read_data);

    fsSettings.release();

    return true;
}

bool read_PARAM_file_and_push(sw::redis::Redis* redis)
{
    cv::FileStorage fsSettings("../data_software/parameter/PARAM_file.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings identification." << std::endl;
        return false;
    }

    std::string read_data;

    fsSettings["Param_K"] >> read_data;
    redis->set("Param_K", read_data);

    fsSettings["Param_V"] >> read_data;
    redis->set("Param_V", read_data);

    fsSettings["Param_F"] >> read_data;
    redis->set("Param_F", read_data);

    fsSettings["Param_back_angle"] >> read_data;
    redis->set("Param_back_angle", read_data);

    fsSettings["Param_stall_pwm"] >> read_data;
    redis->set("Param_stall_pwm", read_data);

    fsSettings["Param_unstall_pwm"] >> read_data;
    redis->set("Param_unstall_pwm", read_data);

    fsSettings.release();

    return true;
}

bool read_MAP_file_and_push(sw::redis::Redis* redis)
{
    cv::FileStorage fsSettings("../data_software/parameter/MAP_file.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings identification." << std::endl;
        return false;
    }

    std::string read_data;

    fsSettings["Param_localisation"] >> read_data;
    redis->set("Param_localisation", read_data);

    fsSettings["Param_id_current_map"] >> read_data;
    redis->set("Param_id_current_map", read_data);

    fsSettings["Param_type_current_map"] >> read_data;
    redis->set("Param_type_current_map", read_data);

    fsSettings["Param_saved_map"] >> read_data;
    redis->set("Param_saved_map", read_data);

    fsSettings.release();

    return true;
}

bool write_ID_file(sw::redis::Redis* redis)
{
    cv::FileStorage fsSettings("../data_software/parameter/ID_file.yaml", cv::FileStorage::WRITE);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings navigation info." << std::endl;
        return false;
    }

    fsSettings << "Param_modele" << *(redis->get("Param_modele"));
    fsSettings << "Param_version" << *(redis->get("Param_version"));
    fsSettings << "Param_matricule" << *(redis->get("Param_matricule"));
    fsSettings << "Param_exploitation" << *(redis->get("Param_exploitation"));
    fsSettings << "Param_prenom" << *(redis->get("Param_prenom"));

    fsSettings.release();
    return true;
}

bool write_PARAM_file(sw::redis::Redis* redis)
{
    cv::FileStorage fsSettings("../data_software/parameter/PARAM_file.yaml", cv::FileStorage::WRITE);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings navigation info." << std::endl;
        return false;
    }

    fsSettings << "Param_K" << *(redis->get("Param_K"));
    fsSettings << "Param_V" << *(redis->get("Param_V"));
    fsSettings << "Param_F" << *(redis->get("Param_F"));
    fsSettings << "Param_back_angle" << *(redis->get("Param_back_angle"));
    fsSettings << "Param_stall_pwm" << *(redis->get("Param_stall_pwm"));
    fsSettings << "Param_unstall_pwm" << *(redis->get("Param_unstall_pwm"));

    fsSettings.release();
    return true;
}

bool write_MAP_file(sw::redis::Redis* redis)
{
    cv::FileStorage fsSettings("../data_software/parameter/MAP_file.yaml", cv::FileStorage::WRITE);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings navigation info." << std::endl;
        return false;
    }

    fsSettings << "Param_localisation" << *(redis->get("Param_localisation"));
    fsSettings << "Param_id_current_map" << *(redis->get("Param_id_current_map"));
    fsSettings << "Param_type_current_map" << *(redis->get("Param_type_current_map"));
    fsSettings << "Param_saved_map" << *(redis->get("Param_saved_map"));

    fsSettings.release();
    return true;
}
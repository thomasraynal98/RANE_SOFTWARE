#include "com_server_lib.h"
#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include "global_path_lib.h"

void get_param_data(sw::redis::Redis* redis, std::string* adress)
{
    *adress = *(redis->get("Param_server_adress"));
}

void send_robot_status(sw::redis::Redis* redis, sio::socket::ptr current_socket, std::string topic_name)
{
    sio::message::ptr data_robot = sio::object_message::create();
    // data_robot: nom, position{Lat, Long}, batterie, status.

    std::string name_robot = *(redis->get("Param_modele")) + "_" + *(redis->get("Param_exploitation"));
    data_robot->get_map()["name"]       = sio::string_message::create(name_robot);


    double longitude = 2.236340;
    double latitude  = 48.896460;

    transform_in_world_ref(&longitude, &latitude);

    data_robot->get_map()["latitude"]   = sio::double_message::create(latitude);
    data_robot->get_map()["longitude"]  = sio::double_message::create(longitude);

    double batterie_level = 95;
    data_robot->get_map()["batterie"]   = sio::double_message::create(batterie_level);

    std::string status_robot = *(redis->get("State_robot"));
    data_robot->get_map()["status"]     = sio::string_message::create(status_robot);

    current_socket->emit(topic_name, data_robot);
}

void check_the_good_map(sw::redis::Redis* redis, sio::socket::ptr current_socket)
{
    sio::message::ptr test = sio::object_message::create();

    test->get_map()["localisation"] = sio::string_message::create(*(redis->get("Param_localisation")));
    test->get_map()["map_id"]       = sio::int_message::create(std::stoi(*(redis->get("Param_id_current_map"))));
    
    current_socket->emit("check_map", test);
}

void active_download_map(sw::redis::Redis* redis)
{
    redis->publish("command_data_manager", "download_new_map");
}

void map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle)
{
    //TODO: Cette fonction va mapper les commandes brutes de la manelle en commande 
    //TODO: utilisable par le robot.

    if(back_value < 0.05 && front_value < 0.05)
    {
        //! break.
        redis->publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
        return;
    }
    if(back_value > 0.05 && front_value > 0.05)
    {
        //! Security break.
        redis->publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
        return;
    }

    double vitesse_max = 1.0; // m/s

    if(front_value > 0.05)
    {
        // 0   - 90  = plus vers la gauche
        // 90  - 180 = plus vers la droite
        // 180 - 270 = rotation droite
        // 270 - 360 = rotation gauche

        double current_speed = front_value * vitesse_max / 100; //? 1 value max de front_value.

        if(angle > 0   && angle <= 90)
        {
            std::string msg = "1/";
            for(int i = 0; i < 6; i++)
            {
                if(i >= 3) msg += "1/" + std::to_string(current_speed) + "/";
                if(i < 3) msg += "1/" + std::to_string(current_speed - current_speed*(90-angle)/90)  + "/";
            }
            redis->publish("command_micro", msg);
        }
        if(angle > 90  && angle <= 180)
        {
            std::string msg = "1/";
            for(int i = 0; i < 6; i++)
            {
                if(i >= 3) msg += "1/" + std::to_string(current_speed - current_speed*(angle-90)/90) + "/";
                if(i < 3) msg += "1/" + std::to_string(current_speed)  + "/";
            }
            redis->publish("command_micro", msg);
        }
        if(angle > 180 && angle <= 270)
        {
            std::string msg = "1/";
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) msg += "1/" + std::to_string(current_speed) + "/";
                if(i >= 3) msg += "-1/" + std::to_string(current_speed) + "/";
            }
            redis->publish("command_micro", msg);
        }
        if(angle > 270 && angle <= 360)
        {
            std::string msg = "1/";
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) msg += "-1/" + std::to_string(current_speed) + "/";
                if(i >= 3) msg += "1/" + std::to_string(current_speed) + "/";
            }
            redis->publish("command_micro", msg);
        }
    }

    if(back_value > 0.05)
    {
        std::cout << angle << std::endl;
        double current_speed = back_value * vitesse_max / 100; //? 1 value max de front_value.

        if(angle == 90)
        {
            std::string msg = "1/";
            for(int i = 0; i < 6; i++)
            {
                msg += "-1/" + std::to_string(current_speed) + "/";
            }
            redis->publish("command_micro", msg);
        }
        if(angle > 180 && angle <= 270)
        {
            std::string msg = "1/";
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) msg += "-1/" + std::to_string(current_speed) + "/";
                if(i >= 3) msg += "-1/" + std::to_string(current_speed - current_speed*(90-(angle-180))/90) + "/";
            }
            redis->publish("command_micro", msg);
        }
        if(angle > 270 && angle <= 360)
        {
            std::string msg = "1/";
            for(int i = 0; i < 6; i++)
            {
                if(i < 3) msg += "-1/" + std::to_string(current_speed - current_speed*(angle-270)/90) + "/";
                if(i >= 3) msg += "-1/" + std::to_string(current_speed) + "/";
            }
            redis->publish("command_micro", msg);
        }
    }
}

void send_robot_identifiant(sw::redis::Redis* redis, sio::socket::ptr current_socket, std::string topic_name)
{
    std::string name_robot = *(redis->get("Param_modele")) + "_" + *(redis->get("Param_exploitation"));
    current_socket->emit("robot", name_robot);
}

void nicolas_test_function(sw::redis::Redis* redis, double back_value, double front_value, double angle)
{
    std::string msg = "1/";
    
    //TODO:

    //! Get r and v

    double r_max = 9999;
    double r_min = 0.001;
    double v_max = 1; // m/s
    
    double n_angle = ((angle / 360) * 2 * M_PI) - M_PI; // [-pi,pi]
    double r;

    if(n_angle < 0) r = r_max + r_min + r_max * (n_angle / M_PI);
    else{ r = r_max + r_min - r_max * (n_angle / M_PI); } 

    double v = v_max * (back_value - front_value) / 100;

    //? DEBUG
    std::string log = "v = " + std::to_string(v) + " r = " + std::to_string(r);
    redis->set("Error_debug", log);

    //! Calculate motor speed.

    double xk = 0; // difference on x btw center of robot and center of rotation.
    double e  = 0.23; // distance btw wheel en m.
    double l  = 0.53 / 2; 

    std::vector<double> x{e,0,-e,e,0,-e};
    std::vector<double> y{-l,-l,-l,l,l,l};
    
    for(int i = 0; i < 6; i++)
    {
        int side = 1;
        if(r < 0) side = -1;

        r = abs(r);

        double speed_wheel = (v/r)*sqrt(pow(r-y[i]*side,2)+pow(x[i]-xk,2));
        if(speed_wheel < 0) 
        {
            side *= -1;
            speed_wheel = abs(speed_wheel);
        }
        msg += std::to_string(side) + "/" + std::to_string(speed_wheel) + "/";
    }

    //TODO: END

    redis->publish("command_micro", msg);
}

void send_image_64base(sio::socket::ptr current_socket, std::string base64_msg)
{
    base64_msg = "data:image/jpg;base64, " + base64_msg;
    current_socket->emit("stream_video", base64_msg);
}

void transform_in_world_ref(sw::redis::Redis* redis, double * longitude, double * latitude)
{
    // DESCRITION: Cette fonction va recuperer la position x, y, yaw et va la transformer 
    // en position latitude et longitude.

    // PARAM CALCUL.
    Pair pt_glob_1(2.4, 49.8);
    Pair pt_glob_2(2.54, 49.81);

    double p_dx = pt_glob_1.first;
    double p_dy = pt_glob_1.second;

    Pair pt_map_1(0.0, 0.0);
    Pair pt_map_2(12.5,14.0);

    double distance_vecteur_map = sqrt(pow(pt_map_2.first,2)+pow(pt_map_2.second,2));
    double distance_vecteur_global = sqrt(pow(pt_glob_1.first-pt_glob_2.first,2)+pow(pt_glob_1.second-pt_glob_2.second,2));
    double p_scale = distance_vecteur_global / distance_vecteur_map;

    double p_angle_transfo = atan2(pt_map_2.second-pt_map_1.second, pt_map_2.first-pt_map_1.first) - atan2(pt_glob_2.second-pt_glob_1.second, pt_glob_2.first-pt_glob_1.first);
    if(p_angle_transfo < 0) {p_angle_transfo += 2 * M_PI;} // [0,2PI]
    // END PARAM.

    // START COMPUTE.
    double x, y, yaw;
    std::string T;
    redis_output_position_string = *(redis->get("State_robot_position_center"));
    std::stringstream X3(redis_output_position_string);
    i = 0;
    while(std::getline(X3, T, '/'))
    {
        if(i == 0) 
        {
            x = std::stod(T);
        }
        if(i == 1) 
        {
            y = std::stod(T);
        }
        i += 1;
    }
    
    double distance = sqrt(pow(x,2)+pow(y,2));
    double bonus    = M_PI_4;
    double angle    = p_angle_transfo + bonus;

    *longitude = p_dx + (x * cos(angle) - y * sin(angle)) * p_scale;
    *latitude  = p_dy + (x * sin(angle) + y * cos(angle)) * p_scale;
}
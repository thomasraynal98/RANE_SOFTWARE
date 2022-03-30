#include "com_server_lib.h"
#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

void get_param_data(sw::redis::Redis* redis, std::string* adress)
{
    *adress = *(redis->get("Param_server_adress"));
}

// void bind_events(sw::redis::Redis* redis, sio::socket::ptr current_socket)
// {
//     /*
//         DESCRIPTION: this function store all kind of message that we can receive 
//             from the main API.
//     */

//     /* If our current map is the good one. */
//     current_socket->on("good", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
//     {
//         redis->set("State_map_validate", "true");
//     }));

//     /* If our current map is not the good one, we need to get a new one. */
//     current_socket->on("download", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
//     {
//         redis->set("Param_localisation", std::to_string(data->get_map()["id"]->get_int()));
//         redis->set("Param_id_current_map", data->get_map()["localisation"]->get_string());
//         redis->set("Param_link_current_map_session", data->get_map()["link_session"]->get_string());
//         redis->set("Param_link_current_map_png", data->get_map()["link_png"]->get_string());
//         redis->set("State_map_available", "false");
//         redis->set("State_map_validate", "true");
//     }));

//     /* In manual mode we need that robot do a precise command. */
//     current_socket->on("command_to_do", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
//     {
//         redis->set("State_is_autonomous", "false");
//         redis->publish("command_micro", data->get_string());
//     }));

//     /* In autonav mode we need that robot reach a new point. */
//     current_socket->on("position_to_reach", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
//     {
//         std::string msg_destination = std::to_string(data->get_map()["i"]->get_int()) + "/" + std::to_string(data->get_map()["j"]->get_int()) + "/";
//         redis->set("State_position_to_reach", msg_destination);
//         redis->set("State_is_autonomous", "true");
//         redis->set("State_destination_is_reach", "false");
//         redis->set("State_need_compute_global_path", "true");
//     }));

//     /* Ping pong from API. */
//     current_socket->on("ping", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
//     {
//         current_socket->emit("pong");
//     }));

//     //TODO: (new) part. /--------------------

//     current_socket->on("operator_order_command", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
//     {
//         //TODO: mettre au claire ça.
//         redis->set("State_order", data->get_string());
//     }));

//     current_socket->on("operator_order_controller", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
//     {
//         //! pass code in no autonomous mode.
//         std::cout << "YOOOOO" << std::endl;
//         // redis->set("State_is_autonomous", "false");
//         std::cout << "YOOOOO" << std::endl;
//         // map_manual_command(redis, data->get_vector()[1]->get_double(), data->get_vector()[2]->get_double(), data->get_vector()[3]->get_double());
//     }));
// }

void send_robot_status(sw::redis::Redis* redis, sio::socket::ptr current_socket, std::string topic_name)
{
    sio::message::ptr data_robot = sio::object_message::create();
    // data_robot: nom, position{Lat, Long}, batterie, status.

    std::string name_robot = *(redis->get("Param_modele")) + "_" + *(redis->get("Param_exploitation"));
    data_robot->get_map()["name"]       = sio::string_message::create(name_robot);

    double longitude = 2.236340;
    double latitude  = 48.896460;
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
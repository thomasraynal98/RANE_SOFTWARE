#include "com_server_lib.h"
#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <ConvertImage.h>

using namespace sw::redis;

/*
    DESCRIPTION: the program will be in charge of communicate with server.
*/

auto redis = Redis("tcp://127.0.0.1:6379");
sio::socket::ptr current_socket;
std::thread thread_A, thread_B, thread_C;
sio::client h;
bool stream_video = false;

void bind_events(sio::socket::ptr current_socket)
{
    /*
        DESCRIPTION: this function store all kind of message that we can receive 
            from the main API.
    */

    /* If our current map is the good one. */
    current_socket->on("good", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        redis.set("State_map_validate", "true");
    }));

    /* If our current map is not the good one, we need to get a new one. */
    current_socket->on("download", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        redis.set("Param_localisation", std::to_string(data->get_map()["id"]->get_int()));
        redis.set("Param_id_current_map", data->get_map()["localisation"]->get_string());
        redis.set("Param_link_current_map_session", data->get_map()["link_session"]->get_string());
        redis.set("Param_link_current_map_png", data->get_map()["link_png"]->get_string());
        redis.set("State_map_available", "false");
        redis.set("State_map_validate", "true");
    }));

    /* In manual mode we need that robot do a precise command. */
    current_socket->on("command_to_do", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        redis.set("State_is_autonomous", "false");
        redis.publish("command_micro", data->get_string());
    }));

    /* In autonav mode we need that robot reach a new point. */
    current_socket->on("position_to_reach", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        std::string msg_destination = std::to_string(data->get_map()["i"]->get_int()) + "/" + std::to_string(data->get_map()["j"]->get_int()) + "/";
        redis.set("State_position_to_reach", msg_destination);
        redis.set("State_is_autonomous", "true");
        redis.set("State_destination_is_reach", "false");
        redis.set("State_need_compute_global_path", "true");
    }));

    /* Ping pong from API. */
    current_socket->on("ping", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        current_socket->emit("pong");
    }));

    //TODO: (new) part. /--------------------

    current_socket->on("operator_order_command", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        //TODO: mettre au claire ça.
        std::string new_order = data->get_string();

        if(new_order.compare("STOP") == 0)
        {
            redis.set("State_order", data->get_string());
            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
            redis.set("State_is_autonomous", "false");
        }
        if(new_order.compare("HOME") == 0)
        {
            redis.set("State_position_to_reach", "544/750/");
            redis.set("State_need_compute_global_path", "true");
            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
            redis.set("State_is_autonomous", "true");
        }
        if(new_order.compare("OPEN-MODULE") == 0)
        {
            redis.set("State_order", "OPEN");
            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
        }
        if(new_order.compare("CLOSE-MODULE") == 0)
        {
            redis.set("State_order", "CLOSE");
            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
        }
        
    }));

    current_socket->on("operator_order_controller", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        //! pass code in no autonomous mode.
        redis.set("State_is_autonomous", "false");
        // nicolas_test_function(redis, data->get_vector()[1]->get_double(), data->get_vector()[2]->get_double(), data->get_vector()[3]->get_double());
        map_manual_command(&redis, data->get_vector()[1]->get_double(), data->get_vector()[2]->get_double(), data->get_vector()[3]->get_double());
    }));

    // Stream video
    current_socket->on("stream_ON", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        std::cout << "NEW_MESSAGE=" << data->get_string() << std::endl;
        if(data->get_string().compare("TRUE") == 0)
        {
            stream_video = true;
            redis.set("State_stream", "ON");
        }
        if(data->get_string().compare("FALSE") == 0)
        {
            stream_video = false;
            redis.set("State_stream", "OFF");
        }
    }));
}

void function_thread_A()
{
    // THREAD DESCRIPTION: send data to server and manage connection.

    int frequency       = 5;
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

        send_robot_identifiant(&redis, h.socket(), "robot");
        send_robot_status(&redis, h.socket(), "robot_data_operator");
    }
}

void function_thread_C()
{
    // THREAD VIDEO.

    while(true)
    {
        //open the video file for reading
    
        while (stream_video)
        {
            cv::VideoCapture cap(4); 

            std::string window_name = "Debug_screen_video";

            int i = 0;

            while(stream_video)
            {
                cv::Mat frame; cv::Mat Dest;
                bool bSuccess = cap.read(frame); // read a new frame from video 
                cv::resize(frame, Dest, cv::Size(0,0), 0.20, 0.20, 6);

                //Breaking the while loop at the end of the video
                if (bSuccess == false) 
                {
                    std::cout << "Found the end of the video" << std::endl;
                    break;
                }

                // show the frame in the created window
                imshow(window_name, frame);

                if( i == 0)
                {
                    ImagemConverter i;
                    std::string msg_64 = i.mat2str(&Dest);
                    // send_image_64base(h.socket(), msg_64);

                    // TODO: REMOVE DEBUG.
                    send_image_64base(h.socket(), redis.set("State_module_identifiant"););
                }
                i++;
                if( i > 3) i = 0;

                if (cv::waitKey(10) == 27)
                {
                    std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
                    break;
                }
            }
            cap.release();
        }

        // Sleep to avoid infinity loop.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

int main()
{
    // init connection server.
    std::string adress_server;
    get_param_data(&redis, &adress_server);
    h.connect(adress_server);
    bind_events(h.socket());

    send_robot_identifiant(&redis, h.socket(), "robot");
    send_robot_status(&redis, h.socket(), "robot_status_operator");

    thread_A = std::thread(&function_thread_A);
    thread_C = std::thread(&function_thread_C);

    thread_A.join();
    thread_C.join();
    
    return 0;
}
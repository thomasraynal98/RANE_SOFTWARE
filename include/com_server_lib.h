#include <sio_client.h>
#include <unistd.h>
#include <functional>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <map>
#include <vector>
#include <list>
#include <cstdlib>
#include <sw/redis++/redis++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void get_param_data(sw::redis::Redis* redis, std::string* adress);
void init_server_connection(sw::redis::Redis* redis, sio::socket::ptr current_socket);
void bind_events(sw::redis::Redis* redis, sio::socket::ptr current_socket);
void check_the_good_map(sw::redis::Redis* redis, sio::socket::ptr current_socket);
void active_download_map(sw::redis::Redis* redis);
void map_manual_command(sw::redis::Redis* redis, double back_value, double front_value, double angle);
void send_robot_identifiant(sw::redis::Redis* redis, sio::socket::ptr current_socket, std::string topic_name);
void send_robot_status(sw::redis::Redis* redis, sio::socket::ptr current_socket, std::string topic_name);
void send_image_64base(sio::socket::ptr current_socket, std::string base64_msg);
void send_image_64base_LCDS(sio::socket::ptr current_socket, std::string base64_msg);
void transform_in_world_ref(sw::redis::Redis* redis, double* longitude, double* latitude);

#ifndef CONNECTION_LISTERNER_H
#define CONNECTION_LISTERNER_H

class connection_listener
{   
    private:
        sio::client &_handler;
        bool connect_finish;
        std::condition_variable_any _cond;

    public:
        /* Constructor. */
        connection_listener(sio::client& h): _handler(h) {}

        /* Function. */
        void on_connected();
        void on_close(sio::client::close_reason const& reason);
        void on_fail();
        bool get_connect_finish();
        std::_V2::condition_variable_any* get_cond();
};

#endif
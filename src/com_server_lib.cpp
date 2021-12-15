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

void init_server_connection(sw::redis::Redis* redis, sio::socket::ptr current_socket)
{
    std::string adress_server;
    std::mutex _lock;
    sio::client h;
    connection_listener *l;

    get_param_data(redis, &adress_server);

    connection_listener li(h);
    l = &li;

    h.set_open_listener (std::bind(&connection_listener::on_connected, l));
    h.set_close_listener(std::bind(&connection_listener::on_close    , l, std::placeholders::_1));
    h.set_fail_listener (std::bind(&connection_listener::on_fail     , l));
    h.connect(adress_server);

    /* Stop if we are not connect. */
    _lock.lock();
    if(!l->get_connect_finish())
    {
        l->get_cond()->wait(_lock);
    }
    _lock.unlock();

    /* Inform server. */
    current_socket = h.socket();
    std::string envoie = "MK2R2_1";
    current_socket->emit("robot", envoie);
    
    /* Initialisation server listening. */
    bind_events(redis, current_socket, &_lock);
}

void connection_listener::on_connected()
{
    // _lock.lock();
    _cond.notify_all();
    connect_finish = true;
    // _lock.unlock();
}

void connection_listener::on_close(sio::client::close_reason const& reason)
{
    std::cout<<"sio closed "<<std::endl;
    exit(0);
}  

void connection_listener::on_fail()
{
    std::cout<<"sio failed "<<std::endl;
    exit(0);
}

bool connection_listener::get_connect_finish()
{
    return connect_finish;
}

std::condition_variable_any* connection_listener::get_cond()
{
    return &_cond;
}

void bind_events(sw::redis::Redis* redis, sio::socket::ptr current_socket, std::mutex* _lock)
{
    /*
        DESCRIPTION: this function store all kind of message that we can receive 
            from the main API.
    */

    /* If our current map is the good one. */
    current_socket->on("good", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock->lock();
        redis->set("State_map_validate", "true");
        _lock->unlock();
    }));

    /* If our current map is not the good one, we need to get a new one. */
    current_socket->on("download", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock->lock();
        redis->set("Param_localisation", std::to_string(data->get_map()["id"]->get_int()));
        redis->set("Param_id_current_map", data->get_map()["localisation"]->get_string());
        redis->set("Param_link_current_map_session", data->get_map()["link_session"]->get_string());
        redis->set("Param_link_current_map_png", data->get_map()["link_png"]->get_string());
        redis->set("State_map_validate", "true");
        redis->set("State_map_available", "false");
        _lock->unlock();
    }));

    /* In manual mode we need that robot do a precise command. */
    current_socket->on("command_to_do", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock->lock();
        redis->publish("command_micro", data->get_string());
        _lock->unlock();
    }));

    /* In autonav mode we need that robot reach a new point. */
    current_socket->on("position_to_reach", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock->lock();
        std::string msg_destination = std::to_string(data->get_map()["i"]->get_int()) + "/" + std::to_string(data->get_map()["j"]->get_int()) + "/";
        redis->set("State_position_to_reach", msg_destination);
        redis->set("State_need_compute_global_path", "true");
        _lock->unlock();
    }));

    /* Ping pong from API. */
    current_socket->on("ping", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        current_socket->emit("pong");
    }));
}

void recover_data_and_send_to_server(sio::socket::ptr current_socket)
{
    sio::message::ptr data_robot = sio::object_message::create();
    current_socket->emit("global_data", data_robot);
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
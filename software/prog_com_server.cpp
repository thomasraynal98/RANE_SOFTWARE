#include "com_server_lib.h"
#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <chrono>

using namespace sw::redis;

/*
    DESCRIPTION: the program will be in charge of communicate with server.
*/

auto redis = Redis("tcp://127.0.0.1:6379");
sio::socket::ptr current_socket;
std::thread thread_A, thread_B, thread_C;

void function_thread_A()
{
    // THREAD DESCRIPTION: send data to server and manage connection.

    int frequency       = 20;
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

        // state of robot.
        recover_data_and_send_to_server(current_socket);

        // map_checking validation.
        if((*redis.get("State_map_validate")).compare("false") == 0)
        {
            check_the_good_map(&redis, current_socket);
        }
        else
        {
            if((*redis.get("State_map_available")).compare("false") == 0)
            {
                active_download_map(&redis);
            }
        }


    }
}


void function_thread_C()
{
    // THREAD DESCRIPTION: download map and manage.
}

int main()
{
    // init program.
    init_server_connection(&redis, current_socket);

    // run thread.
    thread_A = std::thread(&function_thread_A);
    thread_C = std::thread(&function_thread_C);

    thread_A.join();
    thread_C.join();
    
    return 0;
}
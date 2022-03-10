#include <sw/redis++/redis++.h>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_A;
LibSerial::SerialPort* connection;

void function_thread_A()
{
    // THREAD DESCRIPTION: this thread listen REDIS

    //
    int frequency       = 5;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //

    int current_status = 1;

    while(true)
    {
        //
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //

        std::string state_slamcore = *(redis.get("State_slamcore"));

        if(state_slamcore.compare("LOST") == 0 || state_slamcore.compare("NOT_INITIALISED") == 0)
        {
            if(current_status != 1)
            {
                connection->Write("1/\n");
                current_status = 1;
            }
        }
        else
        {
            std::string robot_status = *(redis.get("State_robot"));
            if(robot_status.compare("WAITING") == 0 && current_status != 2)
            {
                connection->Write("2/\n");
                current_status = 2;
            }
            if(robot_status.compare("IN_DELIVERY") == 0 && current_status != 3)
            {
                connection->Write("3/\n");
                current_status = 3;
            }
            if(robot_status.compare("WAITING_FOR_CODE") == 0 && current_status != 4)
            {
                connection->Write("4/\n");
                current_status = 4;
            }
        }
    }
}

int main()
{
    connection = new LibSerial::SerialPort;
    connection->Open("/dev/ttyACM0");

    // run thread.
    thread_A = std::thread(&function_thread_A);
    thread_A.join();
}
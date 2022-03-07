#include <sw/redis++/redis++.h>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <thread>
#include <chrono>

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_verification, thread_lecture;

std::chrono::high_resolution_clock::time_point last_event;
int wait_until_stop = 0;
bool stop_is_send = false;


void function_thread_lecture()
{
    std::string command;

    while(true)
    {
        std::cin >> command;

        stop_is_send = false;
        int direction = -1;
        double speed = 0;

        std::string msg_micro = "";
        last_event = std::chrono::high_resolution_clock::now();

        std::string T;
        std::stringstream X(command);
        int i = 0;
        while(std::getline(X, T, '/'))
        {
            if(i == 0) 
            {
                if(T.compare("F") == 0)  direction = 1;
                if(T.compare("B") == 0)  direction = 2;
                if(T.compare("L") == 0)  direction = 3;
                if(T.compare("R") == 0)  direction = 4;
                if(T.compare("FL") == 0) direction = 5;
                if(T.compare("FR") == 0) direction = 6;
            }
            if(i == 1) 
            { 
                speed =  std::stod(T);
            }
            if(i == 2) 
            { 
                wait_until_stop = std::stoi(T);
            }
            i += 1;
        }

        // create message command_micro.
        if(direction == 1) 
        {
            msg_micro = "1/";
            for(int i = 0; i < 6; i++)
            {
                msg_micro += "1/";
                msg_micro += std::to_string(speed) + "/";
            }
        }
        if(direction == 2) 
        {
            msg_micro = "1/";
            for(int i = 0; i < 6; i++)
            {
                msg_micro += "-1/";
                msg_micro += std::to_string(speed) + "/";
            }
        }
        if(direction == 3) 
        {
            msg_micro = "1/";
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "1/";
                msg_micro += std::to_string(speed) + "/";
            }
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "-1/";
                msg_micro += std::to_string(speed) + "/";
            }
        }
        if(direction == 4) 
        {
            msg_micro = "1/";
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "-1/";
                msg_micro += std::to_string(speed) + "/";
            }
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "1/";
                msg_micro += std::to_string(speed) + "/";
            }
        }
        if(direction == 5) 
        {
            msg_micro = "1/";
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "1/";
                msg_micro += std::to_string(speed/2) + "/";
            }
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "1/";
                msg_micro += std::to_string(speed) + "/";
            }
        }
        if(direction == 6) 
        {
            msg_micro = "1/";
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "1/";
                msg_micro += std::to_string(speed) + "/";
            }
            for(int i = 0; i < 3; i++)
            {
                msg_micro += "1/";
                msg_micro += std::to_string(speed/2) + "/";
            }
        }

        if(direction > 0 && direction <= 6) redis.publish("command_micro", msg_micro);
    }
}

void function_thread_verification()
{
    //
    int frequency       = 20;
    double time_of_loop = 1000/frequency;                          // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //

    std::chrono::duration<double, std::milli> time_span;
    
    while(true)
    {
        //
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //

        auto now  = std::chrono::high_resolution_clock::now();
        time_span = now - last_event;

        if((int)time_span.count() > wait_until_stop && !stop_is_send)
        {
            redis.publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");
            stop_is_send = true;
        }
    }
}

int main()
{
    thread_lecture = std::thread(&function_thread_lecture);
    thread_verification = std::thread(&function_thread_verification);

    thread_lecture.join();
    thread_verification.join();
}
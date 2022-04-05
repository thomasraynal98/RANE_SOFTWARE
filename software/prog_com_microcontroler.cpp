#include <sw/redis++/redis++.h>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <thread>
#include <chrono>

#include <com_micro_lib.h>

using namespace sw::redis;

/*
    DESCRIPTION: the program will be in charge of communicate with teensy microcontroler and send command
        data to it and retrieve data from encoder.
*/

auto redis = Redis("tcp://127.0.0.1:6379");
LibSerial::SerialPort* connection;
std::string last_command_motor = "";
std::string last_command_motor_micro = "";
std::thread thread_A, thread_B, thread_C, thread_D;
std::chrono::high_resolution_clock::time_point ping_time = std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point pong_time = std::chrono::high_resolution_clock::now();

void callback_command(std::string channel, std::string msg)
{
    if(msg[0] == '1')
    {
        last_command_motor = msg + "\n";
    }
}

void function_thread_A()
{
    // THREAD DESCRIPTION: ping and connection manager.

    //
    int frequency       = 2;
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
        
        try
        {
            if(connection != NULL)
            {
                send_ping_micro(connection, &ping_time);
            }
            else
            {
                connection = found_micro_port(1,true);
            }
        } catch(...) {}
    }
}

void function_thread_B(sw::redis::Subscriber* sub)
{
    // THREAD DESCRIPTION: subscribe REDIS.
    while(true)
    {
        sub->consume();
    }
}

void function_thread_C()
{
    // THREAD DESCRIPTION; listen microcontroler, pong and publish REDIS.

    auto now  = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;

    auto next = std::chrono::high_resolution_clock::now();
    std::string reponse;
    char stop = '\n';   
    // const unsigned int msTimeout = 100; // 50Hz

    while(true)
    {
        try
        {
            if(connection != NULL && connection->IsOpen())
            {
                connection->ReadLine(reponse, stop);
                if(reponse.size() > 0)
                {
                    if(reponse[0] == '0')
                    {
                        // pong reception.
                        pong_time = std::chrono::high_resolution_clock::now();
                    }
                    if(reponse[0] == '1')
                    {
                        // pong motor reception.
                        if((last_command_motor).compare(reponse) == 0)
                        {
                            last_command_motor_micro = reponse;
                        }
                    }
                    if(reponse[0] == '2')
                    {
                        // encoder data reception.
                        publish_raw_data_encoder(&redis, reponse);
                    }
                }

                // check pong last reception.
                now  = std::chrono::high_resolution_clock::now();
                time_span = now - pong_time;
                if((int)time_span.count() > 700)
                {
                    // we don't receive ping/pong.
                    // connection->Close();
                    // connection = NULL;
                }
                
            }
            else
            {
                next = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(500);
                std::this_thread::sleep_until(next);
                pong_time = std::chrono::high_resolution_clock::now();
            }
        } catch(...) {}
    }
}

void function_thread_D()
{
    // THREAD DESCRIPTION: send command to micro.

    //
    int frequency       = 10;
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

        if(last_command_motor_micro.compare(last_command_motor) != 0 && connection != NULL)
        {
            try{ send_command_micro(connection, last_command_motor);}
            catch(...) {}
        }
    }
}

int main()
{
    auto sub = redis.subscriber();
    sub.on_message(callback_command);
    sub.subscribe("command_micro");

    // detect teensy port.
    connection = found_micro_port(1,true);
    std::cout << "FOUND CONNECTION:" << connection <<  std::endl;
    
    // run thread.
    thread_A = std::thread(&function_thread_A);
    thread_B = std::thread(&function_thread_B, &sub);
    thread_C = std::thread(&function_thread_C);
    thread_D = std::thread(&function_thread_D);

    thread_A.join();
    thread_B.join();
    thread_C.join();
    thread_D.join();

    return 0;
}
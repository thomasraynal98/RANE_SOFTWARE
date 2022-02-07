#include <sw/redis++/redis++.h>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <thread>
#include <chrono>

#include <com_micro_lib.h>

using namespace sw::redis;

auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_A, thread_B, thread_C;
LibSerial::SerialPort* connection;
std::chrono::high_resolution_clock::time_point ping_time, pong_time;


void function_thread_A()
{
    /* DESCRIPTION:
        1. LISTEN SERIAL PORT INFORMATION FROM MODULE.
        2. PUBLISH THIS INFORMATION ON REDIS RAM MEMORY.
    */

    //! CHRONO TIMER VARIABLE
    int frequency       = 20;                              // en Hz.
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //! CHRONO TIMER VARIABLE

    std::string reponse;
    char stop = '\n'; 
    auto now  = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    int time_before_disconected = 5000;                    // en milliseconde.

    while(true)
    {
        //! CHRONO TIMER VARIABLE
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //! CHRONO TIMER VARIABLE

        if(connection != NULL)
        {
            if(connection->IsOpen())
            {
                //! READ MESSAGE IF AVAILABLE.
                connection->ReadLine(reponse, stop);

                if(reponse.size() > 0)
                {
                    if(reponse[0] == '0')
                    {
                        // pong reception.
                        pong_time = std::chrono::high_resolution_clock::now();

                        std::string pong_message = "0/";
                        connection->Write(pong_message);
                    }
                    if(reponse[0] == '1')
                    {
                        // information reception.
                        get_module_information(&redis, reponse);
                    }
                }

                //! CHECK IF THE BASE IS ALWAYS CONNECTED.
                now  = std::chrono::high_resolution_clock::now();
                time_span = now - pong_time;
                if((int)time_span.count() > time_before_disconected)
                {
                    // we don't receive ping/pong.
                    connection->Close();
                    connection = NULL;
                }
            }
        }
    }
}

void function_thread_B()
{
    /* DESCRIPTION:
        1. READ REDIS RAM MEMORY INFORMATION.
        2. PUBLISH THIS INFORMATION THROUGH SERIAL PORT TO MODULE.
    */

    //! CHRONO TIMER VARIABLE
    int frequency       = 10;                              // en Hz.
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //! CHRONO TIMER VARIABLE

    while(true)
    {
        //! CHRONO TIMER VARIABLE
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //! CHRONO TIMER VARIABLE

        if(connection != NULL)
        {
            inform_module(&redis, connection);
        }
    }
}

void function_thread_C()
{
    /* DESCRIPTION:
        1. MANAGE SERIAL CONNECTION.
        2. FOUND GOOD SERIAL PORT NAME.
        3. PING MODULE.
    */

    //! CHRONO TIMER VARIABLE
    int frequency       = 5;                               // en Hz.
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //! CHRONO TIMER VARIABLE

    while(true)
    {
        //! CHRONO TIMER VARIABLE
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //! CHRONO TIMER VARIABLE

        if(connection != NULL)
        {
            // PING ROBOT.
            send_ping_module(connection, &ping_time);
            if(connection != NULL) 
            {
                redis.set("State_connection_base", "AVAILABLE");
            }
        }
        else
        {
            // FOUND NEW CONNECTION.
            redis.set("State_connection_base", "NO_CONNECTION");
            connection = found_micro_port(0,true);
        }
    }

}

int main()
{
    // run thread.
    thread_A = std::thread(&function_thread_A);
    thread_B = std::thread(&function_thread_B);
    thread_C = std::thread(&function_thread_C);
    thread_A.join();
    thread_B.join();
    thread_C.join();

    return 0;
}
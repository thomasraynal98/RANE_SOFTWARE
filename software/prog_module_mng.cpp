#include <sw/redis++/redis++.h>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <sys/stat.h>

#include <com_micro_lib.h>

using namespace sw::redis;

auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_A, thread_B, thread_C;
std::chrono::high_resolution_clock::time_point ping_time, pong_time;
std::ofstream usbWrite, usbWrite2;
std::ifstream usbRead;


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

    char reponse[20];
    char c;
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

        if(usbWrite.is_open() && usbRead.is_open())
        {
            //! READ MESSAGE IF AVAILABLE.
            // usbRead.get(reponse);
            int i = 0;
            while(usbRead.get(c))
            {
                reponse[i] = c;
                i++;
                if(c == '\n') break;
            }

            if(reponse[0] == '0')
            {
                // pong reception.
                pong_time = std::chrono::high_resolution_clock::now();

                std::string pong_message = "0/HELLO_BASE\r\n";
                usbWrite << pong_message;
            }
            if(reponse[0] == '1')
            {
                // information reception.
                get_module_information(&redis, reponse);
            }

            //! CHECK IF THE BASE IS ALWAYS CONNECTED.
            // now  = std::chrono::high_resolution_clock::now();
            // time_span = now - pong_time;
            // if((int)time_span.count() > time_before_disconected)
            // {
            //     // we don't receive ping/pong.
            //     is_open = false;
            //     usbWrite.close();
            //     usbRead.close();
            // }
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

        if(usbWrite.is_open() && usbRead.is_open())
        {
            // inform_module(&redis, usbWrite);
        }
    }
}

// bool fileExists(const std::string& filename)
// {
//     struct stat buf;
//     if (stat(filename.c_str(), &buf) != -1)
//     {
//         return true;
//     }
//     return false;
// }

void function_thread_C()
{
    /* DESCRIPTION:
        1. MANAGE SERIAL CONNECTION.
        2. FOUND GOOD SERIAL PORT NAME.
        3. PING MODULE.
    */

    //! CHRONO TIMER VARIABLE
    int frequency       = 2;                               // en Hz.
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    int timeSleep       = time_of_loop*1000;
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //! CHRONO TIMER VARIABLE

    while(true)
    {
        // usbWrite.open("/dev/ttyUSB0");
        //! CHRONO TIMER VARIABLE
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //! CHRONO TIMER VARIABLE

        if(fileExists("/dev/ttyUSB0")){
            usbWrite.open("/dev/ttyUSB0");
            std::cout << "USB0 is open." << std::endl;
            usbWrite << "0/HELLO_BASE\r\n";
            usbWrite.flush();}
        else{
            usbWrite.close();
            std::cout << "USB0 is close." << std::endl;}

        if(fileExists("/dev/ttyUSB1")){
            usbWrite2.open("/dev/ttyUSB1");
            std::cout << "USB1 is open." << std::endl;
            usbWrite2 << "0/HELLO_BASE2\r\n";
            usbWrite2.flush();}
        else{
            usbWrite2.close();
            std::cout << "USB1 is close." << std::endl;}

        // if(usbWrite.is_open() && usbRead.is_open())
        // {
        //     std::cout << "USE IT" << std::endl;
        //     // PING ROBOT.
        //     send_ping_module(usbWrite, &ping_time);
        //     redis.set("State_connection_base", "AVAILABLE");
        // }
        // else
        // {
        //     // FOUND NEW CONNECTION.
        //     std::cout << "OPEN IT" << std::endl;
        //     redis.set("State_connection_base", "NO_CONNECTION");
        //     usbWrite.open("/dev/ttyUSB0");
        //     usbRead.open("/dev/ttyUSB0");
        // }
        // if(!(usbWrite.is_open())) {usbWrite.close(); std::cout << "CLOSE" << std::endl;}
        
        std::cout << "THREAD(3) is running.\n" << std::endl;
    }

}

int main()
{
    usbRead.close();
    // usbRead.open("/dev/ttyUSB0");
    // run thread.usbRead.open("/dev/ttyUSB0");
    // thread_A = std::thread(&function_thread_A);
    // thread_B = std::thread(&function_thread_B);

    thread_C = std::thread(&function_thread_C);
    // thread_A.join();
    // thread_B.join();
    thread_C.join();

    return 0;
}
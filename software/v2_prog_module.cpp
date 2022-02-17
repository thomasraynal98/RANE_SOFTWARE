//! THIS V2 PROGRAM FOR MODULE MANAGEMENT HAS FOR OBJECTIF TO WORK.
//! SIMPLE - SPEED - NO SERIAL LIBRARY - OLD SCHOOL

#include <sw/redis++/redis++.h>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <fstream>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include <com_micro_lib.h>

using namespace sw::redis;

auto redis = Redis("tcp://127.0.0.1:6379");
std::thread thread_A, thread_B;
std::ifstream usbRead;
std::ofstream usbWrite;


LibSerial::SerialStream my_serial_stream;

void function_thread_A()
{
    /*
        DESCRIPTION:
            1. Talk with other device.
    */
    //! CHRONO TIMER VARIABLE
    int frequency       = 2;                              // en Hz.
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    //! CHRONO TIMER VARIABLE

    // bool is_open = false;

    while(true)
    {
        //! CHRONO TIMER VARIABLE
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        //! CHRONO TIMER VARIABLE

        if(fileExists("/dev/ttyModule"))
        {
            std::cout << "Port /dev/ttyModule is Open." << std::endl;
            std::string reponse;
            char reponsec[100];
            char stop = '\n'; 
            
            while(true)
            {
                std::streamsize taille = 100;
                my_serial_stream.getline(reponsec, taille, '\n');
                std::cout << "[FROM RASP]> " << reponsec << std::endl;
            }
        }
        // if(fileExists("/dev/ttyModule"))
        // {
        //     usbWrite.open("/dev/ttyModule");
        //     std::cout << "ttyModule is open." << std::endl;
        //     usbWrite << format_msg_for_delivery_module(&redis);
        //     usbWrite.flush();
        //     usbWrite.close();
        // }
        // else
        // {
        //     usbWrite.close();
        //     std::cout << "ttyModule is close." << std::endl;
        // }
        // std::cout << std::endl;
    }
}
void function_thread_B()
{
    /*
        DESCRIPTION:
            1. Listen other device.
    */
    //! NO TIME FREQUENCY.
    // char c;
    // bool is_open = false;
    int i = 0;
    while(true)
    {
        i++;
        char msg[] = "FROM PC.";
        if(i >= 10) msg[0] = 'O';
        usleep(1000000);
        std::streamsize taille = 100;
        my_serial_stream.FlushOutputBuffer();
        my_serial_stream.write(msg, taille);
        std::cout << "[EMIT FROM PC]" << msg << std::endl;
        // if(fileExists("/dev/ttyModule"))
        // {
        //     if(!is_open) {usbRead.open("/dev/ttyModule"); is_open = true;}
        //     char buffer[70];
        //     int i = 0;
        //     while(usbRead.get(c))
        //     {
        //         buffer[i] = c;
        //         i++;
        //         if(c == '\n') break;
        //     }
        //     char message[i-1];
        //     for(int j = 0; j < i-1; j++)
        //     {
        //         message[j] = buffer[i];
        //     }
        //     std::cout << "[RECEPTION_MSG]>" << message << std::endl;
        // }
        // else
        // {
        //     is_open = false;
        //     usbRead.close();
        // }
    }
}

int main()
{
    my_serial_stream.Open("/dev/ttyModule");
    my_serial_stream.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    my_serial_stream.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    my_serial_stream.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    my_serial_stream.SetParity(LibSerial::Parity::PARITY_ODD);

    // thread_A = std::thread(&function_thread_A);
    thread_B = std::thread(&function_thread_B);

    // thread_A.join();
    thread_B.join();

    return 0; 
}
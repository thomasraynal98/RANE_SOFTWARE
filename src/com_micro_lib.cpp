#include "com_micro_lib.h"
#include <sw/redis++/redis++.h>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <cstdlib>
#include <unistd.h>
#include <chrono>


LibSerial::SerialPort* found_micro_port(int debug_mode, bool wait_option)
{
    std::string message = "0/A\n";

    for (int i=0; i<4; i++)
    {
        LibSerial::SerialPort* serial_port = new LibSerial::SerialPort;
        std::string name_port = "/dev/ttyACM" + std::__cxx11::to_string(i);
        bool is_openable = true;

        if(debug_mode==1) {std::cout << "pointeur:" << serial_port <<"\n";}

        try{ serial_port->Open(name_port);}
        catch (LibSerial::AlreadyOpen ex)
        {
            if(debug_mode==1) {std::cout << "SerialPort already open : " << name_port << std::endl;}
            is_openable = false;
        }
        catch (LibSerial::OpenFailed ex)
        {
            if(debug_mode==1) {std::cout << "Failed to open SerialPort : " << name_port << std::endl;}
            is_openable = false;
        }

        if(is_openable)
        {
            if(wait_option){ usleep(500000);}

            if(debug_mode==1) {std::cout << "Succes to open SerialPort : " << name_port << std::endl;}
            if(debug_mode==1) {std::cout << "message:" << message << "\n";}
            
            try{ serial_port->Write(message);}
            catch(std::runtime_error ex) { std::cout << "nop\n"; }
            
            std::string reponse;
            serial_port->ReadLine(reponse);
            if(debug_mode==1) {std::cout << "reponse:" << reponse;}

            if(message.compare(reponse) == 0)
            {   
                // TODOEND.
                std::cout << "[MATCH]" << serial_port << std::endl;
                return serial_port;
            }
            else{serial_port->Close(); std::cout << "[CLOSE_PORT]" << std::endl;}
        }     
    }

    return NULL;
}

void send_ping_micro(LibSerial::SerialPort* connection, std::chrono::high_resolution_clock::time_point* ping_time)
{
    std::string ping_message = "0/A";
    connection->Write(ping_message);
    *ping_time = std::chrono::high_resolution_clock::now();
}

void send_command_micro(LibSerial::SerialPort* connection, std::string motor_message)
{
    connection->Write(motor_message);
}

void publish_raw_data_encoder(sw::redis::Redis* redis, std::string encoder_message)
{
    redis->publish("raw_data_encoder", encoder_message);
}
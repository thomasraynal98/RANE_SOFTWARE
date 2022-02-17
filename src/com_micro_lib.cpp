#include "com_micro_lib.h"
#include <sw/redis++/redis++.h>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <cstdlib>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <sys/stat.h>


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

void send_ping_module(std::ofstream& usbWrite, std::chrono::high_resolution_clock::time_point* ping_time)
{
    std::string ping_message = "0/HELLO_BASE";
    usbWrite << ping_message;
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

void inform_module(sw::redis::Redis* redis, std::ofstream& usbWrite)
{
    /*
        State_status_order
        State_base_identifiant
    */

    std::string message = "1/";
    message += "RANE_MK3_KODA_1/" + *(redis->get("State_robot")) + "/\n";
    usbWrite << message;

    message = "2/";
    message += *(redis->get("State_order")) + "/\n";
    usbWrite << message;
}

void get_module_information(sw::redis::Redis* redis, std::string msg)
{
    /*
        msg description: 1/A/B/
        A = identifiant module "DEL_V1_1"
    */

    std::string T;
    std::stringstream X(msg);
    
    int i = 0;
    while(std::getline(X, T, '/'))
    {
        if(i == 1) { redis->set("State_module_identifiant", T);}
        i += 1;
    }
}

bool fileExists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}

std::string format_msg_for_delivery_module(sw::redis::Redis* redis)
{
    std::string message = "1/";
    message += *(redis->get("State_base_identifiant")) + "/" + *(redis->get("State_robot")) + "/" \
    + *(redis->get("State_order")) + "/\n";
    return message;
}

void send_information_to_module(sw::redis::Redis* redis, LibSerial::SerialPort* connection)
{
    /*
        msg description: 1/A/B/C/
        A = State_base_identifiant "RANE_MK3_KODA_1"
        B = State_robot "WAITING" "IN_DELIVERY" "REACH_DESTINATION"
        C = State_order "OPEN" "CLOSE" "LOCK" "UNLOCK"
    */

    std::string message = "1/";
    message += *(redis->get("State_base_identifiant")) + "/" + *(redis->get("State_robot")) + "/" \
    + *(redis->get("State_order")) + "/\n";

    connection->Write(message);
}

void read_module(sw::redis::Redis* redis, std::string msg)
{
    /*
        msg description: 1/A
        A = State_module_identifiant
    */

    std::string T;
    std::stringstream X(msg);
    
    int i = 0;
    while(std::getline(X, T, '/'))
    {
        if(i == 1) { redis->set("State_module_identifiant", T);}
        i += 1;
    }
}
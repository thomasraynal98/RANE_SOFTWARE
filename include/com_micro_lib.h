#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>
#include <sw/redis++/redis++.h>


LibSerial::SerialPort* found_micro_port(int debug_mode, bool wait_option);
void send_ping_micro(LibSerial::SerialPort* connection, std::chrono::high_resolution_clock::time_point* ping_time);
void send_ping_module(LibSerial::SerialPort* connection, std::chrono::high_resolution_clock::time_point* ping_time);
void send_command_micro(LibSerial::SerialPort* connection, std::string motor_message);
void publish_raw_data_encoder(sw::redis::Redis* redis, std::string encoder_message);
void inform_module(sw::redis::Redis* redis, LibSerial::SerialPort* connection);
void get_module_information(sw::redis::Redis* redis, std::string msg);

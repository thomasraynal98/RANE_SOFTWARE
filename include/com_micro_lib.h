#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>
#include <sw/redis++/redis++.h>
#include <sys/stat.h>


LibSerial::SerialPort* found_micro_port(int debug_mode, bool wait_option);
void send_ping_micro(LibSerial::SerialPort* connection, std::chrono::high_resolution_clock::time_point* ping_time);
void send_ping_module(std::ofstream &usbWrite, std::chrono::high_resolution_clock::time_point* ping_time);
void send_command_micro(LibSerial::SerialPort* connection, std::string motor_message);
void publish_raw_data_encoder(sw::redis::Redis* redis, std::string encoder_message);
void inform_module(sw::redis::Redis* redis, std::ofstream &usbWrite);
void get_module_information(sw::redis::Redis* redis, std::string msg);
bool fileExists(const std::string& filename);
std::string format_msg_for_delivery_module(sw::redis::Redis* redis);
void send_information_to_module(sw::redis::Redis* redis, LibSerial::SerialPort* connection);
void read_module(sw::redis::Redis* redis, std::string msg);
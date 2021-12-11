#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>


LibSerial::SerialPort* found_micro_port(int debug_mode, bool wait_option);
void send_ping_micro(LibSerial::SerialPort* connection, std::chrono::high_resolution_clock::time_point* ping_time);
void send_command_micro(LibSerial::SerialPort* connection, std::string motor_message);
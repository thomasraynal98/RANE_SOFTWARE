#include <sw/redis++/redis++.h>
#include <iostream>

#include <manager_lib.h>

using namespace sw::redis;

/*
    DESCRIPTION: the program will be in charge of reading and writing the param data of the robot.
        it will organize the data in the robot.
*/
    
auto redis = Redis("tcp://127.0.0.1:6379");

void callback_command(std::string channel, std::string msg)
{
    if(msg.compare("save_all") == 0)
    {
        write_ID_file(&redis);
        write_PARAM_file(&redis);
        write_MAP_file(&redis);
    }
}

int main()
{    
    if(!read_ID_file_and_push(&redis))
    {
        // inform REDIS for this error.
    }
    if(!read_MAP_file_and_push(&redis))
    {
        // inform REDIS for this error.
    }
    if(!read_PARAM_file_and_push(&redis))
    {
        // inform REDIS for this error.
    }

    auto sub = redis.subscriber();
    sub.on_message(callback_command);
    sub.subscribe("command_data_manager");

    while (true) 
    {
        sub.consume();
    }

    return 0;
}
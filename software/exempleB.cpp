#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>

using namespace sw::redis;

void print_message(std::string channel, std::string msg) {
    std::cout << "received message: " << msg  << " from channel: " << channel << std::endl;
}

int main()
{
    auto redis = Redis("tcp://127.0.0.1:6379");

    // Create a Subscriber.
    auto sub = redis.subscriber();

    sub.on_message(print_message);
    sub.subscribe("channel1");

    while (true) {
    try {
        sub.consume();
        } catch (const Error &err) {
        // Handle exceptions.
        }
    }

}
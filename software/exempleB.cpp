#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace sw::redis;

void print_message(std::string channel, std::string msg) {
    std::cout << "received message: " << msg  << " from channel: " << channel << std::endl;
}

void test(std::vector<int>& v)
{
    // for(int i = 0; i < v.size(); i++)
    // {
    //     v[i] = 0;
    // }
    for(auto &a : v)
    {
        a = 0;
    }
}
int main()
{
    //! START
    // std::vector<int> v;
    // v.push_back(90);
    // v.push_back(100);
    // test(v);
    
    // for(auto a : v) std::cout << a << std::endl;
    //! END
    // auto redis = Redis("tcp://127.0.0.1:6379");

    // // Create a Subscriber.
    // auto sub = redis.subscriber();

    // sub.on_message(print_message);
    // sub.subscribe("channel1");

    // while (true) {
    // try {
    //     sub.consume();
    //     } catch (const Error &err) {
    //     // Handle exceptions.
    //     }
    // }

    //! NEW
    cv::Mat LCDS_compute(50, 100, CV_8UC1, cv::Scalar(50)); // Ligne, Colonnes
    cv::circle(LCDS_compute, cv::Point((int)(99),(int)(0)),0, cv::Scalar(10), cv::FILLED, 0,0); // Colones ligne
    std::cout << (int)LCDS_compute.at<uchar>( cv::Point(99,0)) << std::endl;

    return 0;

}
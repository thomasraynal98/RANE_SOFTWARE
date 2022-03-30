#ifndef GLOBAL_PATH_LIB_H
#define GLOBAL_PATH_LIB_H

#include <sw/redis++/redis++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stack>
#include <slamcore/slamcore.hpp>

typedef std::pair<int, int> Pair;

struct cell {
    /*
        DESCRIPTION: this structure is using for A* algorythm.
    */

    // Row and Column index of its parent
    Pair parent;
    // f = g + h + t
    double f, g, h, t;
    cell()
        : parent()
        , f(0)
        , g(0)
        , h(0)
        , t(0)
    {
    }
};

typedef std::tuple<double, int, int> Tuple;

bool check_redis_variable(sw::redis::Redis* redis);
void compute_global_path(sw::redis::Redis* redis, cv::Mat* grid);
bool check_if_map_is_ready(sw::redis::Redis* redis);
void import_map_png(sw::redis::Redis* redis, cv::Mat* grid);
bool aStarSearch(cv::Mat grid, Pair& src, Pair& dest, sw::redis::Redis* redis, int local_option, std::vector<Pair>* keypoint_global_path);
bool isValid(cv::Mat grid, const Pair& point);
double calculateHValue(const Pair src, const Pair dest);
bool isDestination(const Pair& position, const Pair& dest);
bool isUnBlocked(cv::Mat grid, const Pair& point);
void from_global_path_to_keypoints_path(std::stack<Pair> Path, sw::redis::Redis* redis, int local_option, std::vector<Pair>* keypoint_global_path);
Pair get_destination_position(sw::redis::Redis* redis);
Pair get_current_position(sw::redis::Redis* redis);
void send_keypoint_global_path(sw::redis::Redis* redis, std::vector<Pair>* keypoint_global_path);
bool found_new_src(cv::Mat grid, const Pair& point, Pair* new_src);
void nicolas_test_function(sw::redis::Redis* redis, double back_value, double front_value, double angle);

#endif
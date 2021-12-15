#include <sw/redis++/redis++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stack>

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
cv::Mat* import_map_png(sw::redis::Redis* redis);
std::vector<Pair>* aStarSearch(cv::Mat grid, Pair& src, Pair& dest, sw::redis::Redis* redis);
bool isValid(cv::Mat grid, const Pair& point);
double calculateHValue(const Pair src, const Pair dest);
bool isDestination(const Pair& position, const Pair& dest);
bool isUnBlocked(cv::Mat grid, const Pair& point);
std::vector<Pair>* from_global_path_to_keypoints_path(std::stack<Pair> Path, sw::redis::Redis* redis);
Pair get_destination_position(sw::redis::Redis* redis);
Pair get_current_position(sw::redis::Redis* redis);

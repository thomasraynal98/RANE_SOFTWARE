#include "local_path_lib.h"

std::vector<Pair> format_lidar_data(std::string raw_msg)
{
    std::vector<Pair> lidar_data;

    std::string T;
    std::stringstream X(raw_msg);
    
    int i = 0;
    double value;

    while(std::getline(X, T, ','))
    {
        if(i % 2 == 0) { value  = std::stod(T);}
        if(i % 2 == 1) 
        {
            Pair lidar_push;
             /* Project brut data into lidar referencial grid. */
            lidar_push.first  = sin(value)*std::stod(T)*40.0/4.0+40.0;
            lidar_push.second = cos(value)*std::stod(T)*40.0/4.0+40.0;
            std::cout << "value:" << lidar_push.first << " " << lidar_push.second << " et " << -value << " " << T << "\n";
            lidar_data.push_back(lidar_push); 
        }
        i += 1;
    }
    std::cout << "size:" << lidar_data.size() << "\n";
    return lidar_data;
}

bool check_process_LCDS(sw::redis::Redis* redis)
{
    if(((*(redis->get("State_map_validate"))).compare("true") == 0) && \
    ((*(redis->get("State_map_available"))).compare("true") == 0) && \
    ((*(redis->get("State_is_autonomous"))).compare("true") == 0) && \
    ((*(redis->get("State_global_path_is_computing"))).compare("false") == 0) && \
    ((*(redis->get("State_slamcore"))).compare("OK") == 0))
    {
        return true;
    }
    return false;
}

cv::Mat show_local_environnement(cv::Mat grid, std::vector<Pair> data_lidar)
{
    // add lidar data on copy.
    for(auto data : data_lidar)
    {
        cv::circle(grid, cv::Point((int)(data.first),(int)(data.second)),2, cv::Scalar(255,192,48), cv::FILLED, 0,0);
    }
    for(auto data : data_lidar)
    {
        cv::circle(grid, cv::Point((int)(data.first),(int)(data.second)),0, cv::Scalar(255,0,0), cv::FILLED, 0, 0);
    }

    // print robot.
    for(int i = 36; i < 43; i++)
    {cv::circle(grid, cv::Point((int)(i),(int)(39)),0, cv::Scalar(50,50,50), cv::FILLED, 0, 0);}
    cv::circle(grid, cv::Point((int)(39),(int)(39)),0, cv::Scalar(0,0,255), cv::FILLED, 0, 0);

    cv::namedWindow("Local_env",cv::WINDOW_AUTOSIZE);
    cv::resize(grid, grid, cv::Size(0,0),10.0,10.0,6);
    cv::imshow("Local_env", grid);

    char d=(char)cv::waitKey(25);
    // if(d==27)
    //     break;

    return grid;
}
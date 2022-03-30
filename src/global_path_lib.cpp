#include "global_path_lib.h"
#include <tuple>
#include <iostream>
#include <stack>
#include <string>  
#include <vector>
#include <sstream>
#include <sw/redis++/redis++.h>

bool check_redis_variable(sw::redis::Redis* redis)
{
    if(((*(redis->get("State_map_validate"))).compare("true") == 0) && \
    ((*(redis->get("State_map_available"))).compare("true") == 0) && \
    ((*(redis->get("State_need_compute_global_path"))).compare("true") == 0) && \
    ((*(redis->get("State_slamcore"))).compare("OK") == 0))
    {
        return true;
    }
    return false;
}

bool check_if_map_is_ready(sw::redis::Redis* redis)
{
    if(((*(redis->get("State_map_validate"))).compare("true") == 0) && \
    ((*(redis->get("State_map_available"))).compare("true") == 0))
    {
        return true;
    }
    return false;
}

void import_map_png(sw::redis::Redis* redis, cv::Mat* grid)
{   
    std::string link = "../data_software/map/" + *(redis->get("Param_localisation")) + "_" + *(redis->get("Param_id_current_map")) + ".png";
    cv::Mat map_weighted = cv::imread(link, cv::IMREAD_GRAYSCALE);
    if(!map_weighted.empty())
    {
        *grid = map_weighted;
    }
}

void compute_global_path(sw::redis::Redis* redis, cv::Mat* grid)
{
    // stop robot for this procedure.
    redis->publish("command_micro", "1/0/7/0/7/0/7/0/7/0/7/0/7/");

    // compute global path.
    Pair current = get_current_position(redis);
    Pair target = get_destination_position(redis);
    std::vector<Pair> keypoint_global_path;

    aStarSearch(*grid, current, target, redis, 0, &keypoint_global_path);

    if(keypoint_global_path.size() > 2)
    {
        send_keypoint_global_path(redis, &keypoint_global_path);
        redis->set("State_need_compute_global_path", "false");
        redis->set("State_robot", "IN_DELIVERY");
    }
    else
    {
        redis->set("State_global_path", "no_path");
        redis->set("State_need_compute_global_path", "error");
        redis->set("State_robot", "WAITING");
    }
}

bool aStarSearch(cv::Mat grid, Pair& src, Pair& dest, sw::redis::Redis* redis, int local_option, std::vector<Pair>* keypoint_global_path)
{

	// If the source is out of range
	if (!isValid(grid, src)) {
		printf("Source is invalid\n");
		return false;
	}
    
	// If the destination is out of range
	if (!isValid(grid, dest)) {
		printf("Destination is invalid\n");
		return false;;
	}

	// Either the source or the destination is blocked
	if (!isUnBlocked(grid, src)) {
		printf("Source is blocked\n");
        Pair new_src = {-1,-1};
        if(found_new_src(grid, src, &new_src))
        {
            src.first = new_src.first;
            src.second = new_src.second;
            printf("Alternatif source found\n");
        }
        else
        {
            printf("No new source found\n");
            return false;
        }
	}
	if (!isUnBlocked(grid, dest)) {
		printf("Dest is blocked\n");
		return false;;
	}

	// If the destination cell is the same as source cell
	if (isDestination(src, dest)) {
		printf("We are already at the destination\n");
		return false;;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	bool closedList[grid.cols][grid.rows];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details
	// of that cell
    // constexpr auto p = static_cast<int>(grid.cols);
    // const int pp = grid.rows;
	// array<array<cell, COL>, ROW> cellDetails;
    int cols = grid.cols;
    int rows = grid.rows;
    
    std::vector<std::vector<cell>> cellDetails(cols, std::vector<cell>(rows));

	int i, j;
	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
    cellDetails[i][j].t = 0.0;
	cellDetails[i][j].parent = { i, j };

	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	This open list is implenented as a set of tuple.*/
	std::priority_queue<Tuple, std::vector<Tuple>,std::greater<Tuple> >openList;


	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.emplace(0.0, i, j);


	// We set this boolean value as false as initially
	// the destination is not reached.
    // high_resolution_clock::time_point t1 = high_resolution_clock::now();
	while (!openList.empty()) {

		const Tuple& p = openList.top();
		// Add this vertex to the closed list
		i = std::get<1>(p); // second element of tupla
		j = std::get<2>(p); // third element of tupla
        Pair parent(i,j);
		// Remove this vertex from the open list
		openList.pop();
		closedList[i][j] = true;
		/*
				Generating all the 8 successor of this cell
						N.W N N.E
						\ | /
						\ | /
						W----Cell----E
								/ | \
						/ | \
						S.W S S.E

				Cell --> Popped Cell   ( i  , j  )
				N    --> North	       ( i-1, j  )
				S    --> South	       ( i+1, j  )
				E    --> East	       ( i  , j+1)
				W    --> West	       ( i  , j-1)
				N.E  --> North-East    ( i-1, j+1)
				N.W  --> North-West    ( i-1, j-1)
				S.E  --> South-East    ( i+1, j+1)
				S.W  --> South-West    ( i+1, j-1)
		*/
		for (int add_x = -1; add_x <= 1; add_x++) {
			for (int add_y = -1; add_y <= 1; add_y++) {
				Pair neighbour(i + add_x, j + add_y);
				// Only process this cell if this is a valid one.
				if (isValid(grid, neighbour)) {
					// If the destination cell is the same as the current successor.
					if (isDestination(neighbour, dest)) 
                    {   
                        // Set the Parent of the destination cell.
						cellDetails[neighbour.first][neighbour.second].parent = { i, j };

                        /* Process the complete path. */
                        std::stack<Pair> Path;
                        int col = dest.first;
                        int row = dest.second;
                        Pair next_node = cellDetails[col][row].parent;
                        do {
                            Path.push(next_node);
                            next_node = cellDetails[col][row].parent;
                            col = next_node.first;
                            row = next_node.second;
                        } while (cellDetails[col][row].parent != next_node);
                        
                        /* Transform the brut path to keypoint path. */
                        if(local_option == 1) 
                        { 
                            from_global_path_to_keypoints_path(Path, redis, 1, keypoint_global_path);
                            return true;
                        }
                        from_global_path_to_keypoints_path(Path, redis, 0, keypoint_global_path);
						return true;
					}
					// If the successor is already on the
					// closed list or if it is blocked, then
					// ignore it. Else do the following
					else if (!closedList[neighbour.first][neighbour.second] && isUnBlocked(grid, neighbour)) 
                    {
						double gNew, hNew, fNew, tNew;

                        /* Add weight to gNew if path is in proximity area. */

                        if((int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) == 255)
						    {fNew = cellDetails[i][j].f + 1.0;}

                        //! NEW ONE (for dirty pixel)
                        if((int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) != 0 && (int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) != 50 && \
                        (int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) != 120 && (int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) != 200 && \
                        (int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) != 255 && (int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) != 75)
						    {fNew = cellDetails[i][j].f + 1.0;}
                        //! NEW ONE (highway)s
                        if((int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) == 75)
						    {fNew = cellDetails[i][j].f + 0.2;}

                        if((int)grid.at<uchar>(cv::Point(neighbour.first, neighbour.second)) == 200)
						    {fNew = cellDetails[i][j].f + 6.0;} // IMPORTANT VALUE.

                        if(add_y != 0 && add_x !=0){fNew += 0.01;}
                        fNew = fNew + calculateHValue(neighbour, parent);
                        
                        /* Compute distance from source and destination. */
                        
						// hNew = calculateHValue(neighbour, dest);
                        // tNew = calculateHValue(neighbour, src);

                        // fNew = gNew + hNew + tNew;
						// If it isn’t on the open list, add
						// it to the open list. Make the
						// current square the parent of this
						// square. Record the f, g, and h
						// costs of the square cell
						//			 OR
						// If it is on the open list
						// already, check to see if this
						// path to that square is better,
						// using 'f' cost as the measure.
						if (cellDetails[neighbour.first][neighbour.second].f == 0 || cellDetails[neighbour.first][neighbour.second].f > fNew) 
                        {
                            hNew = calculateHValue(neighbour, dest);
                            tNew = 0;//calculateHValue(neighbour, src);
							openList.emplace(fNew + hNew + tNew, neighbour.first,neighbour.second);

							// Update the details of this
							// cell
							// cellDetails[neighbour.first][neighbour.second].g      = gNew;
							// cellDetails[neighbour.first][neighbour.second].h      = hNew;
							cellDetails[neighbour.first][neighbour.second].f      = fNew;
                            // cellDetails[neighbour.first][neighbour.second].t      = tNew;
							cellDetails[neighbour.first][neighbour.second].parent = { i, j };
						}
					}
				}
			}
		}
	}

	// When the destination cell is not found and the open
	// list is empty, then we conclude that we failed to
	// reach the destiantion cell. This may happen when the
	// there is no way to destination cell (due to
	// blockages)

	printf("Failed to find the Destination Cell\n");
    return false;
}

bool isValid(cv::Mat grid, const Pair& point)
{ 
    // Returns true if row number and column number is in range.
    // return (point.first >= 0) && (point.first < grid.size[1]) && (point.second >= 0) && (point.second < grid.size[0]);
    //! point.first = cols
    //! point.second = rows
    return point.first >= 0 && point.first < grid.cols && point.second >= 0 && point.second < grid.rows;
}

double calculateHValue(const Pair src, const Pair dest)
{
	// h is estimated with the two points distance formula
	return sqrt(pow((src.first - dest.first), 2.0)
				+ pow((src.second - dest.second), 2.0));
}

bool isDestination(const Pair& position, const Pair& dest)
{
	return position == dest;
}

bool isUnBlocked(cv::Mat grid, const Pair& point)
{
	// Returns true if the cell is not blocked else false
    // std::cout << ">> " << point.first << ", " << point.second << "\n";
    // std::cout << ">> " << int(grid.at<uchar>(point.second, point.first)) << "\n";

    //! ORIGINAL
	// return isValid(grid, point) && ((grid.at<uchar>(point.second, point.first) == 255) || (grid.at<uchar>(point.second,point.first) == 200));

    //! NEW ONE
    return isValid(grid, point) && ((grid.at<uchar>(cv::Point(point.first, point.second)) != 0) && (grid.at<uchar>(cv::Point(point.first, point.second)) != 50) && (grid.at<uchar>(cv::Point(point.first, point.second)) != 120));
}

bool found_new_src(cv::Mat grid, const Pair& point, Pair* new_src)
{
    /*
        DESCRIPTION: found new source if the current one is blocked.
    */

    // basic approach.
    double search_distance = 3.0; // meters.
    int search_box = (int)(search_distance*20.0); // 20 cases by 1m.
    double closest_point_distance = 9999;

    for(int i = -search_box; i < search_box; i++)
    {
        for(int j = -search_box; j < search_box; j++)
        {
            if(point.first+i >= 0 && point.first+i < grid.cols && \
              point.second+j >= 0 && point.second+j < grid.rows)
            {
                if((int)grid.at<uchar>(cv::Point(point.first+i, point.second+j)) == 200 || \
                (int)grid.at<uchar>(cv::Point(point.first+i, point.second+j)) == 255)
                {
                    if(sqrt(pow(j,2)+pow(i,2))*0.05 < closest_point_distance)
                    {
                        closest_point_distance = sqrt(pow(j,2)+pow(i,2))*0.05;
                        new_src->second = point.second+j;
                        new_src->first  = point.first+i;
                    }
                }
            }
        }
    }

    if(new_src->first == -1) 
    {
        return false;
    }
    else 
    {
        return true;
    } 
}

void from_global_path_to_keypoints_path(std::stack<Pair> Path, sw::redis::Redis* redis, int local_option, std::vector<Pair>* keypoint_global_path)
{
    /*
        DESCRIPTION: the goal of this function is to take the brute global map
            from A* algorythme, and create a usefull list of keypoint and add
            some information for navigation process like distance_KPD, validation_angle
            isTryAvoidArea and distance_validation.
    */

    double distance_between_keypoint = std::stod(*(redis->get("Param_distance_btw_kp")));

    // Transform stack into vector and compute distance from destination.
    std::vector<Pair> vector_global_path;
    std::vector<double> vector_distances_from_destination;
    bool isDestination = true;
    double the_distance_from_destination = 0;
    Pair previous_p = Path.top();

    // change order.
    while(!Path.empty())
    {
        Pair p = Path.top();
        Path.pop();

        the_distance_from_destination += calculateHValue(previous_p, p)* 0.05;

        vector_distances_from_destination.push_back(the_distance_from_destination); 

        vector_global_path.push_back(p);
        
        previous_p = p;
    }

    // for local path planning only.
    if(local_option == 1) 
    { 
        *keypoint_global_path = vector_global_path;
        return;
    }
    
    // Variable.
    bool first_keypoint = true;
    keypoint_global_path->clear();

    // Select Keypoints and compute data.
    for(int i = 0; i < vector_global_path.size(); i++)
    {   
        Pair current_keypoint;
        // OK it's the start point.
        if(i == 0)
        {   
            current_keypoint.first  = vector_global_path[i].first;
            current_keypoint.second = vector_global_path[i].second;
            // push.
            keypoint_global_path->push_back(current_keypoint);
        }
        // Ok it's the last point.
        else if(i == vector_global_path.size()-1)
        {
            current_keypoint.first  = vector_global_path[i].first;
            current_keypoint.second = vector_global_path[i].second;
            // push.
            keypoint_global_path->push_back(current_keypoint);
        }
        // Ok it's other point.
        else
        {
            if(calculateHValue(keypoint_global_path->back(), vector_global_path[i])*0.05 >= distance_between_keypoint)
            {
                current_keypoint.first  = vector_global_path[i].first;
                current_keypoint.second = vector_global_path[i].second;
                // push.
                keypoint_global_path->push_back(current_keypoint);
            }
        }
    }
    return;
}

void send_keypoint_global_path(sw::redis::Redis* redis, std::vector<Pair>* keypoint_global_path)
{
    std::string msg;
    for(auto kp : *keypoint_global_path)
    {
        msg += std::to_string(kp.first) + "/" + std::to_string(kp.second) + "/";
    }
    redis->set("State_global_path", msg);
}

Pair get_current_position(sw::redis::Redis* redis)
{
    Pair current_position;
    std::string msg = *(redis->get("State_robot_position_png"));
    std::string T;
    std::stringstream X(msg);
    
    int i = 0;
    while(std::getline(X, T, '/'))
    {
        if(i == 2) { current_position.first = std::stoi(T);}
        if(i == 3) { current_position.second = std::stoi(T);}
        i += 1;
    }
    return current_position;
}

Pair get_destination_position(sw::redis::Redis* redis)
{
    Pair current_destination_position;
    std::string msg = *(redis->get("State_position_to_reach"));
    std::string T;
    std::stringstream X(msg);
    
    int i = 0;
    while(std::getline(X, T, '/'))
    {
        if(i == 0) { current_destination_position.first = std::stoi(T);}
        if(i == 1) { current_destination_position.second = std::stoi(T);}
        i += 1;
    }
    return current_destination_position;
}
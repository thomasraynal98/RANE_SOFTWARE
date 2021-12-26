#include "global_path_lib.h"
#include "local_path_lib.h"
#include <opencv2/imgproc/imgproc.hpp>

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
            lidar_push.first  = sin(value)*std::stod(T)*80.0/4.0+80.0;
            lidar_push.second = cos(value)*std::stod(T)*80.0/4.0+80.0;
            lidar_data.push_back(lidar_push); 
        }
        i += 1;
    }
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

void show_local_environnement(cv::Mat* grid, std::vector<Pair>* data_lidar, std::vector<double>* current_speed)
{
    // add simulation +1000ms.
    cv::Point pp[1][4];
    pp[0][0] = cv::Point(73,79);
    pp[0][1] = cv::Point(85,79);
    pp[0][3] = cv::Point(73,79);
    pp[0][2] = cv::Point(85,79);

    if(!current_speed->empty())
    {
        pp[0][3] = cv::Point(73+(int)(current_speed->at(1)*20),79-(int)(current_speed->at(0)*20));
        pp[0][2] = cv::Point(85+(int)(current_speed->at(1)*20),79-(int)(current_speed->at(0)*20));
    }

    const cv::Point* ppt[1] = { pp[0] };

    int npt[] = { 4 };
    cv::fillPoly( *grid,
        ppt,
        npt,
        1,
        cv::Scalar( 0, 255, 255 ),
        0 );

    // add lidar data on copy.
    if(!(data_lidar->empty()))
    {   
        // try avoid area.
        for(auto data : *data_lidar)
        {
            cv::circle(*grid, cv::Point((int)(data.first),(int)(data.second)),12, cv::Scalar(255,242,212), cv::FILLED, 0,0);
        }
        // robot center not go in (but body can).
        for(auto data : *data_lidar)
        {
            cv::circle(*grid, cv::Point((int)(data.first),(int)(data.second)),6, cv::Scalar(255,192,48), cv::FILLED, 0,0);
        }
        // obstacle.
        for(auto data : *data_lidar)
        {
            cv::circle(*grid, cv::Point((int)(data.first),(int)(data.second)),0, cv::Scalar(255,0,0), cv::FILLED, 0, 0);
        }
    }


    // print robot.
    for(int i = 73; i < 86; i++)
    {cv::circle(*grid, cv::Point((int)(i),(int)(79)),0, cv::Scalar(50,50,50), cv::FILLED, 0, 0);}
    cv::circle(*grid, cv::Point((int)(79),(int)(79)),0, cv::Scalar(0,0,255), cv::FILLED, 0, 0);

    // show.
    cv::namedWindow("Local_env",cv::WINDOW_AUTOSIZE);
    cv::resize(*grid, *grid, cv::Size(0,0),9.0,9.0,6);
    // cv::rotate(grid, grid, 1);
    cv::imshow("Local_env", *grid);

    char d=(char)cv::waitKey(25);
    // if(d==27)
    //     break;
}

void get_robot_speed(sw::redis::Redis* redis, std::vector<double>* encoder_data)
{
    std::vector<double> encoder_data_clone;

    std::string result = *(redis->get("State_robot_speed"));

    std::string T;
    std::stringstream X(result);

    while(std::getline(X, T, '/'))
    {
        encoder_data_clone.push_back(std::stod(T));
    }

    while(encoder_data->size() != 3) { encoder_data->push_back(0.0);}

    encoder_data->at(0) = encoder_data_clone[0];
    encoder_data->at(1) = encoder_data_clone[1];
    encoder_data->at(2) = encoder_data_clone[2];

}

void get_global_path(sw::redis::Redis* redis, std::vector<Path_keypoint>* global_keypoint_path)
{
    global_keypoint_path->clear();

    std::string result = *(redis->get("State_global_path"));

    std::string T;
    std::stringstream X(result);

    // get all point.
    bool end = false;
    int coord_i = 0;
    while(std::getline(X, T, '/'))
    {
        if(!end)
        {
            coord_i = std::stoi(T);
        }
        if(end)
        {
            Path_keypoint kp;
            kp.coordinate.first = coord_i;
            kp.coordinate.second = std::stoi(T);
            global_keypoint_path->push_back(kp);
        }
        end = !end;
    }

    // fill in variable. back sens.
    Pair destination;
    std::vector<double> current_position = get_current_position_n(redis);
    for(int i = global_keypoint_path->size()-1; i >= 0; i--)
    {
        if(i == global_keypoint_path->size()-1)
        {
            // the destination KP.
            destination.first  = global_keypoint_path->at(i).coordinate.first;
            destination.second = global_keypoint_path->at(i).coordinate.second;
            global_keypoint_path->at(i).distance_KPD        = 0;
            global_keypoint_path->at(i).isTryAvoidArea      = 200;
            global_keypoint_path->at(i).distance_validation = 0.4; // arbitratry

            // non fix variable.
            global_keypoint_path->at(i).isReach             = false;
            global_keypoint_path->at(i).target_angle        = compute_target_angle(global_keypoint_path->at(i).coordinate, current_position);
            global_keypoint_path->at(i).distance_RKP        = compute_distance_RPK(global_keypoint_path->at(i).coordinate, current_position)*0.05;
        }
        if(i == 0)
        {
            // the current position.
            // fix variable.
            global_keypoint_path->at(i).distance_KPD        = compute_distance_RPK(destination, current_position)*0.05;
            // current_keypoint.isTryAvoidArea      = map_weighted.at<uchar>(current_keypoint.coordinate.first, current_keypoint.coordinate.second);
            global_keypoint_path->at(i).isTryAvoidArea  = 200;
            global_keypoint_path->at(i).distance_validation = compute_distance_validation(global_keypoint_path->at(i));
            
            // non fix variable.
            global_keypoint_path->at(i).isReach             = true;
            global_keypoint_path->at(i).target_angle        = compute_target_angle(global_keypoint_path->at(i).coordinate, current_position);
            global_keypoint_path->at(i).distance_RKP        = compute_distance_RPK(global_keypoint_path->at(i).coordinate, current_position)*0.05;
        }
        if(i != 0 && i != global_keypoint_path->size()-1)
        {
            global_keypoint_path->at(i).distance_KPD        = compute_distance_RPK(destination, current_position)*0.05;
            global_keypoint_path->at(i).isTryAvoidArea      = 200;
            global_keypoint_path->at(i).distance_validation = compute_distance_validation(global_keypoint_path->at(i));

            // non fix variable.
            global_keypoint_path->at(i).isReach             = false;
            global_keypoint_path->at(i).target_angle        = compute_target_angle(global_keypoint_path->at(i).coordinate, current_position);
            global_keypoint_path->at(i).distance_RKP        = compute_distance_RPK(global_keypoint_path->at(i).coordinate, current_position)*0.05;
        }
    }

    // fill validation angle.
    for(int i = 0; i < global_keypoint_path->size(); i++)
    {
        if(i == 0){global_keypoint_path->at(i).validation_angle = 0;}
        if(i == global_keypoint_path->size()-1){global_keypoint_path->at(i).validation_angle = 180;}
        if(i > 0 && i < global_keypoint_path->size()-1) {global_keypoint_path->at(i).validation_angle = compute_validation_angle( global_keypoint_path->at(i-1).coordinate, \
                                                                                                        global_keypoint_path->at(i).coordinate, \      
                                                                                                        global_keypoint_path->at(i+1).coordinate);}
    }
}

double compute_distance_validation(Path_keypoint kp)
{
    /*
        DESCRIPTION: this function will compute and return the distance of validation.
            this distance means the minimun distance between robot and focus points
            to be considered like reach.
        INFORMATION: this distance take in consideration, the nature of the current 
            area (try_avoid or not) and the validation angle. 
        TODO       : take also in consideration the speed in futur.
        COMPUTE    : the value is between 0.05m in worst case where precision in needed
            and 0.40m in case that don't necessite precision.
    */

    double value = 0.10;
    if(!kp.isTryAvoidArea)          { value += 0.20; }
    if(kp.validation_angle <= 90/4) { value += 0.15; }
    return value;
}

double compute_target_angle(Pair kp, std::vector<double> current_position)
{
    /*
        DESCRIPTION: this function will compute and return the angle between the 
            current orientation of robot and the angle of pose of robot and pose
            of path_keypoint pose.
        COMPUTE    : the value is between 0° deg and 180° deg only > 0.
    */

    double angle_RKP         = compute_vector_RKP(kp, current_position);
    double angle_ORIENTATION = current_position[2];

    double distance_deg      = -1;
    
    if(angle_RKP >= angle_ORIENTATION)
    {
        distance_deg         = angle_RKP - angle_ORIENTATION;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}
    }
    else
    {
        distance_deg         = angle_ORIENTATION - angle_RKP;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}    
    }

    return distance_deg;
}

double compute_distance_RPK(Pair kp, std::vector<double> current_position)
{
    /*
        DESCRIPTION: return the distance between robot en kp.
    */

    return sqrt(pow((kp.first - current_position[0]), 2.0)
            + pow((kp.second - current_position[1]), 2.0));
}

double compute_vector_RKP(const Pair& kp, std::vector<double> current_position)
{
    /*
        DESCRIPTION: compute the angle in world map referenciel of vector 
            from robot to keypoint. (like North, West, East, South)
    */
    
    double x_sum = kp.first  - current_position[0];
    double y_sum = kp.second - current_position[1];

    double angle_degree = acos((x_sum)/(sqrt(pow(x_sum, 2.0) + pow(y_sum, 2.0))));
    if(y_sum < 0)
    {
        angle_degree = (M_PI - angle_degree) + M_PI;
    }
    return angle_degree * (180/M_PI);
}

std::vector<double> get_current_position_n(sw::redis::Redis* redis)
{
    std::vector<double> current_position;

    std::string result = *(redis->get("State_robot_position_png"));

    std::string T;
    std::stringstream X(result);

    // get all point.
    int i = 0;
    while(std::getline(X, T, '/'))
    {
        if(i >= 2)
        {
            current_position.push_back(std::stod(T));
        }
        i++;
    }

    return current_position;
}

double compute_validation_angle(const Pair& kpPrev, const Pair& kpCurrent, const Pair& kpNext)
{
    /*
        DESCRIPTION: this function will compute validation angle, it's an angle form from 3 points,
            the current one, the previously and the next one.
    */
    
    double angle_RPREV   = compute_vector_RKP_2(kpPrev, kpCurrent);
    double angle_RNEXT   = compute_vector_RKP_2(kpNext, kpCurrent);
    double distance_deg  = -1;

    if(angle_RPREV >= angle_RNEXT)
    {
        distance_deg         = angle_RPREV - angle_RNEXT;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}
    }
    else
    {
        distance_deg         = angle_RNEXT - angle_RPREV;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}    
    }

    return 180 - distance_deg; 
}

double compute_vector_RKP_2(const Pair& kpCurrent, const Pair& kp2)
{
    /*
        DESCRIPTION: same as version 1 but will take two points in param.
    */

    double x_sum = kp2.first  - kpCurrent.first;
    double y_sum = kp2.second - kpCurrent.second;

    double angle_degree = acos((x_sum)/(sqrt(pow(x_sum, 2.0) + pow(y_sum, 2.0))));
    if(y_sum < 0)
    {
        angle_degree = (M_PI - angle_degree) + M_PI;
    }
    return angle_degree * (180/M_PI);
}

void select_target_keypoint(std::vector<Path_keypoint>* global_path_keypoint, Path_keypoint* target_keypoint)
{
    /*
        DESCRIPTION: this fundamentale function will take all state data 
            en compte for select the better target keypoint between all
            path_keypoint in keypoints_path vector.
        HEURISTIC  : there are multiple variable and the heuristic need 
            to be fine tunned but this is the order of importence for 
            all variable
                > 1. (YES) distance_RKP
                > 2. (YES) target_angle
                > 3. (YES) isReach  
                > 4. (YES) distance_KPD
        PS         : that can be a good feature to integrate the neural 
            network in this process. Or to integrate a variable that say
            if there are an object between robot and kp.
    */

    /* PART 1. Get all pointor from keypoint vector that are in a range
    of threshold from robot. */

    double threshold = 3.0; //in meter.
    std::vector<Path_keypoint*> possible_candidate;

    return_nearest_path_keypoint(threshold, global_path_keypoint, &possible_candidate);

    /* PART 2. We need to normalise all variable so first we get the max
    value of all variable. */
    double max_distance_RKP = 0;
    double max_distance_KPD = 0;
    for(int i = 0; i < possible_candidate.size(); i++)
    {
        if(possible_candidate[i]->distance_RKP > max_distance_RKP)
        {
            max_distance_RKP = possible_candidate[i]->distance_RKP;
        }
        if(possible_candidate[i]->distance_KPD > max_distance_KPD)
        {
            max_distance_KPD = possible_candidate[i]->distance_KPD;
        }
    }

    /* PART 3. Compute the target keypoint score of all this data. */
    std::vector<double> score_possible_candidate;

    double weight_distance_RKP = 0.3;
    double weight_target_angle = 0.7;
    double weight_distance_KPD = 0.6;
    double weight_isReach      = -0.4;

    for(int i = 0; i < possible_candidate.size(); i++)
    {
        double candidate_note    = 0;
        double note_distance_RKP = 1 - (possible_candidate[i]->distance_RKP/max_distance_RKP);
        double note_target_angle = 1 - (possible_candidate[i]->target_angle/180);
        double note_distance_KPD = 1 - (possible_candidate[i]->distance_RKP/max_distance_KPD);

        candidate_note = weight_distance_RKP*note_distance_RKP + weight_target_angle*note_target_angle + weight_distance_KPD*note_distance_KPD;
        candidate_note += weight_isReach*possible_candidate[i]->isReach;

        score_possible_candidate.push_back(candidate_note);
    }

    /* PART 4. Select the better one. */
    double max_note = 0;
    int index_candidate = 0;
    for(int i = 0; i < score_possible_candidate.size(); i++)
    {
        if(max_note < score_possible_candidate[i])
        {   
            max_note = score_possible_candidate[i];
            index_candidate = i;
        }
    }

    /* PART 5. Init the current target_keypoint. */
    target_keypoint->coordinate.first  = possible_candidate[index_candidate]->coordinate.first;
    target_keypoint->coordinate.second = possible_candidate[index_candidate]->coordinate.second;
}

void select_target_keypoint_2(std::vector<Path_keypoint>* global_path_keypoint, Path_keypoint* target_keypoint)
{
    /*
        DESCRIPTION: this function is the 2.0 version of select_target_keypoint function.
        this version is more simple and i hope more efficient.
    */

    /* PART 1. Get the nearest KP from the robot. And take the next one. */
    int index_TKP       = 9999;
    double distance_min = 9999;
    for(int i = 0; i < global_path_keypoint->size(); i++)
    {
        if(global_path_keypoint->at(i).distance_RKP < distance_min)
        {
            distance_min = global_path_keypoint->at(i).distance_RKP;
            if(i != global_path_keypoint->size()-1)
            {
                index_TKP = i + 1;
            }
            else
            {
                index_TKP = i;
            }
        }
    }

    /* PART 2. declare all KP before TKP as reach and KP after as not reach. */
    for(int i = 0; i < global_path_keypoint->size(); i++)
    {
        if(i < index_TKP) {global_path_keypoint->at(i).isReach = true; }
        else {global_path_keypoint->at(i).isReach = true; }
    }

    /* PART 3. declare new TKP. */
    target_keypoint->coordinate.first  = global_path_keypoint->at(index_TKP).coordinate.first;
    target_keypoint->coordinate.second = global_path_keypoint->at(index_TKP).coordinate.second;
    target_keypoint->distance_RKP      = global_path_keypoint->at(index_TKP).distance_RKP;
    target_keypoint->target_angle      = global_path_keypoint->at(index_TKP).target_angle;
    target_keypoint->isReach           = global_path_keypoint->at(index_TKP).isReach;
}

void return_nearest_path_keypoint(double threshold, std::vector<Path_keypoint>* global_path_keypoint, std::vector<Path_keypoint*>* possible_candidate_target_keypoint)
{
    /*
        DESCRIPTION: this function will run the keypoints_path vector
            and send back the pointer of keypoints in a distance of less
            then "threshold". If they are no point less far than the 
            threshold, we send back the pointer of the less far of all.
    */

    /* clean this vector. */
    possible_candidate_target_keypoint->clear();

    /* save the nearest kp in case of no kp is in threshold to avoid bug. */
    bool isEmpty               = true;
    Path_keypoint* nearest_kp  = NULL;
    double distance_nearest_kp = 9999;

    for(int i = 0; i < global_path_keypoint->size(); i++)
    {
        if(global_path_keypoint->at(i).distance_RKP < threshold) 
        { 
            isEmpty = false;
            possible_candidate_target_keypoint->push_back(&global_path_keypoint->at(i));
        }
        else
        {
            if(global_path_keypoint->at(i).distance_RKP < distance_nearest_kp)
            {
                /* this one is the nearest outside threshold. */
                nearest_kp          = &global_path_keypoint->at(i);
                distance_nearest_kp = global_path_keypoint->at(i).distance_RKP;
            }
        }
    }

    if(isEmpty)
    {
        possible_candidate_target_keypoint->push_back(nearest_kp);
    }
}

bool destination_reach(Path_keypoint* destination, std::vector<double> current_position)
{
    if(compute_distance_RPK(destination->coordinate, current_position)*0.05 <= 0.5)
    {
        return true;
    }
    return false;
}

void update_data(sw::redis::Redis* redis, std::vector<Path_keypoint>* global_keypoint, std::vector<double>* current_position)
{
    // update current_position
    *current_position = get_current_position_n(redis);

    // update global information.
    for(auto kp : *global_keypoint)
    {
        kp.distance_RKP = compute_distance_RPK(kp.coordinate, *current_position)*0.05;
        kp.target_angle = compute_target_angle(kp.coordinate, *current_position);
    }
}

std::vector<Pair> project_keypoint_in_lidar_referencial(std::vector<Path_keypoint>* global_keypoint, std::vector<double>* current_position, Path_keypoint* TKP)
{
    /* DESCRIPTION:
        this function will take target keypoint in 6 meters range and project then in lidar
        referencial.
    */

    /* Get keypoint in selection range. */
    double kp_selection_range = 5.0; //m
    std::vector<Path_keypoint*> keypoints_list_for_projection;

    for(int i = 0; i < global_keypoint->size(); i++)
    {
        if(global_keypoint->at(i).distance_RKP <= kp_selection_range \
        && !global_keypoint->at(i).isReach)
        {
            keypoints_list_for_projection.push_back(&global_keypoint->at(i));
        }
    }

    /* Project them in lidar referencial. The target KP to. */
    /* We need to change referencial angle. */
    return transform_angle_in_lidar_ref(keypoints_list_for_projection, current_position, TKP);
}

std::vector<Pair> transform_angle_in_lidar_ref(std::vector<Path_keypoint*> keypoints_list_for_projection, std::vector<double>* position, Path_keypoint* TKP)
{
    /* DESCRIPTION:
        this function will take a list of keypoint to project and transform the angle to be 
        project into the lidar ref grid.
    */

    /* need this 2 value to determine orientation of angle. */
    double angle_ORIENTATION = position->at(2);
    double angle_RKP = 0;

    double transform_angle;

    /* project keypoints. */
    std::vector<Pair> projected_keypoint;

    /* do all process if keypoints_list_for_projection is not empty. */
    if(keypoints_list_for_projection.size() > 0)
    {
        for(int i = 0; i < keypoints_list_for_projection.size()+1; i++)
        {
            /* init the keypoint. */
            Pair projected_kp;

            /* put the target keypoint at the end of this list. */
            /* compuute angle_RKP to now the direction, left or right. */
            if(i == keypoints_list_for_projection.size())
            {
                Pair keypoint_format_pair(TKP->coordinate.first, TKP->coordinate.second);
                angle_RKP            = compute_vector_RKP(keypoint_format_pair, *position);
            }
            else
            {
                Pair keypoint_format_pair(keypoints_list_for_projection[i]->coordinate.first, keypoints_list_for_projection[i]->coordinate.second);
                angle_RKP            = compute_vector_RKP(keypoint_format_pair, *position);
            }
            
            if( i != keypoints_list_for_projection.size() )
            {
                if(angle_ORIENTATION <= angle_RKP)
                {
                    if(angle_RKP - angle_ORIENTATION <= 180)
                    {
                        // Right 
                        angle_RKP    = (180 - keypoints_list_for_projection[i]->target_angle);
                    }
                    else
                    {
                        // Left 
                        angle_RKP    = -180 + keypoints_list_for_projection[i]->target_angle;
                    }
                }
                else
                {
                    if(angle_ORIENTATION - angle_RKP <= 180)
                    {
                        // Left 
                        angle_RKP    = -180 + keypoints_list_for_projection[i]->target_angle;
                    }
                    else
                    {
                        // Right 
                        angle_RKP    = (180 - keypoints_list_for_projection[i]->target_angle);
                    }
                }
            }
            else
            {
                if(angle_ORIENTATION <= angle_RKP)
                {
                    if(angle_RKP - angle_ORIENTATION <= 180)
                    {
                        // Right 
                        angle_RKP    = 180 - TKP->target_angle;
                    }
                    else
                    {
                        // Left 
                        angle_RKP    = (-180) + TKP->target_angle;
                    }
                }
                else
                {
                    if(angle_ORIENTATION - angle_RKP <= 180)
                    {
                        // Left 
                        angle_RKP    = (-180) + TKP->target_angle;
                    }
                    else
                    {
                        // Right 
                        angle_RKP    = 180 - TKP->target_angle;
                    }
                }
            }

            if(i == keypoints_list_for_projection.size())
            {
                /* only project the point in front of robot. */
                if(abs(TKP->target_angle) <= 90)
                {
                    /* project the keypoint into the lidar local grid referencial (LLG). */
                    projected_kp.first = sin(angle_RKP*M_PI/180)*TKP->distance_RKP*40/4+40;
                    projected_kp.second = cos(angle_RKP*M_PI/180)*TKP->distance_RKP*40/4+40;
                    projected_keypoint.push_back(projected_kp);
                }
            }
            if(i != keypoints_list_for_projection.size())
            {   
                /* only project the point in front of robot. */
                if(abs(keypoints_list_for_projection[i]->target_angle) <= 90)
                {
                    /* project the keypoint into the LLG referencial. */
                    projected_kp.first = sin(angle_RKP*M_PI/180)*keypoints_list_for_projection[i]->distance_RKP*40/4+40;
                    projected_kp.second = cos(angle_RKP*M_PI/180)*keypoints_list_for_projection[i]->distance_RKP*40/4+40;
                    projected_keypoint.push_back(projected_kp);
                }
            }
        }
    }

    return projected_keypoint;
}

bool simulation_problem(int futur_ms, cv::Mat* grid, std::vector<double>* current_speed, std::vector<Pair>* data_lidar)
{
    /*
        DESCRIPTION:
        this function will take current mouvement about robot and in 
        futur about some objectif in the local field and see if in some
        millisecond, the robot will touch this object. If it touch, it 
        will return true.

        grid_RGB_2: it's just to visualize the working matrix grid_Gray_2.
    */

    // 1. Draw on current grid the trajectory in futur_ms time.
    cv::Point pp[1][4];

    // current position of robot in LCDS reference.
    /* INFORMATION, the coordinate of the polygone is different than the
    show function, because we only want that the center of robot not go in
    lidar dead zone, not all the robot body. */
    pp[0][0] = cv::Point(78,79);
    pp[0][1] = cv::Point(80,79);

    pp[0][3] = cv::Point(78+(int)(current_speed->at(1)*20*(futur_ms/1000)),79-(int)(current_speed->at(0)*20*(futur_ms/1000)));
    pp[0][2] = cv::Point(80+(int)(current_speed->at(1)*20*(futur_ms/1000)),79-(int)(current_speed->at(0)*20*(futur_ms/1000)));

    const cv::Point* ppt[1] = { pp[0] };

    int npt[] = { 4 };
    cv::fillPoly( *grid,
        ppt,
        npt,
        1,
        cv::Scalar( 0, 255, 255 ),
        0 );

    // 2. check if we are some lidar point on this trajectory.
    bool is_on_trajectory = false;
    for(auto sample: *data_lidar)
    {
        cv::Vec3b bgrPixel = grid->at<cv::Vec3b>(sample.first, sample.second);
        if(bgrPixel.val[0] == 0 && bgrPixel.val[1] == 255 && bgrPixel.val[2] == 255)
        {
            is_on_trajectory = true;
            return is_on_trajectory;
        }
    }

    // 3. return the bool if it's false.
    return is_on_trajectory;
}

bool TKP_problem(cv::Mat* grid_RGB, Path_keypoint* TKP, std::vector<Pair>* data_lidar)
{
    /*
        DESCRIPTION:
        this function will check if the TKP is block by lidar data.
        return true if it's block.
    */

    // 1. draw it on grid.
    for(auto sample: *data_lidar)
    {   
        // try avoid.
        cv::circle(*grid_RGB, cv::Point((int)(sample.first),(int)(sample.second)),12, cv::Scalar(255,242,212), cv::FILLED, 0,0);
        // no center in.
        cv::circle(*grid_RGB, cv::Point((int)(sample.first),(int)(sample.second)),6, cv::Scalar(255,192,48), cv::FILLED, 0,0);
    }

    // 2. check if TKP is on sample lidar area.
    bool TKP_is_blocked = false;
    cv::Vec3b bgrPixel = grid_RGB->at<cv::Vec3b>(TKP->coordinate.first, TKP->coordinate.second);
    if(bgrPixel.val[0] == 255 && bgrPixel.val[1] == 192 && bgrPixel.val[2] == 48)
    {
        TKP_is_blocked = true;
    }

    // 3. return the bool.
    return TKP_is_blocked;
}

void compute_new_TKP(cv::Mat* grid_RGB, std::vector<Pair>* projected_keypoint, std::vector<Pair>* data_lidar, cv::Mat* grid_gray, sw::redis::Redis* redis, \
Path_keypoint* TKP)
{
    /*
        DESCRIPTION:
        this function will compute new TKP if the previous algorithm 
        detect problem on a current road.
    */

    // 1. found the projected KP most far of robot.
    double distance_max = 0;
    double index_distance_max = -1;
    Pair new_destination;
    for(int i = 0; i < projected_keypoint->size(); i++)
    {
        if((projected_keypoint->at(i).first >= 0 && projected_keypoint->at(i).first < 160) \
        && (projected_keypoint->at(i).second >= 0 && projected_keypoint->at(i).second < 80))
        {
            double distance = sqrt(pow(79-projected_keypoint->at(i).first,2)+pow(79-projected_keypoint->at(i).second,2));
            if(distance > distance_max) 
            {
                index_distance_max = i;
                new_destination.first = projected_keypoint->at(i).first;
                new_destination.second = projected_keypoint->at(i).second;
            }
        }
    }

    // 2. create new destination.
    bool found = false;
    bool alternatif = false;
    int research_j = 0;
    int research_i = 0;
    int size_cube = 0;
    while(!found)
    {
        int index_i = new_destination.first;
        int index_j = new_destination.second;
        if(size_cube == 0)
        {
            cv::Vec3b bgrPixel = grid_RGB->at<cv::Vec3b>(index_i, index_j);
            if(bgrPixel.val[0] != 255 && bgrPixel.val[1] != 192 && bgrPixel.val[2] != 48)
            {
                found = true;
                new_destination.first = index_i;
                new_destination.second = index_j;
            }
            else
            {
                size_cube++;
            }
        }
        else
        {
            for(int i = -size_cube; i <= size_cube; i++)
            {
                for(int j = -size_cube; j <= size_cube; j++)
                {
                    index_i += i; index_j+= j;
                    if(index_i < 0){index_i = 0;}
                    if(index_i > 79){index_i = 79;}
                    if(index_j < 0){index_j = 0;}
                    if(index_j > 79){index_j = 79;}
                    cv::Vec3b bgrPixel = grid_RGB->at<cv::Vec3b>(index_i, index_j);
                    if(bgrPixel.val[0] != 255 && bgrPixel.val[1] != 192 && bgrPixel.val[2] != 48)
                    {
                        found = true;
                        new_destination.first = index_i;
                        new_destination.second = index_j;
                        i = 2000; j = 2000; // leave for loops.
                    }
                }
            }
            size_cube++;
        }
    }

    // 3. prepare gray grid for local A* to destination.
    // make no go zone and try avoid area on grid and feed A*.
    for(auto sample : *data_lidar)
    {   
        // try avoid.
        cv::circle(*grid_gray, cv::Point((int)(sample.first),(int)(sample.second)),12, cv::Scalar(200), cv::FILLED, 0,0);
        // no center in.
        cv::circle(*grid_gray, cv::Point((int)(sample.first),(int)(sample.second)),6, cv::Scalar(0), cv::FILLED, 0,0);
    }

    // 4. compute A*.
    Pair current_position;
    current_position.first  = 79;
    current_position.second = 79;
    std::vector<Pair>* local_path = aStarSearch(*grid_gray, current_position, new_destination, redis, 1);

    // 4. select new TKP in this local A*.
    // INFO: we will chose a point at 30 cm of robot (index=6)
    if(local_path->size() > 6) 
    { 
        TKP->coordinate.first  = local_path->at(6).first;
        TKP->coordinate.second = local_path->at(6).second;
    }
    else
    {
        TKP->coordinate.first  = local_path->at(local_path->size()-1).first;
        TKP->coordinate.second = local_path->at(local_path->size()-1).second;
    }
    TKP->distance_RKP = sqrt(pow(79-TKP->coordinate.first,2)+pow(79-TKP->coordinate.second,2));
    
    // 5. compute angle in global ref.
    int index_i = TKP->coordinate.first - 80;
    int index_j = 80 - TKP->coordinate.second;
    double angle_TKP = 9999;
    if(index_i != 0) { angle_TKP = atan((double)(index_j)/(double)(index_i));} //in rad (-PI to PI)
    else
    {
        index_i = 0.01;
        angle_TKP = atan((double)(index_j)/(double)(index_i));
    }
    // put in motor commande referenciel [-90,0,90] we are currently in [0,-90,90,0]
    if(angle_TKP > 0) {angle_TKP = (M_PI_2 - angle_TKP);}
    else {angle_TKP = (-M_PI_2 - angle_TKP);}
    TKP->target_angle = angle_TKP;

    // 5. for visualisation, we can draw projected KP on RGB matrix.
    for(auto projected_KP : *projected_keypoint)
    {
        cv::circle(*grid_RGB, cv::Point((int)(projected_KP.first),(int)(projected_KP.second)),0, cv::Scalar(255,0,251), cv::FILLED, 0,0);
    }
    if(false)
    {
        cv::namedWindow("Local_env_debug",cv::WINDOW_AUTOSIZE);
        cv::resize(*grid_RGB, *grid_RGB, cv::Size(0,0),9.0,9.0,6);
        // cv::rotate(grid, grid, 1);
        cv::imshow("Local_env_debug", *grid_RGB);
        char d=(char)cv::waitKey(25);
    }
}

void compute_motor_autocommandeNico(sw::redis::Redis* redis, Path_keypoint* TKP, int option, std::vector<double>* position, Param_nav* navigation_param)
{
    /*
        DESCRIPTION: http://faculty.salina.k-state.edu/tim/robot_prog/MobileBot/Steering/pointFwd.html
    */
    double angle_ORIENTATION;
    double angle_RKP;

    if(option == 0)
    {
        angle_ORIENTATION = position->at(2);
        angle_RKP         = compute_vector_RKP(TKP->coordinate, *position);
    }

    /* TODO : Reflechir à une maniere to integrate other variable in this calcule, 
    like speed or area type. */
    // TODO: influencer par distance de validation
    const double V        = navigation_param->V;// desired velocity, u can make V depend on distance to target to slow down when close
    const double K        = navigation_param->K;// turning gain [0.5:1]
    double F              = navigation_param->F; // influence of straight line component
    const int back_angle  = navigation_param->back_angle; // angle to consider that we are moving backwards in rad
    const int stall_pwm   = navigation_param->stall_pwm;
    const int unstall_pwm = navigation_param->unstall_pwm;

    /* target_angle variable is good but is it between 0 and 180 degres.
    We don't know if we need to go left or right so we recompute a version on target angle
    between -180 and 180.*/
    double alpha;

    if(option == 0)
    {
        alpha         = (angle_RKP - angle_ORIENTATION) * M_PI / 180; // difference in rad between robot angle and targetvector angle
        alpha         += M_PI;
        alpha         = atan2(sin(alpha), cos(alpha));                //[-PI:PI]
    }
    if(option == 1)
    {
        alpha         = TKP->target_angle;
    }

    // we don't have sensors behind so don't move backwards
    // if (abs(alpha)>back_angle){
    //     F = 0;
    // }
    double rightspeed = (double)(V*(F*cos(alpha)+K*sin(alpha)));
    double leftspeed  = (double)(V*(F*cos(alpha)-K*sin(alpha)));

    //make sure the robot will move
    if(abs(leftspeed) < stall_pwm)
    {
        leftspeed     = unstall_pwm * leftspeed/abs(leftspeed);
    }
    if(abs(rightspeed) < stall_pwm)
    {
        rightspeed    = unstall_pwm * rightspeed/abs(rightspeed);
    }

    // std::cout <<  angle_RKP << " " << angle_ORIENTATION << " " << alpha << "[LS:" << leftspeed << ", RS:" << rightspeed << "\n";
    
    // send to robot  
    std::string msg_command = "1/";
    int direction;
    if(leftspeed>0) {direction = 1;}
    else {direction = -1;}
    for(int i = 0; i < 3; i++) { msg_command += std::to_string(direction) + "/" + std::to_string(leftspeed) + "/";}
    if(rightspeed>0) {direction = 1;}
    else {direction = -1;}
    for(int i = 0; i < 3; i++) { msg_command += std::to_string(direction) + "/" + std::to_string(rightspeed) + "/";}

    redis->publish("command_micro", msg_command);
}

void get_navigation_param(sw::redis::Redis* redis, Param_nav* navigation_param)
{
    navigation_param->F = std::stod(*(redis->get("Param_F")));
    navigation_param->K = std::stod(*(redis->get("Param_K")));
    navigation_param->V = std::stod(*(redis->get("Param_V")));
    navigation_param->back_angle = std::stoi(*(redis->get("Param_back_angle")));
    navigation_param->stall_pwm = std::stod(*(redis->get("Param_stall_pwm")));
    navigation_param->unstall_pwm = std::stod(*(redis->get("Param_unstall_pwm")));

    // if one time navigation param.
    navigation_param->get_param = true;
}

bool security_break(std::vector<Pair>* data_lidar)
{
    // if they are lidar point in this area, run security break.
    for(auto sample : *data_lidar)
    {
        if(sample.first >= 73 && sample.first <= 85 && \
        sample.second >= 76 && sample.second <= 79 &&)
        {
            // 20 cm in front of robot.
            return true;
        }
    }
    return false;
}
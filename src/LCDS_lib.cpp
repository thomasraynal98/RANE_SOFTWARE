#include <LCDS_lib.h>

void setup_new_lidar_sample(std::vector<Lidar_data>* new_lidar_sample)
{
    //TODO:    DESCRIPTION: Cette function va remplir le vecteur new_lidar_sample avec 360 cases de
    //TODO:    données lors du lancement.

    Lidar_data reset_model(-1,-1);
    for(int idx = 0; idx < 360; idx++)
    {
        new_lidar_sample->push_back(reset_model);
    }
}

void get_new_lidar_sample(std::vector<Lidar_data>* new_lidar_sample, std::string raw_lidar_sample)
{
    //TODO:    DESCRIPTION: Cette fonction va recuperer le nouveau jeu de données provenant du lidar.
    //TODO:    1) il va reinitialiser les datas du vecteur en input.
    //TODO:    2) il va remplir avec les nouvelles données.
    
    // 1. Reset vector.
    Lidar_data reset_model(-1,-1);
    for(int idx = 0; idx < new_lidar_sample->capacity(); idx++)
    {
        new_lidar_sample->at(idx) = reset_model;
    }

    // 2. Read redis subscriber and fill vector.
    std::string T;
    std::stringstream X(raw_lidar_sample);
    int i = 0;
    while(std::getline(X, T, ','))
    {
        if(i % 2 == 0) new_lidar_sample->at((int)i/2).value = std::stod(T);
        if(i % 2 == 1) new_lidar_sample->at((int)i/2).angle = std::stod(T);
        i += 1;
    }
}

void setup_new_GPKP(std::vector<Pixel_position>* GPKP)
{
    //TODO:    DESCRIPTION: Cette function va remplir le vecteur new_lidar_sample avec 360 cases de
    //TODO:    données lors du lancement.

    Pixel_position reset_model(-1,-1);
    for(int idx = 0; idx < GPKP->capacity(); idx++)
    {
        GPKP->push_back(reset_model);
    }
}

void get_new_GPKP(std::vector<Pixel_position>* GPKP, std::string redis_input_str)
{
    //TODO:    DESCRIPTION: Cette fonction va recuperer le nouveau Global Path KeyPoint.
    //TODO:    1) il va reinitialiser les datas du vecteur en input.
    //TODO:    2) il va remplir avec les nouvelles données.

    // 1. Reset vector.
    Pixel_position reset_model(-1,-1);
    for(int idx = 0; idx < GPKP->capacity(); idx++)
    {
        GPKP->at(idx) = reset_model;
    }

    // 2. Read redis variable and fill vector.
    std::string T;
    std::stringstream X(redis_input_str);
    int i = 0;
    while(std::getline(X, T, '/'))
    {
        if(i % 2 == 0) GPKP->at((int)i/2).idx_col = std::stoi(T);
        if(i % 2 == 1) GPKP->at((int)i/2).idx_row = std::stoi(T);
        i += 1;
    }
}

void get_navigation_param(sw::redis::Redis* redis, Param_nav* navigation_param)
{
    //TODO: update les parametres de navigations pour le controle moteur.

    navigation_param->F = std::stod(*(redis->get("Param_F")));
    navigation_param->K = std::stod(*(redis->get("Param_K")));
    navigation_param->V = std::stod(*(redis->get("Param_V")));
    navigation_param->back_angle = std::stoi(*(redis->get("Param_back_angle")));
    navigation_param->stall_pwm = std::stod(*(redis->get("Param_stall_pwm")));
    navigation_param->unstall_pwm = std::stod(*(redis->get("Param_unstall_pwm")));
}

void setup_lidarWindows(std::vector<Lidar_sample>* lidarWindows)
{
    //TODO: setup toute la lidarWindows.

    // 1. Reset vector.
    Robot_position_transformation reset_model;

    for(int idx = 0; idx < lidarWindows->capacity(); idx++)
    {
        lidarWindows->at(idx).viewpoint = reset_model;
        lidarWindows->at(idx).observation.reserve(360);
        setup_new_lidar_sample(&lidarWindows->at(idx).observation);
    }
}

void add_lidar_sample_to_lidarWindows(int lidar_count, std::vector<Lidar_sample>* lidarWindows, std::vector<Lidar_data>* new_lidar_sample, Robot_complete_position* position_robot)
{
    //TODO: add this new observation lidar sample to the lidarWindows.
    //TODO: 1. Reset the current line.
    //TODO: 2. Add the new_lidar_sample value to this line.

    // 1. Reset the line.
    Robot_position_transformation reset_model_transfo;
    lidarWindows->at(lidar_count).viewpoint = reset_model_transfo;
    Lidar_data reset_model(-1,-1);
    for(int idx = 0; idx < lidarWindows->at(lidar_count).observation.capacity(); idx++)
    {
        lidarWindows->at(lidar_count).observation[idx] = reset_model;
    }

    // 2. add the new lidar sample in this line.
    //! Ici la position du lidar correspond à la position de la caméra.
    lidarWindows->at(lidar_count).viewpoint.x       = position_robot->x_cam;
    lidarWindows->at(lidar_count).viewpoint.y       = position_robot->y_cam;
    lidarWindows->at(lidar_count).viewpoint.yaw_deg = position_robot->yaw_deg_cam;

    for(int idx = 0; idx < lidarWindows->at(lidar_count).observation.capacity(); idx++)
    {
        if(new_lidar_sample->at(idx).value == -1) break;
        lidarWindows->at(lidar_count).observation[idx] = new_lidar_sample->at(idx);
    }
}

bool is_new_position_detected(Robot_complete_position* position_robot, sw::redis::Redis* redis)
{
    //TODO: cette fonction va detecter si le vslam à mis à jour la position du robot.
    //TODO: 1. Recuperer la position de redis.
    //TODO: 2. La comparer.

    Robot_complete_position redis_output_position;

    // 1. Recuperation des données.
    std::string redis_output_position_string = *(redis->get("State_robot_position_png"));

    std::string T;
    std::stringstream X(redis_output_position_string);
    int i = 0;
    while(std::getline(X, T, '/'))
    {
        if(i == 4) 
        {
            redis_output_position.yaw_deg_cam    = std::stod(T);
            redis_output_position.yaw_deg_center = std::stod(T);
        }
        i += 1;
    }
    
    redis_output_position_string = *(redis->get("State_robot_position"));
    std::stringstream X2(redis_output_position_string);

    //TODO: VERIFICATION QUE LE COUPLE X,Y EST LE BON.
    i = 0;
    while(std::getline(X2, T, '/'))
    {
        if(i == 0) 
        {
            redis_output_position.x_cam = std::stod(T);
        }
        if(i == 1) 
        {
            redis_output_position.y_cam = std::stod(T);
        }
        i += 1;
    }

    redis_output_position_string = *(redis->get("State_robot_position_center"));
    std::stringstream X3(redis_output_position_string);

    //TODO: VERIFICATION QUE LE COUPLE X,Y EST LE BON.
    i = 0;
    while(std::getline(X3, T, '/'))
    {
        if(i == 0) 
        {
            redis_output_position.x_center = std::stod(T);
        }
        if(i == 1) 
        {
            redis_output_position.y_center = std::stod(T);
        }
        i += 1;
    }

    redis_output_position_string = *(redis->get("State_robot_position_png"));
    std::stringstream X4(redis_output_position_string);

    //TODO: VERIFICATION QUE LE COUPLE I,J EST DANS LE BON SENSE.
    i = 0;
    while(std::getline(X4, T, '/'))
    {
        if(i == 0) 
        {
            redis_output_position.p_cam.idx_col = std::stoi(T);
        }
        if(i == 1) 
        {
            redis_output_position.p_cam.idx_row = std::stoi(T);
        }
        if(i == 2) 
        {
            redis_output_position.p_center.idx_row = std::stoi(T);
        }
        if(i == 3) 
        {
            redis_output_position.p_center.idx_row = std::stoi(T);
        }
        i += 1;
    }

    // 2. Comparer with le current.
    if(!(*position_robot == redis_output_position))
    {
        *position_robot = redis_output_position;
        return true;
    }
    return false;
}

void filter_GPKP(Robot_complete_position* position_robot, std::vector<Pixel_position>* GPKP, std::vector<bool>* GPKP_notYetReached_b)
{
    //TODO: Cette fonction va chercher le KP le plus proche du centre du robot et va ensuite selectionner l'ensemble des KP qui
    //TODO: le separe de la destination en ajoutant true sur le vecteur GPKP_notYetReached_b;

    // 1. Setup GPKP_notYetReached_b.
    for(int idx = 0; idx < GPKP_notYetReached_b->capacity(); idx++)
    {
        GPKP_notYetReached_b->at(idx) = false;
    }

    // 2. Found the index of nearest KP.
    double distance_min = 9999; int idx_min = -1;
    double new_distance;
    for(int idx = 0; idx < GPKP->capacity(); idx++)
    {
        new_distance = distance_btw_pixel(position_robot->p_center, GPKP->at(idx), 0.05);
        if(new_distance < distance_min)
        {
            distance_min = new_distance;
            idx_min = idx;
        }
    }

    // 3. Set GPKP_notYetReached_b.
    int start = idx_min + 1;
    if(start >= GPKP_notYetReached_b->capacity()) start = idx_min;
    for(int idx = start + 1; idx < GPKP_notYetReached_b->capacity(); idx++)
    {
        GPKP_notYetReached_b->at(idx) = true;
    }
}

double distance_btw_pixel(Pixel_position p1, Pixel_position p2, double resolution)
{
    //TODO: return the distance in meter between two pixel.
    return sqrt(pow(p1.idx_col-p2.idx_col,2)+pow(p1.idx_row-p2.idx_row,2)) * resolution;
}

void project_GPKP_onLCDS(cv::Mat* LCDS_color, std::vector<Pixel_position>* GPKP, std::vector<bool>* GPKP_notYetReached_b, std::vector<Pixel_position>* GPKP_onLCDS, Robot_complete_position* position_robot)
{
    //TODO: Cette function va projeter les KP validant les conditions et va remplir le vecteur GPKP_onLCDS.

    // 1. Clean vector GPKP_onLCDS.
    Pixel_position reset_model(-1,-1);
    for(int idx = 0; idx < GPKP_onLCDS->capacity(); idx++) GPKP_onLCDS->at(idx) = reset_model;

    // 2. Project each points and only keep the ones in LCDS windows.
    int counter_PKP = 0; double distance_RKP = -1; double angle_RKP_onLCDS;
    for(int idx = 0; idx < GPKP->capacity(); idx++)
    {
        if(GPKP_notYetReached_b->at(idx))
        {
            // 3. Project.
            distance_RKP     = compute_distance_RKP(position_robot, &GPKP->at(idx));
            angle_RKP_onLCDS = compute_angle_RKP_onLCDS(position_robot, &GPKP->at(idx));
        }
    }
}

double compute_distance_RKP(Robot_complete_position* position_robot, Pixel_position* KP)
{
    return distance_btw_pixel(position_robot->p_center, *KP, 0.05);
}

double compute_angle_RKP_onLCDS(Robot_complete_position* position_robot, Pixel_position* KP)
{
    //TODO: Cette function va compute l'angle entre le robot et un KP dans le referenciel du LCDS.
    //TODO: RAPPEL REF LCDS: <Gauche à droite en partant de derrière> [-pi,0,0,pi] <Fixe par rapport à l'orientation du robot>
    //TODO: RAPPEL REF GMAP: <Fixe comme le nord géographique> [0,2pi]

    double x_sum = KP->idx_col - position_robot->p_center.idx_col;
    double y_sum = KP->idx_row - position_robot->p_center.idx_row;
    double angle_rad = acos((x_sum)/(sqrt(pow(x_sum, 2.0) + pow(y_sum, 2.0))));
    if(y_sum < 0)
    {
        angle_rad = (M_PI - angle_rad) + M_PI;
    }


    double angle_RKP_GMAP   = rad_to_deg(angle_rad);
    double angle_robot_GMAP = position_robot->yaw_deg_center;
}

double deg_to_rad(double deg)
{
    return deg * M_PI / 180;
}

double rad_to_deg(double rad)
{
    return rad * 180 / M_PI;
}
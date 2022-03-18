#include <LCDS_lib.h>
#include <global_path_lib.h>

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

void setup_new_GPKP_notYetReached_b(std::vector<bool>* GPKP_notYetReached_b)
{
    bool setup_model = false;
    for(int idx = 0; idx < GPKP_notYetReached_b->capacity(); idx++)
    {
        GPKP_notYetReached_b->push_back(setup_model);
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
        if(i % 2 == 0) new_lidar_sample->at((int)i/2).angle = std::stod(T);
        if(i % 2 == 1) new_lidar_sample->at((int)i/2).value = std::stod(T);
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
    Lidar_sample reset_sample_model;

    for(int idx = 0; idx < lidarWindows->capacity(); idx++)
    {
        lidarWindows->push_back(reset_sample_model);
        lidarWindows->at(idx).viewpoint = reset_model;
        lidarWindows->at(idx).observation.reserve(360);
        setup_new_lidar_sample(&lidarWindows->at(idx).observation);
    }
}

void reset_lidarWindows(std::vector<Lidar_sample>* lidarWindows)
{
    //TODO: reset value.
    Lidar_data reset_model(-1,-1);
    Robot_position_transformation reset_model_view;
    for(int idx_sample; idx_sample < lidarWindows->capacity(); idx_sample++)
    {
        if(lidarWindows->at(idx_sample).observation[0].value != -1)
        {
            for(auto lidar_data: lidarWindows->at(idx_sample).observation)
            {
                if(lidarWindows->at(idx_sample).observation[0].value == -1) break;
                lidar_data = reset_model;
            }

            lidarWindows->at(idx_sample).viewpoint = reset_model_view;
        }
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
        if(lidarWindows->at(lidar_count).observation[idx].value == -1) break;
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

    // 3. Filtrage param.
    // ! filter_windows egale le nombre de case devant et derrière.
    int filter_windows = 10;
    double filter_x = 0; double filter_y = 0; double filter_yaw_deg = 0;
    int idx_filtered = lidar_count - filter_windows;

    if(idx_filtered < 0) idx_filtered = lidarWindows->capacity() + idx_filtered;

    // Check if all the windows is valide.
    bool windows_valide = true;

    for(int i = -filter_windows; i <= filter_windows; i++)
    {
        int idx_search = idx_filtered + i;

        if(idx_search < 0) idx_search = lidarWindows->capacity() + idx_search;
        if(idx_search >= lidarWindows->capacity()) idx_search = idx_search - lidarWindows->capacity();
        if(lidarWindows->at(idx_search).viewpoint.x == 0) windows_valide = false;
    }

    // Filter.
    // double deg_dif = 0;
    if(windows_valide)
    {
        int l_c = 0;
        for(int i = -filter_windows; i <= filter_windows; i++)
        {
            int idx_search = idx_filtered + i;
            if(idx_search < 0) idx_search = lidarWindows->capacity() + idx_search;
            if(idx_search >= lidarWindows->capacity()) idx_search = idx_search - lidarWindows->capacity();
            filter_x += lidarWindows->at(idx_search).viewpoint.x;
            filter_y += lidarWindows->at(idx_search).viewpoint.y;

            //TODO: Filter rotation angle is not available yet.
            // if(l_c == 0) filter_yaw_deg = lidarWindows->at(idx_search).viewpoint.yaw_deg;
            // if(l_c != 0)
            // {
            //     if(lidarWindows->at(idx_search).viewpoint.yaw_deg > filter_yaw_deg)
            //     {
            //         if(lidarWindows->at(idx_search).viewpoint.yaw_deg - filter_yaw_deg > 180)
            //         {
            //             deg_dif = filter_yaw_deg + (360 - lidarWindows->at(idx_search).viewpoint.yaw_deg);
            //             deg_dif = deg_dif / 2;
            //             filter_yaw_deg -= deg_dif;
            //             if(filter_yaw_deg < 0) filter_yaw_deg += 360;
            //         }
            //         else
            //         {
            //             filter_yaw_deg = (filter_yaw_deg + lidarWindows->at(idx_search).viewpoint.yaw_deg)/2;
            //         }
            //     }
            //     else
            //     {
            //         if(filter_yaw_deg - lidarWindows->at(idx_search).viewpoint.yaw_deg > 180)
            //         {
            //             deg_dif = lidarWindows->at(idx_search).viewpoint.yaw_deg + (360 - filter_yaw_deg);
            //             deg_dif = deg_dif / 2;
            //             filter_yaw_deg += deg_dif;
            //             if(filter_yaw_deg > 360) filter_yaw_deg -= 360;
            //         }
            //         else
            //         {
            //             filter_yaw_deg = (filter_yaw_deg + lidarWindows->at(idx_search).viewpoint.yaw_deg)/2;
            //         }
            //     }
            // }

            // filter_yaw_deg += lidarWindows->at(idx_search).viewpoint.yaw_deg;
            l_c ++;
        }
        filter_x = filter_x / (l_c);
        filter_y = filter_y / (l_c);
        filter_yaw_deg = lidarWindows->at(idx_filtered).viewpoint.yaw_deg;

        // Add filtered variable.
        lidarWindows->at(idx_filtered).viewpoint.x = filter_x;
        lidarWindows->at(idx_filtered).viewpoint.y = filter_y;
        lidarWindows->at(idx_filtered).viewpoint.yaw_deg = filter_yaw_deg;
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
            redis_output_position.p_center.idx_col = std::stoi(T);
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
    double new_distance = -1;
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
        if(GPKP->at(idx).idx_col == -1) break;
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

    // 0. Variable initialisation.
    double col_PKP = -1; double row_PKP = -1;
    double windows_col = LCDS_color->cols;
    double windows_row = LCDS_color->rows;

    // 1. Clean vector GPKP_onLCDS.
    Pixel_position reset_model(-1,-1);
    for(int idx = 0; idx < GPKP_onLCDS->capacity(); idx++) GPKP_onLCDS->at(idx) = reset_model;

    // 2. Project each points and only keep the ones in LCDS windows.
    int counter_PKP = 0; double distance_RKP = -1; double angle_RKP_onLCDS;
    for(int idx = 0; idx < GPKP->capacity(); idx++)
    {
        if(GPKP_notYetReached_b->at(idx))
        {
            // 3. Compute data information.
            distance_RKP     = compute_distance_RKP(position_robot, &GPKP->at(idx));
            angle_RKP_onLCDS = compute_angle_RKP_onLCDS(position_robot, &GPKP->at(idx));

            // 4. Project KP in LCDS.
            //! We can project the KP even if they are behind the robot.
            //TODO: check if windows_col and windows_row are in the good direction.
            //TODO: FORMULE = sin(180-angle_RKP) * distance_RKP * px_Width_Windows/2 / m_demi_Width_Windows + px_Height_Windows ?
            col_PKP = row_PKP = -1;
            col_PKP = cos(deg_to_rad(angle_RKP_onLCDS)-M_PI_2)*distance_RKP/0.05+(windows_col/2);
            row_PKP = sin(deg_to_rad(angle_RKP_onLCDS)-M_PI_2)*distance_RKP/0.05+(windows_row/2);

            // 5. Check if this PKP can fit in windows.
            if(col_PKP >= 0 && col_PKP < windows_col && row_PKP >= 0 && row_PKP < windows_row)
            {   
                GPKP_onLCDS->at(counter_PKP).idx_col = col_PKP;
                GPKP_onLCDS->at(counter_PKP).idx_row = row_PKP;
                counter_PKP += 1;
            }
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

    // double debg = position_robot->yaw_deg_center + 90;
    // if(debg > 360) debg -= 360;
    double angle_robot_GMAP = position_robot->yaw_deg_center;
    // double angle_robot_GMAP = debg;
    double angle_RKP_GMAP   = rad_to_deg(angle_rad);



    // std::cout << "ROBOT:" << debg << " RKP:" << angle_RKP_GMAP << std::endl;


    if(angle_robot_GMAP > angle_RKP_GMAP)
    {
        if(angle_robot_GMAP - angle_RKP_GMAP > 180)
        {
            return (360-angle_robot_GMAP) + angle_RKP_GMAP;
        }
        else
        {
            return -(angle_robot_GMAP - angle_RKP_GMAP);
        }
    }
    else
    {
        if(angle_RKP_GMAP - angle_robot_GMAP > 180)
        {
            return -(angle_robot_GMAP + (360-angle_RKP_GMAP));
        }
        else
        {
            return angle_RKP_GMAP - angle_robot_GMAP;
        }
    }
}

double compute_angle_btw_Robot_and_observation_pts(double pt_x, double pt_y, Robot_complete_position* position_robot)
{
    //TODO: Cette fonction va compute l'angle entre l'orientation du robot (vers l'avant) et le dernier points 
    //TODO: d'observation du lidar. 
    //TODO: REF GMAP_continue : les deux positions sont données en X et Y continue.
    //TODO: REF LCDS: l'angle est données [-180,0,0,180]. return in degres.

    // double x_sum = pt_x - position_robot->x_cam;
    // double y_sum = pt_y - position_robot->y_cam;
    // double angle_rad = acos((x_sum)/(sqrt(pow(x_sum, 2.0) + pow(y_sum, 2.0))));
    // if(y_sum < 0)
    // {
    //     angle_rad = (M_PI - angle_rad) + M_PI;
    // }
    double y = pt_x - position_robot->x_cam;
    double x = pt_y - position_robot->y_cam;

    // std::cout << "DEST:" << pt_x << "|" << pt_y << " RB:" << position_robot->x_cam << "|" << position_robot->y_cam << std::endl;
    // std::cout << "MOUVEMENT MAP X=" << x << " Y=" << y << std::endl;

    double angle_rad = acos((x)/(sqrt(pow(x,2)+pow(y,2))));
    if(y < 0) angle_rad = 2*M_PI - angle_rad;

    double angle_RKP_GMAP   = rad_to_deg(angle_rad);    // Depend de la direction
    double angle_robot_GMAP = position_robot->yaw_deg_cam; 

    // std::cout << "ROBOT: " << angle_robot_GMAP << " " << angle_RKP_GMAP << " " << angle_robot_GMAP-angle_RKP_GMAP << std::endl;

    if(angle_robot_GMAP > angle_RKP_GMAP)
    {
        if(angle_robot_GMAP - angle_RKP_GMAP > 180)
        {
            return (360-angle_robot_GMAP) + angle_RKP_GMAP;
        }
        else
        {
            return -(angle_robot_GMAP - angle_RKP_GMAP);
        }
    }
    else
    {
        if(angle_RKP_GMAP - angle_robot_GMAP > 180)
        {
            return -(angle_robot_GMAP + (360-angle_RKP_GMAP));
        }
        else
        {
            return angle_RKP_GMAP - angle_robot_GMAP;
        }
    }
}

double compute_angle_btw_px(Pixel_position* p1, Pixel_position* p2)
{
    double x_sum = p1->idx_col - p2->idx_col;
    double y_sum = p1->idx_row - p2->idx_row;
    double angle_rad = acos((x_sum)/(sqrt(pow(x_sum, 2.0) + pow(y_sum, 2.0))));
    
    angle_rad += M_PI_2;
    if(angle_rad > M_PI) angle_rad -= 2*M_PI;

    //! angle_rad est dans le referenciel de opencv image.
    //! mettre directement dans le ref lidar.
    return angle_rad;
}

double deg_to_rad(double deg)
{
    return deg * M_PI / 180;
}

double rad_to_deg(double rad)
{
    return rad * 180 / M_PI;
}

void project_LW_onLCDS(Robot_complete_position* position_robot, std::vector<Lidar_sample>* lidarWindows, std::vector<Pixel_position>* LW_onLCDS, cv::Mat* LCDS_color, int lidar_count)
{
    //TODO: Cette fonction qui est la grande nouveauté de LCDS 3.0 va projecter dans la LCDS Windows
    //TODO: L'intégralité des x derniers samples obtenu par le lidar en prenant en compte le 
    //TODO: déplacement du lidar entre ces x derniers samples. Cette fonction ce rapproche un peux
    //TODO: d'une fonction de mapping sur un lapse de temps très court. Ce qui permet d'avoir un
    //TODO: drift minimun et de ne pas a implementer d'autre fonction de SLAM.
    //! Ici LCDS_color est en faite LCDS_compute !

    // 0. Setup variable.
    double windows_col = LCDS_color->cols;
    double windows_row = LCDS_color->rows;
    double m_LCDS_demi_width = LCDS_color->cols * 0.05 / 2;

    // 1. Reset les valeurs de LW_onLCDS.
    Pixel_position reset_model(-1,-1);
    for(int idx = 0; idx < LW_onLCDS->capacity(); idx++)
    {
        if(LW_onLCDS->at(idx).idx_col == -1) break;
        LW_onLCDS->at(idx) = reset_model;
    }

    // 2. Update les translations des points viewpoints de chaque samples de lidar.
    for(int idx = 0; idx < lidarWindows->capacity(); idx++)
    {
        // 3. Update only les lignes contenant de la data.
        if(lidarWindows->at(idx).viewpoint.x != 0 && \
           lidarWindows->at(idx).viewpoint.y != 0 && \
           lidarWindows->at(idx).viewpoint.yaw_deg != 0)
        {   
            // 4. Update translation.
            lidarWindows->at(idx).viewpoint.delta_x = position_robot->x_cam - lidarWindows->at(idx).viewpoint.x;
            lidarWindows->at(idx).viewpoint.delta_y = position_robot->y_cam - lidarWindows->at(idx).viewpoint.y;

            // 5. Update rotation.
            lidarWindows->at(idx).viewpoint.detal_yaw_deg = compute_angle_btw_angle(position_robot->yaw_deg_cam, lidarWindows->at(idx).viewpoint.yaw_deg);
        }
    }

    // 6. Project chaque sample dans la LCDS.
    int counter_lidar_data = 0; int col_lidar_data = -1; double row_lidar_data = -1; double angle_data = -1;
    for(int idx_sample = 0; idx_sample < lidarWindows->capacity(); idx_sample++)
    {
        if(lidarWindows->at(idx_sample).viewpoint.x != 0 && \
           lidarWindows->at(idx_sample).viewpoint.y != 0 && \
           lidarWindows->at(idx_sample).viewpoint.yaw_deg != 0)
        { 
            double debug_angle_rad = deg_to_rad(compute_angle_btw_Robot_and_observation_pts(lidarWindows->at(idx_sample).viewpoint.x, lidarWindows->at(idx_sample).viewpoint.y, position_robot) - 90);
            double distance_Frame_to_LidarObservationRef = sqrt(pow(lidarWindows->at(idx_sample).viewpoint.delta_x,2)+pow(lidarWindows->at(idx_sample).viewpoint.delta_y,2));
            double col_origin = -1; double row_origin = -1;
            
            double angle_btw_obs = compute_angle_btw_angle(position_robot->yaw_deg_cam, lidarWindows->at(idx_sample).viewpoint.yaw_deg);

            for(int idx_data = 0; idx_data < lidarWindows->at(idx_sample).observation.capacity(); idx_data++)
            {
                row_lidar_data = col_lidar_data = -1;
                if(lidarWindows->at(idx_sample).observation[idx_data].value == -1) break;

                //! Verification de FORMULE obligatoire. Notament col<>row, et x<>y.
                //! Le +6.0 correspond au faite que lidar est situé 30 cm en avant du centre du robot.
               
                col_origin = cos(debug_angle_rad - M_PI_2)*distance_Frame_to_LidarObservationRef/0.05+(windows_col/2);
                row_origin = sin(debug_angle_rad - M_PI_2)*distance_Frame_to_LidarObservationRef/0.05+(windows_row/2)-6.0;

                angle_data = lidarWindows->at(idx_sample).observation[idx_data].angle;

                col_lidar_data = cos(angle_data+deg_to_rad(angle_btw_obs)+M_PI_2)*lidarWindows->at(idx_sample).observation[idx_data].value/0.05+col_origin;
                row_lidar_data = sin(angle_data+deg_to_rad(angle_btw_obs)+M_PI_2)*lidarWindows->at(idx_sample).observation[idx_data].value/0.05+row_origin;

                // 7. Check si la lidar data projeté est dans la LCDS windows.
                if(col_lidar_data >= 0 && col_lidar_data < windows_col && row_lidar_data >= 0 && row_lidar_data < windows_row)
                {

                    LW_onLCDS->at(counter_lidar_data).idx_col = col_lidar_data;
                    LW_onLCDS->at(counter_lidar_data).idx_row = row_lidar_data;
                    counter_lidar_data += 1;
                }
            }
        }
    }

    // 7. Draw data on LCDS_compute.
    for(auto lidar_data : *LW_onLCDS)
    {
        cv::circle(*LCDS_color, cv::Point((int)(lidar_data.idx_col),(int)(lidar_data.idx_row)),10, cv::Scalar(200), cv::FILLED, 0,0);
        cv::circle(*LCDS_color, cv::Point((int)(lidar_data.idx_col),(int)(lidar_data.idx_row)),6, cv::Scalar(0),   cv::FILLED, 0,0);
    }
}

double compute_angle_btw_angle(double angle_principal, double angle_secondaire)
{
    //TODO: Cette fonction va retourner l'angle formé entre deux directions.
    //TODO: Les deux angles sont données dans le REF GMAP: [0,2pi]
    //TODO: L'angle renvoyé est données dans le REF LCDS:  [-pi,0,0,pi]
    //! ICI ce n'ai pas [-pi,0,0,pi] mais [-180,0,0,180]
    //TODO: PS: La valeur renvoyé depend de angle_principal, Si angle_secondaire ce situe à gauche
    //TODO: l'angle renvoyé sera négatif, si il est à droite, l'angle renvoyé sera positif.

    if(angle_principal > angle_secondaire)
    {
        if(angle_principal - angle_secondaire > 180)
        {
            return (360-angle_principal) + angle_secondaire;
        }
        else
        {
            return -(angle_principal - angle_secondaire);
        }
    }
    else
    {
        if(angle_secondaire - angle_principal > 180)
        {
            return -(angle_principal + (360-angle_secondaire));
        }
        else
        {
            return angle_secondaire - angle_principal;
        }
    }
}

bool new_relocalisation(sw::redis::Redis* redis)
{
    //TODO: Cette fonction va avertir le thread si le slam c'est repositionner.

    std::string redis_output = *(redis->get("State_slamcore"));
    if(redis_output.compare("RELOCALISED") == 0) return true;
    if(redis_output.compare("LOOP_CLOSURE") == 0) return true;
    return false;
}   

double distance_btw_pts(double pts_A_x, double pt_A_y, double pt_B_x, double pt_B_y)
{
    return sqrt(pow(pts_A_x-pt_B_x,2)+pow(pt_A_y-pt_B_y,2));
}

//TODO: FUNCTION PART B.
void project_ILKP_onLCDS(cv::Mat* LCDS_color, Robot_complete_position* position_robot, Intermediate_LCDS_KP* ILKP)
{
    //TODO: Cette function va projeter le ILKP dans le LCDS.

    // 0. Setup variable.
    double windows_col = LCDS_color->cols;
    double windows_row = LCDS_color->rows;
    double m_LCDS_demi_width = LCDS_color->cols * 0.05 / 2;

    // 1. Compute origine of observation in new position.
    double idx_col_origine = -1; double idx_row_origine = -1; double angle_btw_obs = -1;

    double distance_observation_to_current = distance_btw_pts(ILKP->viewpoint.x, ILKP->viewpoint.y, position_robot->x_center, position_robot->y_center);
    double angle_btw_current_and_observation = deg_to_rad(compute_angle_btw_Robot_and_observation_pts(ILKP->viewpoint.x, ILKP->viewpoint.y, position_robot) - 90);

    idx_col_origine = cos(angle_btw_current_and_observation - M_PI_2)*distance_observation_to_current/0.05+(windows_col/2);
    idx_row_origine = sin(angle_btw_current_and_observation - M_PI_2)*distance_observation_to_current/0.05+(windows_row/2);

    // 2. Project ILKP on LCDS.
    angle_btw_obs = compute_angle_btw_angle(position_robot->yaw_deg_cam, ILKP->viewpoint.yaw_deg);
    ILKP->px_onLCDS.idx_col = cos(ILKP->angle+deg_to_rad(angle_btw_obs)+M_PI_2)*ILKP->distance/0.05+idx_col_origine;
    ILKP->px_onLCDS.idx_row = sin(ILKP->angle+deg_to_rad(angle_btw_obs)+M_PI_2)*ILKP->distance/0.05+idx_row_origine;
}

void select_local_destination(Pixel_position* Local_destination, Intermediate_LCDS_KP* ILKP, Robot_complete_position* position_robot, cv::Mat* LCDS_compute, std::vector<Pixel_position>* GPKP_onLCDS)
{
    //TODO: Cette function va selectionner la local destination de cette époque du LCDS.

    // 1. Check if they are a ILKP init.
    if(ILKP->distance != -1)
    {
        // Check projection.
        project_ILKP_onLCDS(LCDS_compute, position_robot, ILKP);

        // Check projection checking.
        if(!ILKP_is_onLCDS(LCDS_compute, ILKP))
        {
            ILKP_reset(ILKP);
        }

        // Check reach.
        if(ILKP->distance != -1 && ILKP_is_reach(LCDS_compute, ILKP, 1.0))
        {
            ILKP_reset(ILKP);
        }

        // Check if it's available.
        if(ILKP->distance != -1 && !ILKP_is_available(LCDS_compute, ILKP))
        {
            ILKP_reset(ILKP);
        }
    }

    // 2. Choice between ILKP or FPKP.
    //! FPKP = Farest Projected Key Point.

    if(ILKP->distance != -1)
    {
        //! ILKP.
        *Local_destination = ILKP->px_onLCDS;
    }
    if(ILKP->distance == -1)
    {
        //! FPKP.
        select_FPKP(Local_destination, LCDS_compute, GPKP_onLCDS, 0);
    }
}

bool ILKP_is_onLCDS(cv::Mat* LCDS_compute, Intermediate_LCDS_KP* ILKP)
{
    // 0. Setup variable.
    double windows_col = LCDS_compute->cols;
    double windows_row = LCDS_compute->rows;

    if(ILKP->px_onLCDS.idx_col >= 0 && ILKP->px_onLCDS.idx_col < windows_col && \
       ILKP->px_onLCDS.idx_row >= 0 && ILKP->px_onLCDS.idx_row < windows_row)
    {
        return true;
    }
    return false;
}

void ILKP_reset(Intermediate_LCDS_KP* ILKP)
{
    Intermediate_LCDS_KP reset_model(-1,-1);
    *ILKP = reset_model;
}

bool ILKP_is_reach(cv::Mat* LCDS_compute, Intermediate_LCDS_KP* ILKP, double m_validation_distance)
{
    Pixel_position origine_robot_centre((int)LCDS_compute->cols/2, (int)LCDS_compute->rows/2);
    double m_distance_btw_px = distance_btw_pixel(ILKP->px_onLCDS, origine_robot_centre, 0.05);

    if(m_distance_btw_px < m_validation_distance) return true;
    return false;
}

bool ILKP_is_available(cv::Mat* LCDS_compute, Intermediate_LCDS_KP* ILKP)
{
    if((int)LCDS_compute->at<uchar>((int)(ILKP->px_onLCDS.idx_col), (int)(ILKP->px_onLCDS.idx_row)) == 0)
    {
        return false;
    }
    return true;
}

Pixel_position select_FPKP(Pixel_position* Local_destination, cv::Mat* LCDS_compute, std::vector<Pixel_position>* GPKP_onLCDS, int unknow_option)
{
    //TODO: Cette fonction va retourner le Farest Projected Key Point "available" on LCDS.
    //! unknow option va colorer la partie inconnu et chercher des points horizontal libre.
    //! 0 = Pas de mode Unknow.

    Pixel_position origine_robot_centre((int)LCDS_compute->cols/2, (int)LCDS_compute->rows/2);
    double distance_max = -1;

    Pixel_position FPKP(-1,-1);
    Pixel_position reset_model(-1,-1);

    //! distance_up   : represente la distance max du FPKP.
    //! distance_down : represente la distance min du FPKP en dessous duquel pas de destination est detecter.
    double distance_up    = 20.0;
    double distance_down  = 2.0;

    if(unknow_option == 0)
    {
        double distance_max = -1;
        while(FPKP.idx_col == -1 && distance_max == -1)
        {
            // found new PKP.
            distance_max = -1;
            for(auto PKP : *GPKP_onLCDS)
            {
                double m_distance_btw = distance_btw_pixel(origine_robot_centre, PKP, 0.05);
                if(m_distance_btw > distance_max && m_distance_btw < distance_up && m_distance_btw > distance_down) 
                {
                    FPKP = PKP;
                    distance_max = m_distance_btw;
                }
            }

            if(distance_max != -1) distance_up = distance_max;

            // check if it's available.
            if(!PKP_is_available(LCDS_compute, &FPKP))
            {
                FPKP = reset_model;
            }
        }
    }

    *Local_destination = reset_model;
}

bool PKP_is_available(cv::Mat* LCDS_compute, Pixel_position* px)
{
    if((int)LCDS_compute->at<uchar>((int)(px->idx_col), (int)(px->idx_row)) == 0)
    {
        return false;
    }
    return true;
}

void create_trajectory(sw::redis::Redis* redis, Pixel_position* Local_destination, cv::Mat* LCDS_compute, std::vector<Pixel_position>* Local_trajectory, Robot_complete_position* position_robot, Intermediate_LCDS_KP* ILKP)
{
    //! EXCEPTION : clear Local_trajectory.
    reset_Pixel_position_vector(Local_trajectory);
    if(Local_destination->idx_col == -1 && Local_destination->idx_row == -1) 
    {
        return;
    }

    Pixel_position origine_robot_centre((int)LCDS_compute->cols/2, (int)LCDS_compute->rows/2);
    
    //! Convert new format to old one.
    Pair current_position = {origine_robot_centre.idx_col, origine_robot_centre.idx_row};
    Pair destination      = {  Local_destination->idx_col,   Local_destination->idx_row};

    //! Old format use Pair for A*.
    std::vector<Pair> local_trajectory_pair_format;
    bool trajectory_found = aStarSearch(*LCDS_compute, current_position, destination, redis, 1, &local_trajectory_pair_format);

    if(trajectory_found)
    {   
        //! Convert old format to new format.
        int counter = 0;
        for(auto path_px : local_trajectory_pair_format)
        {
            Local_trajectory->at(counter).idx_col = path_px.first;
            Local_trajectory->at(counter).idx_row = path_px.second;
            counter++;
        }

        // Check for new ILKP.
        if(!is_the_same(&ILKP->px_onLCDS, Local_destination))
        {
            int size_trajectory     = local_trajectory_pair_format.size();
            Pixel_position LCDS_opencv_origine(0,0);
            Pixel_position LCDS_robot_origine((int)(LCDS_compute->cols/2), (int)(LCDS_compute->rows/2));

            //! size max est égale à 70% d'une demi diagonal.
            int size_trajectory_max = (int)(distance_btw_pixel(LCDS_opencv_origine, LCDS_robot_origine,1) * 0.7);

            if(size_trajectory >= size_trajectory_max)
            {
                //! We select the 80% index of this path.
                int index_selection_ILKP = (int)(size_trajectory*0.8);
                
                // setup new ILKP.
                Pixel_position reset_model(-1,-1);

                //! We don't use ILKP for this epoque, so we don't need to save this pixel coordinate.
                ILKP->px_onLCDS = reset_model;
                ILKP->distance  = distance_btw_pixel(origine_robot_centre, Local_trajectory->at(index_selection_ILKP),0.05);
                ILKP->angle     = compute_angle_btw_px(&origine_robot_centre, &Local_trajectory->at(index_selection_ILKP));
            }
        }
    }
    if(!trajectory_found)
    {
        return;
    }
}

bool is_the_same(Pixel_position* px_A, Pixel_position* px_B)
{
    if(px_A->idx_col == px_B->idx_col && px_A->idx_row == px_B->idx_row) return true;
    return false;
}

void reset_Pixel_position_vector(std::vector<Pixel_position>* Local_trajectory)
{
    Pixel_position reset_model(-1,-1);
    for(auto px : *Local_trajectory)
    {
        px = reset_model;
    }
}

//TODO: FUNCTION DE DEBUGAGE.

void debug_data(std::vector<Lidar_data>* new_lidar_sample, std::vector<Lidar_sample>* lidarWindows)
{
    //TODO: Cette fonction va servir a voir si certaine données sont correcte.

    //! SHOW LIDAR SAMPLE
    /* std::vector<Lidar_data>* new_lidar_sample (Put an param)
    int i = 0;
    for(auto lidar_data : *new_lidar_sample)
    {
        std::cout << i << " " << lidar_data.angle << " | " << lidar_data.value << std::endl;
        i ++;
    }*/

    //! SHOW LIDAR WINDOWS
    /* std::vector<Lidar_sample>* lidarWindows (Put an param)
    for(int i = 0; i < 1; i++)
    {
        for(int idx_sample = 0; idx_sample < lidarWindows->capacity(); idx_sample++)
        {
            if(i == 0)
            {
                // std::cout << lidarWindows->at(idx_sample).viewpoint.x << " " << lidarWindows->at(idx_sample).viewpoint.y << " " << lidarWindows->at(idx_sample).viewpoint.detal_yaw_deg << " | ";
            }
            if(i >= 0)
            {
                std::cout << lidarWindows->at(idx_sample).observation[i].angle << "|";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    */

}

void debug2_data(std::vector<Pixel_position>* GPKP, std::vector<bool>* GPKP_notYetReached_b)
{
    //! TEST GPKP update from redis.
    // for(auto KP_GMAP : *GPKP)
    // {
    //     std::cout << KP_GMAP.idx_row << "|";
    // }

    //! TEST GPKP_notYetReached_b.
    // for(int i = 0; i < GPKP->capacity(); i++)
    // {
    //     if(GPKP->at(i).idx_col == -1) break;
    //     std::cout << GPKP->at(i).idx_row << " | " << GPKP_notYetReached_b->at(i) << std::endl;
    // }
    // std::cout << std::endl;

    //! TEST GPKP_onLCDS.
    std::cout << "YESSHURE" << std::endl;
    for(auto PKP : *GPKP)
    {
        std::cout << PKP.idx_col << " " << PKP.idx_row << std::endl;
        if(PKP.idx_col == -1) break;
    }
}

void debug_alpha(cv::Mat* LCDS_color, std::vector<Pixel_position>* LW_onLCDS, std::vector<Pixel_position>* GPKP_onLCDS, std::vector<Lidar_sample>* lidarWindows, Robot_complete_position* position_robot, Pixel_position* Local_destination, std::vector<Pixel_position>* Local_trajectory, Intermediate_LCDS_KP* ILKP)
{
    //TODO: Cette fonction va permettre de visualiser si les données obtenu sont correcte.

    double windows_col = LCDS_color->cols;
    double windows_row = LCDS_color->rows;
    double m_LCDS_demi_width = LCDS_color->cols * 0.05 / 2;

    cv::Mat LCDS_color_clone = LCDS_color->clone();
    debug_add_robotShape(&LCDS_color_clone);

    // Ajouter les GPKP sur la LCDS.
    if(true)
    {
        for(auto PKP_onLCDS : *GPKP_onLCDS)
        {
            cv::circle(LCDS_color_clone, cv::Point((int)(PKP_onLCDS.idx_col),(int)(PKP_onLCDS.idx_row)),0, cv::Scalar(224,0,136), cv::FILLED, 0,0);
        }
    }

    // Ajouter la LW sur la LCDS.
    if(true)
    {
        for(auto P_lidar_data : *LW_onLCDS)
        {
            cv::circle(LCDS_color_clone, cv::Point((int)(P_lidar_data.idx_col),(int)(P_lidar_data.idx_row)),0, cv::Scalar(60,80,100), cv::FILLED, 0,0);
        }
    }

    // Ajouter les points d'observation LIDAR.
    if(true)
    {
        for(auto sample : *lidarWindows)
        {
            // project les points d'observation lidar.
            int col = -1; int row = -1;

            double distance_Frame_to_LidarObservationRef = sqrt(pow(sample.viewpoint.delta_x,2)+pow(sample.viewpoint.delta_y,2));
            double debug_angle_rad = deg_to_rad(compute_angle_btw_Robot_and_observation_pts(sample.viewpoint.x, sample.viewpoint.y, position_robot) - 90);

            //! On rajoute - 90 pour passer dans le ref de OpenCV.
            col = cos(debug_angle_rad - M_PI_2)*distance_Frame_to_LidarObservationRef/0.05+(windows_col/2);
            row = sin(debug_angle_rad - M_PI_2)*distance_Frame_to_LidarObservationRef/0.05+(windows_row/2)-6.0;

            cv::circle(LCDS_color_clone, cv::Point((int)(col),(int)(row)),0, cv::Scalar(0,150,250), cv::FILLED, 0,0);
        }
    }

    // Ajouter Part B visualisation.
    if(true)
    {
        // No destination detect.
        if(Local_destination->idx_col == -1 && Local_destination->idx_row)
        {
            cv::circle(LCDS_color_clone, cv::Point((int)(windows_col),(int)(windows_row * 0.75)), 5, cv::Scalar(0,0,255), cv::FILLED, 0,0);     
        }
        else
        {
            // draw local trajectory.
            for(auto px : *Local_trajectory)
            {
                cv::circle(LCDS_color_clone, cv::Point((int)(px.idx_col),(int)(px.idx_row)), 0, cv::Scalar(0,255,115), cv::FILLED, 0,0);  
            }
            // draw destination.
            if(is_the_same(&ILKP->px_onLCDS, Local_destination))
            {
                cv::circle(LCDS_color_clone, cv::Point((int)(Local_destination->idx_col),(int)(Local_destination->idx_row)), 0, cv::Scalar(255,130,255), cv::FILLED, 0,0);  
            }
            else
            {
                cv::circle(LCDS_color_clone, cv::Point((int)(Local_destination->idx_col),(int)(Local_destination->idx_row)), 0, cv::Scalar(255,130,130), cv::FILLED, 0,0); 
            }
        }
    }

    // Visualiser le resultat.
    if(true)
    {
        cv::namedWindow("Debug_LCDS",cv::WINDOW_AUTOSIZE);
        cv::resize(LCDS_color_clone, LCDS_color_clone, cv::Size(0,0),5.0,5.0,6);
        cv::imshow("Debug_LCDS", LCDS_color_clone);
        char d=(char)cv::waitKey(25);
    }
}

void debug_add_robotShape(cv::Mat* LCDS_color)
{
    //TODO: Cette fonction va dessiner le robot sur LCDS.

    int origin_col = -1; int origin_row = -1; 
    int robot_col = 9; int robot_row = 12;      // 9*0.05m ~45cm 
    origin_col = (int)((LCDS_color->cols/2)-robot_col/2);
    origin_row = (int)((LCDS_color->rows/2)-robot_row/2);

    // draw the shape.
    for(int col = 0; col < robot_col; col++)
    {
        for(int row = 0; row < robot_row; row++)
        {
            cv::circle(*LCDS_color, cv::Point((int)(origin_col+col),(int)(origin_row+row)),0, cv::Scalar(0,0,0), cv::FILLED, 0,0);
        }
    }

    // draw the lidar cell.
    cv::circle(*LCDS_color, cv::Point((int)(LCDS_color->cols/2),(int)(LCDS_color->rows/2-robot_row/2)),0, cv::Scalar(0,0,255), cv::FILLED, 0,0);

    // draw the center cell.
    cv::circle(*LCDS_color, cv::Point((int)(LCDS_color->cols/2),(int)(LCDS_color->rows/2)),0, cv::Scalar(0,255,0), cv::FILLED, 0,0);
}
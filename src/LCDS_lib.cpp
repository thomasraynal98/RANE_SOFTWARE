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
    double m_LCDS_demi_width = LCDS_color->cols * 0.05 / 2;

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
            col_PKP = sin(deg_to_rad(180-angle_RKP_onLCDS))*distance_RKP*(windows_row/2)/m_LCDS_demi_width+(windows_col/2);
            row_PKP = cos(deg_to_rad(180-angle_RKP_onLCDS))*distance_RKP*(windows_row/2)/m_LCDS_demi_width+(windows_col/2);

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

    double angle_RKP_GMAP   = rad_to_deg(angle_rad);
    double angle_robot_GMAP = position_robot->yaw_deg_center;

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

double deg_to_rad(double deg)
{
    return deg * M_PI / 180;
}

double rad_to_deg(double rad)
{
    return rad * 180 / M_PI;
}

void project_LW_onLCDS(Robot_complete_position* position_robot, std::vector<Lidar_sample>* lidarWindows, std::vector<Pixel_position>* LW_onLCDS, cv::Mat* LCDS_color)
{
    //TODO: Cette fonction qui est la grande nouveauté de LCDS 3.0 va projecter dans la LCDS Windows
    //TODO: L'intégralité des x derniers samples obtenu par le lidar en prenant en compte le 
    //TODO: déplacement du lidar entre ces x derniers samples. Cette fonction ce rapproche un peux
    //TODO: d'une fonction de mapping sur un lapse de temps très court. Ce qui permet d'avoir un
    //TODO: drift minimun et de ne pas a implementer d'autre fonction de SLAM.

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

            //?? 6. Filter data of mouvement.
            //?? L'idée est de lisser le mouvements de déplacement.

            //??
            std::cout << lidarWindows->at(idx).viewpoint.delta_x << " " << lidarWindows->at(idx).viewpoint.delta_y << " " << lidarWindows->at(idx).viewpoint.detal_yaw_deg << std::endl;
        }
    }
    //??
    std::cout << std::endl;

    // 6. Project chaque sample dans la LCDS.
    int counter_lidar_data = 0; int col_lidar_data = -1; double row_lidar_data = -1;
    for(int idx_sample = 0; idx_sample < lidarWindows->capacity(); idx_sample++)
    {
        if(lidarWindows->at(idx_sample).viewpoint.x != 0 && \
           lidarWindows->at(idx_sample).viewpoint.y != 0 && \
           lidarWindows->at(idx_sample).viewpoint.yaw_deg != 0)
        { 
            for(int idx_data = 0; idx_data < lidarWindows->at(idx_sample).observation.capacity(); idx_data++)
            {
                row_lidar_data = col_lidar_data = -1;
                if(lidarWindows->at(idx_sample).observation[idx_data].value == -1) break;

                //! Verification de FORMULE obligatoire. Notament col<>row, et x<>y.
                //! Le +6.0 correspond au faite que lidar est situé 30 cm en avant du centre du robot.
                col_lidar_data = sin(lidarWindows->at(idx_sample).observation[idx_data].angle-(deg_to_rad(lidarWindows->at(idx_sample).viewpoint.detal_yaw_deg)))*(lidarWindows->at(idx_sample).observation[idx_data].value)*(windows_row/2)/m_LCDS_demi_width+(windows_col/2)-(lidarWindows->at(idx_sample).viewpoint.delta_x/0.05);
                row_lidar_data = cos(lidarWindows->at(idx_sample).observation[idx_data].angle-(deg_to_rad(lidarWindows->at(idx_sample).viewpoint.detal_yaw_deg)))*(lidarWindows->at(idx_sample).observation[idx_data].value)*(windows_row/2)/m_LCDS_demi_width+(windows_col/2)-(lidarWindows->at(idx_sample).viewpoint.delta_y/0.05)-6.0;

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

void debug_alpha(cv::Mat* LCDS_color, std::vector<Pixel_position>* LW_onLCDS, std::vector<Pixel_position>* GPKP_onLCDS)
{
    //TODO: Cette fonction va permettre de visualiser si les données obtenu sont correcte.

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
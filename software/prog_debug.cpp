#include <sw/redis++/redis++.h>
#include <iostream>
#include <thread>

#include <chrono>

using namespace sw::redis;
auto redis = Redis("tcp://127.0.0.1:6379");

int start(sw::redis::Redis* redis)
{   
    std::string init = "init";
    redis->set("Param_K", init);
    redis->set("Param_V", init);
    redis->set("Param_F", init);
    redis->set("Param_back_angle", init);
    redis->set("Param_stall_pwm", init);
    redis->set("Param_unstall_pwm", init);
    redis->set("Param_server_adress", init);
    redis->set("Param_distance_btw_kp", init);
    redis->set("Param_localisation", init);
    redis->set("Param_id_current_map", init);
    redis->set("Param_type_current_map", init);
    redis->set("Param_saved_map", init);
    redis->set("Param_modele", init);
    redis->set("Param_version", init);
    redis->set("Param_matricule", init);
    redis->set("Param_exploitation", init);
    redis->set("Param_link_session", init);
    redis->set("Param_link_png", init);
    redis->set("Param_prenom", init);
    redis->set("Param_robot_length", init);

    redis->set("State_slamcore", init);
    redis->set("State_robot_position", init);
    redis->set("State_robot_position_center", init);
    redis->set("State_robot_position_png", init);
    redis->set("State_robot_speed", init);
    redis->set("State_global_path_is_computing", "false");
    redis->set("State_global_path", "no_path");
    redis->set("State_is_autonomous", "false");
    redis->set("State_destination_is_reach", "false");
    redis->set("State_position_to_reach", "");
    redis->set("State_need_compute_global_path", "false");
    redis->set("State_map_validate", "true");
    redis->set("State_map_available", "true");
    redis->set("State_stream", "OFF");
    redis->set("State_stream_LCDS", "OFF");
    redis->set("State_distance_destination", "-1");

    redis->set("State_order", "NO_ORDER");
    redis->set("State_connection_base", "DISCONNECTED");
    redis->set("State_robot", "WAITING");
    redis->set("State_module_identifiant", "XXXXXXXXXXXXX");
    redis->set("State_base_identifiant", "RANE_MK3_KODA_1");

    redis->set("Error_debug", "NO_ERROR");
    redis->set("Info_debug", "NO_INFO");
}

int main()
{
    ///TIMER//////////////////////////////////////////////////////////////////////////////////////////////////
    int frequency       = 10;
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    ///END////////////////////////////////////////////////////////////////////////////////////////////////////

    start(&redis);

    while(true)
    {
        ///TIMER///////////////////////////////////////////////////////////////////
        x                          = std::chrono::high_resolution_clock::now();         
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        ///END/////////////////////////////////////////////////////////////////////
        std::system("clear");

        std::cout << "Param_K=" << *redis.get("Param_K") << std::endl;
        std::cout << "Param_V=" << *redis.get("Param_V") << std::endl;
        std::cout << "Param_F=" << *redis.get("Param_F") << std::endl;
        std::cout << "Param_back_angle=" << *redis.get("Param_back_angle") << std::endl;
        std::cout << "Param_stall_pwm=" << *redis.get("Param_stall_pwm") << std::endl;
        std::cout << "Param_unstall_pwm=" << *redis.get("Param_unstall_pwm") << std::endl;
        std::cout << "Param_server_adress=" << *redis.get("Param_server_adress") << std::endl;
        std::cout << "Param_distance_btw_kp=" << *redis.get("Param_distance_btw_kp") << std::endl;
        std::cout << "Param_localisation=" << *redis.get("Param_localisation") << std::endl;
        std::cout << "Param_id_current_map=" << *redis.get("Param_id_current_map") << std::endl;
        std::cout << "Param_type_current_map=" << *redis.get("Param_type_current_map") << std::endl;
        std::cout << "Param_saved_map=" << *redis.get("Param_saved_map") << std::endl;
        std::cout << "Param_modele=" << *redis.get("Param_modele") << std::endl;
        std::cout << "Param_version=" << *redis.get("Param_version") << std::endl;
        std::cout << "Param_matricule=" << *redis.get("Param_matricule") << std::endl;
        std::cout << "Param_exploitation=" << *redis.get("Param_exploitation") << std::endl;
        std::cout << "Param_link_session=" << *redis.get("Param_link_session") << std::endl;
        std::cout << "Param_link_png=" << *redis.get("Param_link_png") << std::endl;
        std::cout << "Param_prenom=" << *redis.get("Param_prenom") << std::endl;
        std::cout << "Param_robot_length=" << *redis.get("Param_robot_length") << std::endl << std::endl;

        std::cout << "State_slamcore=" << *redis.get("State_slamcore") << std::endl;
        std::cout << "State_robot_position=" << *redis.get("State_robot_position") << std::endl;
        std::cout << "State_robot_position_center=" << *redis.get("State_robot_position_center") << std::endl;
        std::cout << "State_robot_position_png=" << *redis.get("State_robot_position_png") << std::endl;
        std::cout << "State_robot_speed=" << *redis.get("State_robot_speed") << std::endl;
        std::cout << "State_global_path_is_computing=" << *redis.get("State_global_path_is_computing") << std::endl;
        std::cout << "State_global_path=" << *redis.get("State_global_path") << std::endl;
        std::cout << "State_is_autonomous=" << *redis.get("State_is_autonomous") << std::endl;
        std::cout << "State_destination_is_reach=" << *redis.get("State_destination_is_reach") << std::endl;
        std::cout << "State_position_to_reach=" << *redis.get("State_position_to_reach") << std::endl;
        std::cout << "State_need_compute_global_path=" << *redis.get("State_need_compute_global_path") << std::endl;
        std::cout << "State_map_validate=" << *redis.get("State_map_validate") << std::endl;
        std::cout << "State_map_available=" << *redis.get("State_map_available") << std::endl;
        std::cout << "State_distance_destination=" << *redis.get("State_distance_destination") << std::endl;
        std::cout << "State_stream=" << *redis.get("State_stream") << std::endl;
        std::cout << "State_stream_LCDS=" << *redis.get("State_stream_LCDS") << std::endl << std::endl;

        std::cout << "State_order=" << *redis.get("State_order") << std::endl;
        std::cout << "State_connection_base=" << *redis.get("State_connection_base") << std::endl;
        std::cout << "State_robot=" << *redis.get("State_robot") << std::endl;
        std::cout << "State_module_identifiant=" << *redis.get("State_module_identifiant") << std::endl;
        std::cout << "State_base_identifiant=" << *redis.get("State_base_identifiant") << std::endl;

        std::cout << "\nError_debug=" << *redis.get("Error_debug") << std::endl;
        std::cout << "Info_debug=" << *redis.get("Info_debug") << std::endl;
    }

    return 0;
}

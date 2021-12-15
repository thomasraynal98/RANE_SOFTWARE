#include <sw/redis++/redis++.h>

void init_variable_REDIS(sw::redis::Redis* redis);

bool read_ID_file_and_push(sw::redis::Redis* redis);
bool read_PARAM_file_and_push(sw::redis::Redis* redis);
bool read_MAP_file_and_push(sw::redis::Redis* redis);

bool write_ID_file(sw::redis::Redis* redis);
bool write_PARAM_file(sw::redis::Redis* redis);
bool write_MAP_file(sw::redis::Redis* redis);

void check_map_available(sw::redis::Redis* redis);
void download_map_file(sw::redis::Redis* redis);
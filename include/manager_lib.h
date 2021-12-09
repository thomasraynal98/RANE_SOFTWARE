bool read_ID_file_and_push(sw::redis::Redis* redis);
bool read_PARAM_file_and_push(sw::redis::Redis* redis);
bool read_MAP_file_and_push(sw::redis::Redis* redis);

bool write_ID_file(sw::redis::Redis* redis);
bool write_PARAM_file(sw::redis::Redis* redis);
bool write_MAP_file(sw::redis::Redis* redis);
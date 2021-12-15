CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(rane_software VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)

# include Socket IO.
include(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/socket.io-client-cpp/CMakeLists.txt)

# set good gcc version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -lredis++ -lhiredis -o CPUUsage")

# found package.
FIND_PACKAGE(OpenCV 4 REQUIRED)
FIND_PACKAGE(ydlidar_sdk REQUIRED)
FIND_PACKAGE(slamcore_sdk REQUIRED)

include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

# add the executable exemple.
ADD_EXECUTABLE(exempleA 
    software/exempleA.cpp)

TARGET_LINK_LIBRARIES(exempleA
    redis++
    pthread
    hiredis)

ADD_EXECUTABLE(exempleB
    software/exempleB.cpp)

TARGET_LINK_LIBRARIES(exempleB
    redis++
    pthread
    hiredis)

# PROGRAMME YAML MANAGER.
ADD_EXECUTABLE(prog_data_manager
    software/prog_data_manager.cpp
    src/manager_lib.cpp 
    include/manager_lib.h)

TARGET_LINK_LIBRARIES(prog_data_manager
    redis++
    pthread
    hiredis
    ${OpenCV_LIBS})

# PROGRAMME LIDAR MANAGER.
ADD_EXECUTABLE(prog_lidar
    software/prog_lidar.cpp
    src/lidar_lib.cpp 
    include/lidar_lib.h)

TARGET_LINK_LIBRARIES(prog_lidar
    redis++
    pthread
    hiredis
    ${YDLIDAR_SDK_LIBRARIES})

# PROGRAMME MICROCONTROLER MANAGER.
ADD_EXECUTABLE(prog_com_microcontroler
    software/prog_com_microcontroler.cpp
    src/com_micro_lib.cpp 
    include/com_micro_lib.h)

TARGET_LINK_LIBRARIES(prog_com_microcontroler
    redis++
    pthread
    hiredis
    /usr/local/lib/libserial.so)

# PROGRAMME COMMUNICATION SERVER.
ADD_EXECUTABLE(prog_com_server
software/prog_com_server.cpp
src/com_server_lib.cpp 
include/com_server_lib.h)

TARGET_LINK_LIBRARIES(prog_com_server
    redis++
    pthread
    hiredis
    sioclient)

# PROGRAMME GLOBAL PATH.
ADD_EXECUTABLE(prog_vslam
software/prog_vslam.cpp
src/vslam_lib.cpp 
include/vslam_lib.h)

TARGET_LINK_LIBRARIES(prog_vslam
    redis++
    pthread
    hiredis
    slamcore::slamcore
    ${OpenCV_LIBS})
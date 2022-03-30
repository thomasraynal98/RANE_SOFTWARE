CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(rane_software VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)

# include Socket IO.
#include(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/socket.io-client-cpp/CMakeLists.txt)

# set good gcc version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -lredis++ -lhiredis -L/usr/local/lib -lserial -o CPUUsage")

# found package.
FIND_PACKAGE(OpenCV 4 REQUIRED)
FIND_PACKAGE(ydlidar_sdk REQUIRED)
FIND_PACKAGE(slamcore_sdk REQUIRED)
FIND_PACKAGE(websocketpp REQUIRED)

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
    hiredis
    ${OpenCV_LIBS})

ADD_EXECUTABLE(exempleB
    software/exempleB.cpp)

TARGET_LINK_LIBRARIES(exempleB
    redis++
    pthread
    hiredis
    ${OpenCV_LIBS})

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
    serial)

# PROGRAMME COMMUNICATION SERVER.
ADD_EXECUTABLE(prog_com_server
software/prog_com_server.cpp
src/com_server_lib.cpp 
include/com_server_lib.h
src/ConvertImage.cpp
include/ConvertImage.h)

TARGET_LINK_LIBRARIES(prog_com_server
    redis++
    pthread
    hiredis
    sioclient
    ${OpenCV_LIBS})

# PROGRAMME VSLAM.
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

# PROGRAMME GLOBAL PATH.
ADD_EXECUTABLE(prog_global_path
software/prog_global_path.cpp
src/global_path_lib.cpp 
include/global_path_lib.h)

TARGET_LINK_LIBRARIES(prog_global_path
    redis++
    pthread
    hiredis
    ${OpenCV_LIBS})

# PROGRAMME LOCAL PATH.
ADD_EXECUTABLE(prog_local_path
software/prog_local_path.cpp
src/local_path_lib.cpp 
include/local_path_lib.h
src/global_path_lib.cpp 
include/global_path_lib.h)

TARGET_LINK_LIBRARIES(prog_local_path
    redis++
    pthread
    hiredis
    ${OpenCV_LIBS})

# PROGRAMME DEBUG REDIS.
ADD_EXECUTABLE(prog_debug
software/prog_debug.cpp)

TARGET_LINK_LIBRARIES(prog_debug
    redis++
    pthread
    hiredis
    ${OpenCV_LIBS})

# PROGRAMME MODULE MANAGERS.
ADD_EXECUTABLE(prog_module_mng
software/prog_module_mng.cpp
src/com_micro_lib.cpp 
include/com_micro_lib.h)

TARGET_LINK_LIBRARIES(prog_module_mng
    redis++
    pthread
    hiredis
    serial)

# PROGRAMME MANUAL PILOTAGE.
ADD_EXECUTABLE(manual_pilotage
software/manual_pilotage.cpp)

TARGET_LINK_LIBRARIES(manual_pilotage
    redis++
    pthread
    hiredis)

# PROGRAMME COM LED.
ADD_EXECUTABLE(prog_com_led
software/prog_com_led.cpp)

TARGET_LINK_LIBRARIES(prog_com_led
    redis++
    pthread
    hiredis
    serial)

# PROGRAMME NEW LOCAL COMPREHENSION AND DESCISION SYSTEM. (REFONTE)
ADD_EXECUTABLE(LocalCDS
software/new_LCDS.cpp
src/LCDS_lib.cpp 
include/LCDS_lib.h
src/global_path_lib.cpp 
include/global_path_lib.h)

TARGET_LINK_LIBRARIES(LocalCDS
    redis++
    pthread
    hiredis
    ${OpenCV_LIBS})
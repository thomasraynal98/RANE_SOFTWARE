CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(rane_software VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)

# set good gcc version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -lredis++ -lhiredis -o CPUUsage")

# add the executable.
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
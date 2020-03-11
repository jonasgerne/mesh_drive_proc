cmake_minimum_required(VERSION 2.8.3)
project(mesh_drive_proc)

# added -fopenmp
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(PCL REQUIRED)

cs_add_executable(${PROJECT_NAME}_node
        src/main.cpp)

cs_add_executable(${PROJECT_NAME}_ply_reader src/readply.cpp)

cs_install()

cs_export()

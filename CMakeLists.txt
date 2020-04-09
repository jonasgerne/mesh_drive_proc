cmake_minimum_required(VERSION 2.8.3)
project(mesh_drive_proc)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}/share/catkin/cmake")
set(catkin_simple_DIR "../../devel/share/catkin_simple/cmake")

find_package(catkin_simple REQUIRED)

catkin_simple()

find_package(PCL REQUIRED common geometry)

cs_add_executable(${PROJECT_NAME}_node
        src/main.cpp)

cs_add_executable(${PROJECT_NAME}_ply_reader src/readply.cpp)

cs_install()

cs_export()

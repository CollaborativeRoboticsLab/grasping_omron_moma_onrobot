cmake_minimum_required(VERSION 3.8)
project(grasping_description_onrobot_omron_moma)

find_package(ament_cmake REQUIRED)

install(DIRECTORY xacro DESTINATION share/${PROJECT_NAME})

ament_package()
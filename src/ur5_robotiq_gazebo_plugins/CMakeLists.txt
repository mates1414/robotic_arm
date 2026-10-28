cmake_minimum_required(VERSION 3.0.2)
project(ur5_robotiq_gazebo_plugins)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(ur5_robotiq_grasp_attacher src/grasp_attacher.cpp)
target_link_libraries(ur5_robotiq_grasp_attacher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ur5_robotiq_grasp_attacher LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

cmake_minimum_required(VERSION 3.0.2)
project(ur5_robotiq_manipulation)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/pick_place_demo.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

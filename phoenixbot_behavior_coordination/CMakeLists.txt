cmake_minimum_required(VERSION 2.8.3)
project(phoenixbot_behavior_coordination)
find_package(catkin REQUIRED)
catkin_package()

file(GLOB scripts scripts/*)
catkin_install_python(PROGRAMS ${scripts} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


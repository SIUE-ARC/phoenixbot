cmake_minimum_required(VERSION 2.8.3)
project(phoenixbot_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    phoenixbot_msgs
    hardware_interface
    joint_limits_interface
    controller_manager
    serial
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_executable(phoenixbot_driver src/phoenixbot_driver.cpp src/PhoenixbotInterface.cpp)
add_dependencies(phoenixbot_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(phoenixbot_driver
    ${catkin_LIBRARIES}
)

install(TARGETS phoenixbot_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


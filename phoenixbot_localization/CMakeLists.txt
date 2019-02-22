cmake_minimum_required(VERSION 2.8.3)
project(phoenixbot_localization)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    cv_bridge
    eigen_conversions
    geometry_msgs
    image_transport
    tf2_eigen
    phoenixbot_msgs
)
find_package(Eigen REQUIRED) 
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 include
)

add_executable(aruco_board src/aruco_board.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(aruco_board ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(aruco_board
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS aruco_board
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


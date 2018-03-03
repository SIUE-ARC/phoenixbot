cmake_minimum_required(VERSION 2.8.3)
project(phoenixbot_localization)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    cv_bridge
    OpenCV
    image_transport
    geometry_msgs
    tf2_eigen
    eigen_conversions
    )
find_package(Eigen REQUIRED) 

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
 ${Eigen_INCLUDE_DIRS}
 include
)

add_executable(aruco_board src/aruco_board.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(aruco_board ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(aruco_board
  ${catkin_LIBRARIES}
)


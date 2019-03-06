cmake_minimum_required(VERSION 2.8.3)
project(wit_imu)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  cmake_modules
  serial
  eigen_conversions
  roscpp
)
find_package(Eigen REQUIRED) 

catkin_package()

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(wit_imu src/wit_imu.cpp)

add_dependencies(wit_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(wit_imu
  ${catkin_LIBRARIES}
)


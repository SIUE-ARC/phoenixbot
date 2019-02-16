cmake_minimum_required(VERSION 2.8.3)
project(beacon_transforms)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	     cmake_modules
	     geometry_msgs
	     marvelmind_nav
	     tf2_eigen
	     tf_conversions
	     eigen_conversions
			 tf
			 sensor_msgs
)


find_package(Eigen REQUIRED)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
 include
)

add_executable(beacon_tf src/beacon_tf.cpp)
add_dependencies(beacon_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(beacon_imu src/beacon_imu.cpp)
add_dependencies(beacon_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(beacon_tf
  ${catkin_LIBRARIES}
)

install(TARGETS beacon_tf beacon_tf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(beacon_imu
  ${catkin_LIBRARIES}
)

install(TARGETS beacon_imu beacon_imu
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

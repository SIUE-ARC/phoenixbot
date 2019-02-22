cmake_minimum_required(VERSION 2.8.3)
project(phoenixbot_description)
find_package(catkin REQUIRED)
catkin_package()

# Parse xacro files
find_package(xacro REQUIRED)
file(GLOB xacro_files urdf/*.urdf.xacro)
xacro_add_files(${xacro_files} INORDER TARGET media_files)

# Install targets
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  FILES_MATCHING PATTERN "*.urdf")

install(DIRECTORY meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)


 ################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(aws_robomaker_hospital_world)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
set(DEPENDENCIES
  ament_cmake
  rclcpp
  ros_gz_sim
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# Install
################################################################################

install(DIRECTORY launch models fuel_models worlds
	DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY photos
  DESTINATION share
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/aws_robomaker_hospital_world.dsv.in")

################################################################################
# Macro for ament package
################################################################################
ament_package()

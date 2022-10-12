cmake_minimum_required(VERSION 3.5)
project(vehicle_simulator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(vehicleSimulator src/vehicleSimulator.cpp)
ament_target_dependencies(vehicleSimulator rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions gazebo_ros gazebo_msgs)
target_link_libraries(vehicleSimulator ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  vehicleSimulator
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  log
  mesh
  rviz
  urdf
  world
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


# cmake_minimum_required(VERSION 2.8.3)
# project(vehicle_simulator)

# set(CMAKE_BUILD_TYPE Release)
# set(BUILD_STATIC_LIBS ON)
# set(BUILD_SHARED_LIBS OFF)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# ## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   sensor_msgs
#   pcl_ros
# )

# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)

# ###################################
# ## catkin specific configuration ##
# ###################################

# catkin_package(
#   CATKIN_DEPENDS
#   roscpp
#   std_msgs
#   sensor_msgs
#   pcl_ros
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
#   "${PROJECT_SOURCE_DIR}/include"
#   /usr/local/include # Location when using 'make system_install'
#   /usr/include       # More usual location (e.g. when installing using a package)
# )

# ## Specify additional locations for library files
# link_directories(
#   /usr/local/lib # Location when using 'make system_install'
#   /usr/lib       # More usual location (e.g. when installing using a package)
# )

# ## Declare executables
# add_executable(vehicleSimulator src/vehicleSimulator.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(vehicleSimulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# install(TARGETS vehicleSimulator
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY log/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/log
# )
# install(DIRECTORY mesh/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh
# )
# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
# )
# install(DIRECTORY urdf/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
# )
# install(DIRECTORY world/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
# )

cmake_minimum_required(VERSION 2.8.12)
project(aist_phoxi_camera)

## Compile as C++17, supported in ROS melodic and newer
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG -mavx2 -DAVX2 -Wunknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE	"")
set(CMAKE_CXX_FLAGS_DEBUG	"-g -DPROFILE")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    image_transport
    sensor_msgs
    ddynamic_reconfigure
    std_srvs
    tf
    pluginlib
    nodelet
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PHOXI_INCLUDE_DIRS}
)

add_definitions(-DPHO_SOFTWARE_VERSION_MAJOR=${PHO_SOFTWARE_VERSION_MAJOR}
	        -DPHO_SOFTWARE_VERSION_MINOR=${PHO_SOFTWARE_VERSION_MINOR})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    sensor_msgs
)

add_library(
  ${PROJECT_NAME}
  src/Camera.cpp
  src/nodelet.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${PHOXI_LIBRARY}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    ${PHOXI_LIB_RELEASE_PER_COMPILER}.${PHO_SOFTWARE_VERSION}
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/check_depth_consistency.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

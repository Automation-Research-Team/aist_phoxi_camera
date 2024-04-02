cmake_minimum_required(VERSION 3.5)
project(aist_phoxi_camera)

## Compile as C++17 code
enable_language(CXX)

set(CMAKE_CXX_STANDARD		17)
set(CMAKE_CXX_STANDARD_REQUIRED	ON)
set(CMAKE_CXX_EXTENSIONS	OFF)
set(CMAKE_CXX_FLAGS		"-Wall -O3 -DNDEBUG -mavx2 -DAVX2 -Wunknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE	"")
set(CMAKE_CXX_FLAGS_DEBUG	"-g -DPROFILE")
set(CMAKE_CXX_BUILD_TYPE	Release)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(ddynamic_reconfigure REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_include_directories(
  ${PHOXI_INCLUDE_DIRS}
)

add_definitions(-DPHO_SOFTWARE_VERSION_MAJOR=${PHO_SOFTWARE_VERSION_MAJOR}
	        -DPHO_SOFTWARE_VERSION_MINOR=${PHO_SOFTWARE_VERSION_MINOR})

add_library(
  ${PROJECT_NAME} SHARED
    src/Camera.cpp
    src/nodelet.cpp
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp
    rclcpp_components
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME} ${PROJECT_NAME}::Camera
)

target_link_libraries(
  ${PROJECT_NAME}
    ${PHOXI_LIBRARY}
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

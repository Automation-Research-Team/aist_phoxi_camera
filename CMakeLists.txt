cmake_minimum_required(VERSION 3.8)
project(aist_phoxi_camera)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(rclcpp_components REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ddynamic_reconfigure2 REQUIRED)

add_definitions(-DPHO_SOFTWARE_VERSION_MAJOR=${PHO_SOFTWARE_VERSION_MAJOR}
	        -DPHO_SOFTWARE_VERSION_MINOR=${PHO_SOFTWARE_VERSION_MINOR})

########################################
#  component: libaist_phoxi_camera.so  #
########################################
add_library(
  ${PROJECT_NAME}
  SHARED
    src/Camera.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
    COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp_components
    image_transport
    sensor_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
    ddynamic_reconfigure2
)

target_link_libraries(
  ${PROJECT_NAME}
    ${PHOXI_LIBRARY}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME} ${PROJECT_NAME}::Camera
)

##########
#  test  #
##########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(aist_phoxi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_definitions(-DPHO_SOFTWARE_VERSION_MAJOR=${PHO_SOFTWARE_VERSION_MAJOR}
	        -DPHO_SOFTWARE_VERSION_MINOR=${PHO_SOFTWARE_VERSION_MINOR})

add_library(
  ${PROJECT_NAME} SHARED
    src/Camera.cpp
    src/nodelet.cpp
)

target_compile_features(
  ${PROJECT_NAME} PUBLIC
    c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(
  ${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE COMPOSITION_BUILDING_DLL
)

ament_target_dependencies(
  ${PROJECT_NAME}
    rclcpp
    rclcpp_components
    image_transport
    sensor_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
)

rclcpp_components_register_nodes(
  ${PROJECT_NAME} ${PROJECT_NAME}::Camera
)

target_link_libraries(
  ${PROJECT_NAME}
    ${PHOXI_LIBRARY}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS aist_phoxi_camera
  DESTINATION lib/${PROJECT_NAME})

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

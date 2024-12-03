cmake_minimum_required(VERSION 3.8)
project(vicon_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_VERSION VERSION_LESS "3.12")
  target_link_libraries(${PROJECT_NAME} stdc++fs)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mess2_msgs REQUIRED)
find_package(mess2_plugins REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(_vicon_calibration_server SHARED src/server.cpp)
add_library(_vicon_calibration_client SHARED src/client.cpp)
# add_library(_vicon_calibration_client_fake SHARED src/client_fake.cpp)

target_include_directories(
  _vicon_calibration_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
)

# target_include_directories(
#   _vicon_calibration_client_fake PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   $<BUILD_INTERFACE:${mess2_plugins_INCLUDE_DIRS}>
# )

target_compile_definitions(
  _vicon_calibration_server
  PRIVATE "VICON_CALIBRATION_SERVER_BUILDING_DLL"
)

target_compile_definitions(
  _vicon_calibration_client
  PRIVATE "VICON_CALIBRATION_CLIENT_BUILDING_DLL"
)

# target_compile_definitions(
#   _vicon_calibration_client_fake
#   PRIVATE "VICON_CALIBRATION_CLIENT_FAKE_BUILDING_DLL"
# )

ament_target_dependencies(
  _vicon_calibration_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "mess2_msgs"
  "mess2_plugins"
  "std_msgs"
)

ament_target_dependencies(
  _vicon_calibration_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "mess2_msgs"
  "yaml-cpp"
)

# ament_target_dependencies(
#   _vicon_calibration_client_fake
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "geometry_msgs"
#   "mess2_msgs"
#   "mess2_plugins"
#   "yaml-cpp"
# )

target_link_libraries(_vicon_calibration_client yaml-cpp)
# target_link_libraries(_vicon_calibration_client_fake yaml-cpp)

rclcpp_components_register_node(_vicon_calibration_server PLUGIN "mess2_nodes::VICONCalibrationServer" EXECUTABLE server)
rclcpp_components_register_node(_vicon_calibration_client PLUGIN "mess2_nodes::VICONCalibrationClient" EXECUTABLE client)
# rclcpp_components_register_node(_vicon_calibration_client_fake PLUGIN "mess2_nodes::VICONCalibrationClientFake" EXECUTABLE client_fake)

install(TARGETS
  _vicon_calibration_server
  _vicon_calibration_client
#   _vicon_calibration_client_fake
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

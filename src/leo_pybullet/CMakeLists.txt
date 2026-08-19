cmake_minimum_required(VERSION 3.8)
project(leo_pybullet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Bullet REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(bullet_sim_core SHARED src/bullet_sim_core.cpp)
target_compile_features(bullet_sim_core PUBLIC cxx_std_17)
target_include_directories(bullet_sim_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${BULLET_INCLUDE_DIRS}
)
target_link_libraries(bullet_sim_core
  PRIVATE
    ${BULLET_LIBRARIES}
    yaml-cpp
)

add_executable(pybullet_sim_node src/pybullet_sim_node.cpp)
target_compile_features(pybullet_sim_node PUBLIC cxx_std_17)
target_link_libraries(pybullet_sim_node bullet_sim_core)
add_executable(bullet_core_smoke src/bullet_core_smoke.cpp)
target_compile_features(bullet_core_smoke PUBLIC cxx_std_17)
target_link_libraries(bullet_core_smoke bullet_sim_core)
add_executable(scan_debug_node src/scan_debug_node.cpp)
add_executable(urdf_inspector_node src/urdf_inspector_node.cpp)
add_executable(odom_logger_node src/odom_logger_node.cpp)

ament_target_dependencies(pybullet_sim_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  visualization_msgs
)
ament_target_dependencies(scan_debug_node
  rclcpp
  sensor_msgs
)
ament_target_dependencies(urdf_inspector_node
  rclcpp
  urdf
)
install(TARGETS
  bullet_sim_core
  EXPORT export_leo_pybullet
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(TARGETS
  bullet_core_smoke
  odom_logger_node
  pybullet_sim_node
  scan_debug_node
  urdf_inspector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  results
  DESTINATION share/${PROJECT_NAME}
)
ament_target_dependencies(odom_logger_node
  rclcpp
  nav_msgs
)

ament_export_targets(export_leo_pybullet HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(bullet_sim_core)

ament_package()

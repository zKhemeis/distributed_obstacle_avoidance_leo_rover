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

add_executable(pybullet_sim_node src/pybullet_sim_node.cpp)
target_include_directories(pybullet_sim_node PRIVATE ${BULLET_INCLUDE_DIRS})
target_link_libraries(pybullet_sim_node ${BULLET_LIBRARIES})
add_executable(scan_debug_node src/scan_debug_node.cpp)
add_executable(urdf_inspector_node src/urdf_inspector_node.cpp)


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

ament_package()

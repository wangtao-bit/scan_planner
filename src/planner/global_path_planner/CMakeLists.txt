cmake_minimum_required(VERSION 3.8)
project(global_path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(global_path_planner_core
  src/support_map.cpp
  src/support_astar.cpp)
target_include_directories(global_path_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(global_path_planner_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(global_path_planner_core PUBLIC
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs visualization_msgs)

add_executable(global_path_planner_node src/global_path_planner_node.cpp)
target_link_libraries(global_path_planner_node global_path_planner_core Eigen3::Eigen)
ament_target_dependencies(global_path_planner_node
  geometry_msgs nav_msgs rclcpp sensor_msgs visualization_msgs)

add_executable(offline_plan_smoke src/offline_plan_smoke.cpp)
target_link_libraries(offline_plan_smoke global_path_planner_core Eigen3::Eigen)

install(TARGETS global_path_planner_core global_path_planner_node offline_plan_smoke
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_support_astar test/test_support_astar.cpp)
  target_link_libraries(test_support_astar global_path_planner_core Eigen3::Eigen)
endif()

ament_package()

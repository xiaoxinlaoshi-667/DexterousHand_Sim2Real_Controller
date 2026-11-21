cmake_minimum_required(VERSION 3.8)
project(allegro_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==========================================================
# Target 1: 高维仿真控制节点 (16-DOF Allegro Sim Controller)
# ==========================================================
add_executable(thumb_index_catch src/thumb_index_catch.cpp)
ament_target_dependencies(thumb_index_catch rclcpp std_msgs sensor_msgs)
target_include_directories(thumb_index_catch PUBLIC ${EIGEN3_INCLUDE_DIRS})
install(TARGETS thumb_index_catch DESTINATION lib/${PROJECT_NAME})

# ==========================================================
# Target 2: 实机真手控制节点 (6-DOF Inspire Real Controller)
# ==========================================================
add_executable(inspire_commander src/inspire_commander.cpp)
ament_target_dependencies(inspire_commander rclcpp std_msgs sensor_msgs)
target_include_directories(inspire_commander PUBLIC ${EIGEN3_INCLUDE_DIRS})
install(TARGETS inspire_commander DESTINATION lib/${PROJECT_NAME})

# ==========================================================
# Target 3: 底层硬件通信桥 (RS485 Hardware Bridge)
# ==========================================================
add_executable(inspire_bridge_node src/inspire_bridge_node.cpp)
ament_target_dependencies(inspire_bridge_node rclcpp std_msgs sensor_msgs)
install(TARGETS inspire_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()

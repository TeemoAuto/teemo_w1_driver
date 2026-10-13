cmake_minimum_required(VERSION 3.8)
project(teemo_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
# 更新为新的依赖包
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

include_directories(include)

add_executable(teemo_vehicle_interface_node src/teemo_vehicle_interface_node.cpp)
ament_target_dependencies(teemo_vehicle_interface_node
  rclcpp
  can_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  tier4_vehicle_msgs
)

install(TARGETS teemo_vehicle_interface_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

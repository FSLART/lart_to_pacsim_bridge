cmake_minimum_required(VERSION 3.8)
project(lart_to_pacsim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(pacsim REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_executable(lart_to_pacsim_bridge src/lart_to_pacsim_bridge.cpp)
ament_target_dependencies(
  lart_to_pacsim_bridge 
  rclcpp 
  lart_msgs
  pacsim
  rosidl_default_runtime
)

install(TARGETS lart_to_pacsim_bridge
 RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

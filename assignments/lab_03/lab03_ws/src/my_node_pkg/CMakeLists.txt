cmake_minimum_required(VERSION 3.8)
project(my_node_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(ament_lint_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(MyNode src/MyNode.cpp)
ament_target_dependencies(MyNode rclcpp std_msgs)

install(TARGETS MyNode DESTINATION lib/${PROJECT_NAME})

ament_package()

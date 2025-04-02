cmake_minimum_required(VERSION 3.8)
project(mpc_rbt_student)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(PUBLIC
                    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)

set(dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
    mpc_rbt_simulator
    nav_msgs
    )

add_library(${PROJECT_NAME} SHARED
    src/KeyboardControl.cpp
    src/Localization.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
        
install(DIRECTORY 
	launch
	rviz
	DESTINATION share/${PROJECT_NAME}/) 

add_executable(keyboard_control src/keyboard_control_node.cpp)
target_link_libraries(keyboard_control ${PROJECT_NAME})
install(TARGETS keyboard_control DESTINATION include/${PROJECT_NAME})

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node ${PROJECT_NAME})
install(TARGETS localization_node DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies($dependencies)
ament_export_libraries(${PROJECT_NAME})
ament_package()

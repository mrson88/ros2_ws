cmake_minimum_required(VERSION 3.8)
project(arduinobot_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(arduinobot_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_core REQUIRED)
add_library(task_server SHARED src/task_server.cpp)
# add_library(moveit_action_server SHARED src/moveit_action_server.cpp)
target_include_directories(task_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(task_server
  PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
ament_target_dependencies(task_server
  "arduinobot_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(task_server 
  PLUGIN "arduinobot_remote::TaskServer" 
  EXECUTABLE task_server_node
)


# target_include_directories(moveit_action_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(moveit_action_server
#   PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
# ament_target_dependencies(moveit_action_server
#   "arduinobot_msgs"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "moveit_ros_planning_interface"
#   )
# rclcpp_components_register_node(moveit_action_server 
#   PLUGIN "arduinobot_remote::MoveItActionServer" 
#   EXECUTABLE moveit_action_server_node
# )


# Install executable
# install(TARGETS
#   moveit_action_server
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS 
# moveit_action_server
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   DESTINATION lib/${PROJECT_NAME}
# )
install(PROGRAMS
  ${PROJECT_NAME}/task_server.py
  ${PROJECT_NAME}/alexa_interface.py
  ${PROJECT_NAME}/test.py
  ${PROJECT_NAME}/fibonacci_action_client.py
  ${PROJECT_NAME}/fibonacci_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

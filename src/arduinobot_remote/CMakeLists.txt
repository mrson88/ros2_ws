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
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
add_library(task_server SHARED src/task_server.cpp)
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



# ----------------------------------------------------- start

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arduinobot_msgs REQUIRED)
add_library(action_server SHARED src/gripper_action_server.cpp)
# add_executable(action_server src/gripper_action_server.cpp)

ament_target_dependencies(action_server
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  "sensor_msgs"
  "std_msgs"
  "arduinobot_msgs")

target_include_directories(action_server PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
        PRIVATE "GRIPPER_SERVER_CPP_BUILDING_DLL")

install(TARGETS action_server
    DESTINATION lib/${PROJECT_NAME})

rclcpp_components_register_node(action_server 
    PLUGIN "arduinobot_remote::GripperActionServer" 
    EXECUTABLE gripper_server_node
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
##################### end





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

install(PROGRAMS
  ${PROJECT_NAME}/task_server.py
  ${PROJECT_NAME}/alexa_interface.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()

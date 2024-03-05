cmake_minimum_required(VERSION 3.8)
project(articubot_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(arduinobot_msgs REQUIRED)
find_package(innobot_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_core REQUIRED)





find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)



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
  PLUGIN "articubot_remote::TaskServer" 
  EXECUTABLE task_server_node
)

add_executable(pick_n_place_node src/pick_n_place_node.cpp)
ament_target_dependencies(pick_n_place_node rclcpp moveit_msgs moveit_ros_planning_interface arduinobot_msgs rclcpp_action control_msgs)


# The innobot_node executable
add_executable(innobot_node src/innobot_node.cpp)
ament_target_dependencies(innobot_node rclcpp moveit_msgs moveit_ros_planning_interface innobot_msgs rclcpp_action control_msgs)
# add_library(pick_n_place SHARED src/pick_n_place_node.cpp)
# target_include_directories(pick_n_place PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(pick_n_place
#   PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
# ament_target_dependencies(pick_n_place
#   "arduinobot_msgs"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "moveit_ros_planning_interface"
#   "moveit_msgs"
#   "control_msgs")
# rclcpp_components_register_node(pick_n_place 
#   PLUGIN "articubot_remote::PickNPlace" 
#   EXECUTABLE pick_n_place_node
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
# pick_n_place
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
  ${PROJECT_NAME}/pnp_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


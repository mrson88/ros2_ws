cmake_minimum_required(VERSION 3.8)
project(arduinobot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "srv/EulerToQuaternion.srv"
  "srv/QuaternionToEuler.srv"
  "action/Fibonacci.action"
  "action/ArduinobotTask.action"
  "action/Pnp.action"
  "action/Home.action"
  "action/Move.action"
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()

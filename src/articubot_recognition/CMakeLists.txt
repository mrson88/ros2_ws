cmake_minimum_required(VERSION 3.5)
project(articubot_recognition)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(arduinobot_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake_python REQUIRED)
# ...

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(arduinobot_msgs REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    scripts
    launch
    data
    scripts/model
    scripts/Model
    scripts/utils
    scripts/Facerec_QT
    scripts/MiAI_MobileNetSSD_Pi
  DESTINATION
    share/${PROJECT_NAME}/
)
install(
  DIRECTORY
  scripts/Facerec_QT
  DESTINATION
    lib/${PROJECT_NAME}/
)
install(
  PROGRAMS
  scripts/yolov8_ros2_pt.py
  scripts/yolov8_ros2_subscriber.py
  scripts/yolo_ros2_depth_test.py
  scripts/app.py
  scripts/tflite_reg.py
  # scripts/Facerec_QT/face_rec.py
  # scripts/utils/cvfpscalc.py
  # scripts/model/keypoint_classifier/keypoint_classifier.py
  # scripts/model/point_history_classifier/point_history_classifier.py
  launch/launch_yolov8.launch.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
  scripts/utils/cvfpscalc.py
  DESTINATION lib/${PROJECT_NAME}/utils
)
install(
  PROGRAMS
  scripts/model/keypoint_classifier/keypoint_classifier.py
  scripts/model/point_history_classifier/point_history_classifier.py
  DESTINATION lib/${PROJECT_NAME}/model
)
ament_package()

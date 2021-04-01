cmake_minimum_required(VERSION 3.5)
project(digitaltwin)

# Add support for C++11
if(NOT WIN32)
    add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlA.msg"
  "msg/ControlP.msg"
  "msg/Reward.msg"
  "msg/Sensor.msg"
  "msg/SensorList.msg"
  "msg/State.msg"
  "msg/StateList.msg"
  "msg/ControlPList.msg"
 )

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})


include_directories(include ${rclcpp_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS})

# Add a c++ library
# add_library(${PROJECT_NAME} src/some_file.cpp)
# target_link_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

# Install C++ headers
# install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.hpp")

# Install python scripts
install(PROGRAMS
  nodes/asset.py
  nodes/twin.py
  nodes/logger.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

ament_package()

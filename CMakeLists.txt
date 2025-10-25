cmake_minimum_required(VERSION 3.8)
project(oakd_pcloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgproc highgui)
find_package(depthai REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(preview_publisher_node
  src/preview_publisher_node.cpp
  src/stereo_pipeline_lib.cpp
)

target_include_directories(preview_publisher_node PRIVATE include)
ament_target_dependencies(preview_publisher_node
  rclcpp sensor_msgs std_msgs image_transport cv_bridge
)
target_link_libraries(preview_publisher_node
  depthai::core
  ${OpenCV_LIBRARIES}
)

add_executable(median_filter_node
  src/median_filter_node.cpp
)

ament_target_dependencies(median_filter_node
  rclcpp sensor_msgs image_transport cv_bridge
)
target_link_libraries(median_filter_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  preview_publisher_node
  median_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


cmake_minimum_required(VERSION 3.16)
project(oakd_pcloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgproc highgui)
find_package(depthai REQUIRED)
find_package(message_filters REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(
	include
	${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
	src/stereo_pipeline_lib.cpp
)

target_include_directories(${PROJECT_NAME}_lib PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}_lib
	rclcpp
	sensor_msgs
	std_msgs
	image_transport
	cv_bridge
	OpenCV
	depthai
	message_filters
	camera_info_manager
)

add_library(wls_filter_node SHARED src/nodelet/wls_filter_nodelet.cpp)
target_compile_features(wls_filter_node PRIVATE cxx_std_17)
ament_target_dependencies(wls_filter_node rclcpp rclcpp_components sensor_msgs cv_bridge image_transport OpenCV message_filters)

add_library(median_filter_node SHARED src/nodelet/median_filter_nodelet.cpp)
target_compile_features(median_filter_node PRIVATE cxx_std_17)
ament_target_dependencies(median_filter_node rclcpp rclcpp_components sensor_msgs cv_bridge image_transport OpenCV message_filters)

add_executable(stereo_rectified_rgb_node src/stereo_rectified_rgb_publisher.cpp)
ament_target_dependencies(stereo_rectified_rgb_node rclcpp sensor_msgs cv_bridge OpenCV depthai camera_info_manager)

install(TARGETS
	${PROJECT_NAME}_lib
	wls_filter_node
	median_filter_node
	stereo_rectified_rgb_node
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(DIRECTORY include/
	DESTINATION include
)

ament_package()
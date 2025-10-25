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
find_package(cv_bridge QUIET)
find_package(OpenCV REQUIRED COMPONENTS imgproc highgui)
find_package(depthai QUIET)

# Option to build without DepthAI (useful for CI or machines without the device)
option(BUILD_WITHOUT_DEPTHAI "Build package without DepthAI support" OFF)

# Global compile-time flags to let sources know whether DepthAI or cv_bridge are available
if(depthai_FOUND AND NOT BUILD_WITHOUT_DEPTHAI)
	add_compile_definitions(OAKD_USE_DEPTHAI)
else()
	add_compile_definitions(OAKD_NO_DEPTHAI)
endif()

if(cv_bridge_FOUND)
	add_compile_definitions(OAKD_USE_CVBRIDGE)
else()
	add_compile_definitions(OAKD_NO_CVBRIDGE)
endif()

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

# Build dependency list conditionally
set(OAKD_COMMON_DEPS
		rclcpp
		sensor_msgs
		std_msgs
		image_transport
		OpenCV
		message_filters
		camera_info_manager
)
if(cv_bridge_FOUND)
	list(APPEND OAKD_COMMON_DEPS cv_bridge)
endif()
if(depthai_FOUND AND NOT BUILD_WITHOUT_DEPTHAI)
	list(APPEND OAKD_COMMON_DEPS depthai)
endif()

ament_target_dependencies(${PROJECT_NAME}_lib ${OAKD_COMMON_DEPS})

add_library(wls_filter_node SHARED src/nodelet/wls_filter_nodelet.cpp)
target_compile_features(wls_filter_node PRIVATE cxx_std_17)
ament_target_dependencies(wls_filter_node rclcpp rclcpp_components sensor_msgs image_transport OpenCV message_filters)
if(cv_bridge_FOUND)
	ament_target_dependencies(wls_filter_node cv_bridge)
endif()

add_library(median_filter_node SHARED src/nodelet/median_filter_nodelet.cpp)
target_compile_features(median_filter_node PRIVATE cxx_std_17)
ament_target_dependencies(median_filter_node rclcpp rclcpp_components sensor_msgs image_transport OpenCV message_filters)
if(cv_bridge_FOUND)
	ament_target_dependencies(median_filter_node cv_bridge)
endif()

add_executable(stereo_rectified_rgb_node src/stereo_rectified_rgb_publisher.cpp)
ament_target_dependencies(stereo_rectified_rgb_node rclcpp sensor_msgs OpenCV camera_info_manager)
if(cv_bridge_FOUND)
	ament_target_dependencies(stereo_rectified_rgb_node cv_bridge)
endif()
if(depthai_FOUND AND NOT BUILD_WITHOUT_DEPTHAI)
	ament_target_dependencies(stereo_rectified_rgb_node depthai)
endif()

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
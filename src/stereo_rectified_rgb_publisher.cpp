
// ROS2 port of the stereo rectified RGB publisher node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <oakd_pcloud/stereo_pipeline.hpp>
// Include DepthAI header only if build flag set and header exists
#if defined(OAKD_USE_DEPTHAI) && __has_include(<depthai/depthai.hpp>)
#include <depthai/depthai.hpp>
#else
#define OAKD_NO_DEPTHAI_RUNTIME
#endif
#include <memory>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_rectified_rgb_node");

    // Declare parameters with defaults
    // Camera identification
    node->declare_parameter("camera_name", "OAK-D");
    node->declare_parameter("camera_param_uri", "file://${ROS_HOME}/camera_info/${camera_name}.yaml");
    
    // OAK-D specific parameters
    node->declare_parameter("fps", 30.0);
    node->declare_parameter("color_width", 1280);
    node->declare_parameter("color_height", 720);
    node->declare_parameter("mono_width", 640);
    node->declare_parameter("mono_height", 400);
    node->declare_parameter("confidence_threshold", 200);
    node->declare_parameter("enable_depth", true);
    node->declare_parameter("enable_rectified", true);
    node->declare_parameter("enable_lr_check", true);
    node->declare_parameter("enable_subpixel", true);
    node->declare_parameter("enable_extended_disparity", false);
    
    // Get parameters
    std::string deviceName = node->get_parameter("camera_name").as_string();
    std::string camera_param_uri = node->get_parameter("camera_param_uri").as_string();
    
    // Get OAK-D configuration
    double fps = node->get_parameter("fps").as_double();
    int color_width = node->get_parameter("color_width").as_int();
    int color_height = node->get_parameter("color_height").as_int();
    int mono_width = node->get_parameter("mono_width").as_int();
    int mono_height = node->get_parameter("mono_height").as_int();
    int confidence = node->get_parameter("confidence_threshold").as_int();
    bool enable_depth = node->get_parameter("enable_depth").as_bool();
    bool enable_rectified = node->get_parameter("enable_rectified").as_bool();
    bool enable_lr_check = node->get_parameter("enable_lr_check").as_bool();
    bool enable_subpixel = node->get_parameter("enable_subpixel").as_bool();
    bool enable_extended = node->get_parameter("enable_extended_disparity").as_bool();
    
    RCLCPP_INFO(node->get_logger(), "Starting with camera_name: %s", deviceName.c_str());
    RCLCPP_INFO(node->get_logger(), "Camera params URI: %s", camera_param_uri.c_str());
    RCLCPP_INFO(node->get_logger(), "Color resolution: %dx%d @ %0.1f FPS", color_width, color_height, fps);
    RCLCPP_INFO(node->get_logger(), "Mono resolution: %dx%d", mono_width, mono_height);
    RCLCPP_INFO(node->get_logger(), "Depth enabled: %s, Rectified: %s", enable_depth ? "yes" : "no", enable_rectified ? "yes" : "no");

    StereoExampe stero_pipeline;
    stero_pipeline.initDepthaiDev();
    #if defined(OAKD_USE_DEPTHAI) && __has_include(<depthai/depthai.hpp>)
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stero_pipeline.getExposedImageStreams();
    RCLCPP_INFO(node->get_logger(), "Data queue sizes: %zu", imageDataQueues.size());
    #else
    std::vector<std::shared_ptr<void>> imageDataQueues; (void)imageDataQueues;
    RCLCPP_WARN(node->get_logger(), "DepthAI not available: running in limited mode (no camera input). Set BUILD_WITHOUT_DEPTHAI=OFF and install ros-jazzy-depthai-ros to enable full functionality.");
    #endif

    // NOTE: The original used depthai_bridge bridge publishers. If a ROS2 depthai_bridge is
    // available with the same API, adapt here. For now we create basic publishers as placeholders.

    auto depth_pub = node->create_publisher<sensor_msgs::msg::Image>("stereo/depth", 10);
    auto left_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("rectified_left/image", 10);
    auto right_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("rectified_right/image", 10);
    auto rgb_pub = node->create_publisher<sensor_msgs::msg::Image>("color/image", 10);

    // A minimal publishing thread for preview/color stream. In a full port we would
    // use depthai_bridge or its ROS2 equivalent to convert dai::ImgFrame to ROS messages.
    #if defined(OAKD_USE_DEPTHAI) && __has_include(<depthai/depthai.hpp>)
    std::thread pub_thread([node, &imageDataQueues, depth_pub, left_rect_pub, right_rect_pub, rgb_pub]() {
        rclcpp::Rate rate(30);
        while (rclcpp::ok()) {
            // attempt to get preview frame (index 5) as an example
            if (imageDataQueues.size() > 5 && imageDataQueues[5]) {
                auto opt = imageDataQueues[5]->tryGet<dai::ImgFrame>(std::chrono::milliseconds(10));
                if (opt) {
                    // convert to sensor_msgs::msg::Image minimal
                    sensor_msgs::msg::Image msg;
                    msg.header.stamp = node->now();
                    msg.header.frame_id = "color_frame";
                    msg.height = opt->getHeight();
                    msg.width = opt->getWidth();
                    msg.encoding = "rgb8";
                    msg.step = opt->getWidth() * 3;
                    auto data = opt->getData();
                    msg.data.assign(data.begin(), data.end());
                    rgb_pub->publish(msg);
                }
            }
            rate.sleep();
        }
    });
    #else
    // No-op thread when DepthAI is not available
    std::thread pub_thread;
    #endif

    rclcpp::spin(node);
    rclcpp::shutdown();
    if (pub_thread.joinable()) pub_thread.join();
    return 0;
}


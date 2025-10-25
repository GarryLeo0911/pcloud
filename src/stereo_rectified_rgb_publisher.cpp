
// ROS2 port of the stereo rectified RGB publisher node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <oakd_pcloud/stereo_pipeline.hpp>
#include <depthai/depthai.hpp>
#include <memory>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_rectified_rgb_node");

    std::string deviceName;
    std::string camera_param_uri;
    if (!node->get_parameter("camera_name", deviceName) || !node->get_parameter("camera_param_uri", camera_param_uri)) {
        RCLCPP_ERROR(node->get_logger(), "Missing required parameters: camera_name or camera_param_uri");
        return 1;
    }

    StereoExampe stero_pipeline;
    stero_pipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stero_pipeline.getExposedImageStreams();

    RCLCPP_INFO(node->get_logger(), "Data queue sizes: %zu", imageDataQueues.size());

    // NOTE: The original used depthai_bridge bridge publishers. If a ROS2 depthai_bridge is
    // available with the same API, adapt here. For now we create basic publishers as placeholders.

    auto depth_pub = node->create_publisher<sensor_msgs::msg::Image>("stereo/depth", 10);
    auto left_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("rectified_left/image", 10);
    auto right_rect_pub = node->create_publisher<sensor_msgs::msg::Image>("rectified_right/image", 10);
    auto rgb_pub = node->create_publisher<sensor_msgs::msg::Image>("color/image", 10);

    // A minimal publishing thread for preview/color stream. In a full port we would
    // use depthai_bridge or its ROS2 equivalent to convert dai::ImgFrame to ROS messages.
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

    rclcpp::spin(node);
    rclcpp::shutdown();
    if (pub_thread.joinable()) pub_thread.join();
    return 0;
}


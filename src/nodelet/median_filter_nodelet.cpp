// Converted to ROS2 rclcpp component
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
// Include DepthAI header only if build-time flag set and header is present
#if defined(OAKD_USE_DEPTHAI) && __has_include(<depthai/depthai.hpp>)
#include <depthai/depthai.hpp>
#else
#define OAKD_NO_DEPTHAI_RUNTIME
#endif

namespace oakd_pcloud
{
using std::placeholders::_1;
using std::placeholders::_2;

class MedianFilterNode : public rclcpp::Node
{
public:
    explicit MedianFilterNode(const rclcpp::NodeOptions & options)
  : Node("median_filter_node", options), kernel_size_(3), blur_iterations_(3)
  {
    // parameters
    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<bool>("exact_sync", false);
    this->declare_parameter<int>("kernel_size", kernel_size_);
    this->declare_parameter<int>("blur_iterations", blur_iterations_);

    auto qos = rclcpp::QoS(10);

    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("median_depth", qos.get_rmw_qos_profile());

    // parameter callback
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MedianFilterNode::on_parameter_changed, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameter_changed(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      if (p.get_name() == "kernel_size") {
        int v = p.as_int();
        if (v == 3 || v == 5) kernel_size_ = v;
        else {
          result.successful = false;
          result.reason = "kernel_size must be 3 or 5";
        }
      } else if (p.get_name() == "blur_iterations") {
        blur_iterations_ = p.as_int();
      }
    }
    return result;
  }

  int kernel_size_;
  int blur_iterations_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

} // namespace oakd_pcloud

RCLCPP_COMPONENTS_REGISTER_NODE(oakd_pcloud::MedianFilterNode)
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
#include <opencv2/ximgproc/disparity_filter.hpp>
// Include DepthAI header only if build-time flag set and header is present
#if defined(OAKD_USE_DEPTHAI) && __has_include(<depthai/depthai.hpp>)
#include <depthai/depthai.hpp>
#else
#define OAKD_NO_DEPTHAI_RUNTIME
#endif

namespace oakd_pcloud
{
using std::placeholders::_1;

class WlsFilterNode : public rclcpp::Node
{
public:
  explicit WlsFilterNode(const rclcpp::NodeOptions & options)
  : Node("wls_filter_node", options)
  {
    wls_filter_ = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    wls_filter_->setLambda(16400.0);
    wls_filter_->setSigmaColor(2.6);

    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<bool>("exact_sync", false);

    pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("wls_depth", 10);
  }

private:
  cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
};

} // namespace oakd_pcloud

RCLCPP_COMPONENTS_REGISTER_NODE(oakd_pcloud::WlsFilterNode)
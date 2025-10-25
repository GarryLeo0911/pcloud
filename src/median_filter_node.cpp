// ROS 2 median filter node (simple image -> image)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

class MedianFilterNode : public rclcpp::Node {
public:
  MedianFilterNode() : rclcpp::Node("median_filter") {
    input_topic_ = this->declare_parameter<std::string>("input_image_topic", "image");
    output_topic_ = this->declare_parameter<std::string>("output_image_topic", "image_filtered");
    kernel_size_ = this->declare_parameter<int>("kernel_size", 3);
    iterations_ = this->declare_parameter<int>("iterations", 1);

    // subscription & publisher
    sub_ = image_transport::create_subscription(
        this, input_topic_,
        std::bind(&MedianFilterNode::image_cb, this, std::placeholders::_1),
        "raw");
    pub_ = image_transport::create_publisher(this, output_topic_);

    // dynamic params
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&MedianFilterNode::on_params, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult on_params(
      const std::vector<rclcpp::Parameter> &params) {
    for (const auto &p : params) {
      if (p.get_name() == "kernel_size" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int v = p.as_int();
        if (v < 1) v = 1;
        if (v % 2 == 0) v += 1; // kernel must be odd
        kernel_size_ = v;
      } else if (p.get_name() == "iterations" && p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int v = p.as_int();
        if (v < 1) v = 1;
        iterations_ = v;
      }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      // Try common encodings; prefer mono8, fallback to bgr8
      if (msg->encoding == "mono8") {
        cv_ptr = cv_bridge::toCvShare(msg, "mono8");
      } else if (msg->encoding == "bgr8" || msg->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      } else {
        // Best effort: convert to mono8
        auto tmp = cv_bridge::toCvShare(msg, msg->encoding);
        cv::Mat gray;
        if (tmp->image.channels() == 3) {
          cv::cvtColor(tmp->image, gray, cv::COLOR_BGR2GRAY);
        } else if (tmp->image.channels() == 1) {
          gray = tmp->image;
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported encoding '%s'", msg->encoding.c_str());
          return;
        }
        cv_ptr = cv_bridge::CvImage(msg->header, "mono8", gray).toCvShare();
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    cv::Mat out = cv_ptr->image.clone();
    for (int i = 0; i < iterations_; ++i) {
      cv::medianBlur(out, out, kernel_size_);
    }

    auto out_msg = cv_bridge::CvImage(msg->header, cv_ptr->encoding, out).toImageMsg();
    pub_.publish(out_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  int kernel_size_;
  int iterations_;

  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MedianFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

